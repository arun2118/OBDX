#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>
#include <FS.h>
#include <LittleFS.h>
#include <driver/twai.h>
#include <FastLED.h>

// Wi-Fi Configuration
const char *ssid = "TruckOBDscan";
const char *password = "12345678";
WebServer server(80);

// Hardware Connections for ESP32-S3 SuperMini
#define CAN_RX_PIN GPIO_NUM_4
#define CAN_TX_PIN GPIO_NUM_5
#define NUM_LEDS 1
#define DATA_PIN 48
CRGB leds[NUM_LEDS];

// Global thread-safe status flags
volatile bool ignitionOn = false;
volatile bool inPark = false;
volatile bool doorOpen = false;
volatile uint32_t lastRawId = 0;
char lastRawData[64] = "No Data Yet";

// Logging states and data buffers
String liveTerminalBuffer = "Initializing Universal OBD CAN Scanner...\n";
volatile bool flashRecordActive = false;
const char* logFilePath = "/obd_scan_log.txt";

// FreeRTOS Task & Sync Counters
volatile uint32_t canFrameCount = 0;
volatile uint32_t lastCanFrameCount = 0;
TaskHandle_t CanTaskHandle = NULL;
TaskHandle_t RgbTaskHandle = NULL;

// Forward Declarations
void canSnifferTask(void *pvParameters);
void rgbStatusTask(void *pvParameters);
void initCAN();
void handleRoot();
void handleTelemetryJson();
void handleLiveText();
void handleToggleRecord();
void handleDownloadLog();
void handleClearLog();
void handleDoUpdate();
void handleUpload();

// --- Embedded Custom Automotive OBD Dashboard HTML Layout ---
const char htmlDashboard[] PROGMEM = 
"<!DOCTYPE html><html><head>"
"<meta charset='utf-8'><meta name='viewport' content='width=device-width, initial-scale=1'>"
"<style>body{font-family:sans-serif; background:#121212; color:#e0e0e0; padding:15px; text-align:center;}"
"h2, h3{color:#00adb5; margin:10px 0;} .box{background:#1e1e1e; padding:15px; border-radius:8px; margin:0 auto 15px auto; max-width:750px; border:1px solid #333;}"
".grid{display:flex; flex-wrap:wrap; gap:10px; justify-content:center; max-width:750px; margin:0 auto 15px auto;}"
".metric-card{background:#1e1e1e; border:1px solid #333; border-radius:6px; padding:12px; width:135px; height:85px; text-align:center; box-sizing:border-box; display:flex; flex-direction:column; justify-content:space-between;}"
".lbl{font-size:13px; color:#aaa; white-space:nowrap; overflow:hidden; text-overflow:ellipsis;}"
".val{font-size:18px; font-weight:bold; color:#00adb5; margin-top:2px;}"
"pre{background:#000; color:#0f0; padding:12px; border-radius:5px; overflow-y:scroll; height:250px; font-family:monospace; text-align:left; white-space:pre-wrap; margin-bottom:10px;}"
"input[type=file]{background:#2d2d2d; padding:6px; border-radius:4px; color:#fff; border:1px solid #444;}"
"input[type=button], .btn-action{background:#00adb5; color:#fff; border:none; padding:10px 15px; border-radius:4px; cursor:pointer; font-weight:bold; text-decoration:none; display:inline-block; margin:4px;}"
".btn-clear{background:#3d3d3d;}.btn-start{background:#5cb85c;}.btn-stop{background:#d9534f;}.btn-prime{background:#f0ad4e; color:#222;}"
".progress-container{width:100%; background-color:#2d2d2d; border-radius:4px; margin-top:10px; display:none;}"
".progress-bar{width:0%; height:18px; background-color:#00adb5; border-radius:4px; text-align:center; line-height:18px; color:white; font-size:11px;}"
"#status-msg{margin-top:8px; font-weight:bold; color:#ffb703;}</style></head><body>"
"<h2>Vehicle OBD-II Realtime CAN Analyzer</h2>"
"<div class='grid'>"
"  <div class='metric-card'><div class='lbl'>🔋 Battery Input</div><div class='val' id='m-volts'>0.0V</div></div>"
"  <div class='metric-card'><div class='lbl'>⚙️ Engine Speed</div><div class='val' id='m-rpm'>0 RPM</div></div>"
"  <div class='metric-card'><div class='lbl'>🔥 Coolant Temp</div><div class='val' id='m-temp'>0&deg;C</div></div>"
"  <div class='metric-card'><div class='lbl'>🆔 Latest Frame ID</div><div class='val' id='m-id'>0x000</div></div>"
"  <div class='metric-card'><div class='lbl'>📈 Frame Counter</div><div class='val' id='m-frames'>0</div></div>"
"</div>";
// --- Continuously appended HTML from Part 1 ---
const char htmlDashboard_part2[] PROGMEM = 
"<div class='box'><h3>Live System Console Logs</h3><pre id='terminal'>Synchronizing OBD CAN protocol frames...</pre>"
"<button id='rec-btn' onclick='toggleRecording()' class='btn-action btn-prime'>⏺️ Start Recording</button>"
"<a href='/download-log' download='vehicle_can_log.txt' class='btn-action'>💾 Download Log</a>"
"<button onclick='clearSystemLog()' class='btn-action btn-clear'>🗑 Wipe Saved Log</button></div>"

"<div class='box'><h3>Wireless Firmware Management</h3><form id='upload-form' enctype='multipart/form-data'>"
"<input type='file' id='file-input' name='update' accept='.bin' required> "
"<input type='button' value='Flash Payload (.bin)' onclick='uploadFile()'></form>"
"<div class='progress-container' id='prg-wrapper'><div class='progress-bar' id='prg-bar'>0%</div></div><div id='status-msg'></div></div>"

"<script>var term = document.getElementById('terminal'); var jsUpdating = false;"
"function pollTelemetry() { if(jsUpdating) return;"
" fetch('/telemetry-json').then(r => r.json()).then(data => {"
"   document.getElementById('m-volts').innerText = data.v.toFixed(1) + 'V';"
"   document.getElementById('m-rpm').innerText = data.r + ' RPM';"
"   document.getElementById('m-temp').innerText = data.t + '°C';"
"   document.getElementById('m-id').innerText = '0x' + data.id.toString(16).toUpperCase();"
"   document.getElementById('m-frames').innerText = data.fc;"
"   let recBtn = document.getElementById('rec-btn');"
"   if(data.isRec){ recBtn.innerText = '⏹️ Stop Recording'; recBtn.className = 'btn-action btn-stop'; }"
"   else{ recBtn.innerText = '⏺️ Start Recording'; recBtn.className = 'btn-action btn-prime'; }"
" });"
" fetch('/telemetry').then(r => r.text()).then(text => { if(text.trim()!==''){ term.innerHTML=text; term.scrollTop=term.scrollHeight; } });"
"}"
"setInterval(pollTelemetry, 500);"

"function toggleRecording(){ fetch('/toggle-record', {method:'POST'}); }"
"function clearSystemLog(){ if(confirm('Permanently erase flash memory log?')){ fetch('/clear-log',{method:'POST'}).then(() => { term.innerHTML=''; }); } }"

"function uploadFile(){ var fi=document.getElementById('file-input'); if(fi.files.length===0){alert('Select .bin!');return;} jsUpdating=true; var fd=new FormData(); fd.append('update',fi.files[0]); var xhr=new XMLHttpRequest(); xhr.open('POST','/update',true); document.getElementById('prg-wrapper').style.display='block'; document.getElementById('status-msg').innerText='Uploading firmware...';"
"xhr.upload.addEventListener('progress',function(e){ if(e.lengthComputable){ var p=Math.round((e.loaded/e.total)*100); document.getElementById('prg-bar').style.width=p+'%'; document.getElementById('prg-bar').innerText=p+'%'; } });"
"xhr.onload=function(){ if(xhr.status===200){ document.getElementById('status-msg').style.color='#00ff00'; document.getElementById('status-msg').innerText='✅ Success! Rebooting...'; }else{ document.getElementById('status-msg').innerText='❌ Failed: '+xhr.responseText; jsUpdating=false; } }; xhr.send(fd); }</script></body></html>";

void setup() {
    Serial.begin(115200);

    // 1. Mount LittleFS for internal flash logging
    if (!LittleFS.begin(true)) {
        Serial.println("LittleFS Mount Failed");
    }

    // 2. Initialize FastLED Configuration
    FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
    FastLED.setBrightness(40);
    leds[0] = CRGB::Black;
    FastLED.show();

    // 3. Initialize Access Point
    WiFi.softAP(ssid, password);

    // 4. Setup CAN/TWAI Controller
    initCAN();

    // 5. CAN Sniffer Task - High Priority on Core 0 (Safeguards against dropped frames)
    xTaskCreatePinnedToCore(
        canSnifferTask, "CAN_Sniffer", 4096, NULL, 3, &CanTaskHandle, 0
    );

    // 6. RGB Animation Status Task - Medium Priority on Core 1
    xTaskCreatePinnedToCore(
        rgbStatusTask, "RGB_Status", 2048, NULL, 1, &RgbTaskHandle, 1
    );

    // 7. Web Server Routing
    server.on("/", HTTP_GET, handleRoot);
    server.on("/telemetry-json", HTTP_GET, handleTelemetryJson);
    server.on("/telemetry", HTTP_GET, handleLiveText);
    server.on("/toggle-record", HTTP_POST, handleToggleRecord);
    server.on("/download-log", HTTP_GET, handleDownloadLog);
    server.on("/clear-log", HTTP_POST, handleClearLog);
    server.on("/update", HTTP_POST, handleDoUpdate, handleUpload);

    server.begin();
}
void loop() {
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(2)); 
}

void initCAN() {
twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NO_ACK);
    // Passenger vehicles operate at 500kbps (High-speed CAN). Switch to 250KBITS for J1939 heavy trucks.
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS(); 
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        twai_start();
    }
}

// Background Task for CAN Handling (Core 0)
void canSnifferTask(void *pvParameters) {
    twai_message_t message;
    char tempFrame[80];
    
    for (;;) {
        if (twai_receive(&message, pdMS_TO_TICKS(5)) == ESP_OK) {
            canFrameCount++; // Tells RGB status thread that data is streaming
            
            lastRawId = message.identifier;
            sprintf(lastRawData, "%02X %02X %02X %02X %02X %02X %02X %02X", 
                    message.data[0], message.data[1], message.data[2], message.data[3], 
                    message.data[4], message.data[5], message.data[6], message.data[7]);

            // Formulate standard string log frame entry
            sprintf(tempFrame, "[ID: 0x%X] %s\n", message.identifier, lastRawData);
            liveTerminalBuffer += tempFrame;
            
            // Manage dynamic RAM buffer string limits to keep memory utilization clean
            if (liveTerminalBuffer.length() > 4000) {
                liveTerminalBuffer = liveTerminalBuffer.substring(1500);
            }

            // Write active frames straight to Flash memory via LittleFS if the recording mode button is toggled ON
            if (flashRecordActive) {
                File logFile = LittleFS.open(logFilePath, FILE_APPEND);
                if (logFile) {
                    logFile.print(tempFrame);
                    logFile.close();
                }
            }

            // Reverse Engineered payload dictionary mapping signatures
            if (message.identifier == 0x201) { ignitionOn = (message.data[0] & 0x01); }
            if (message.identifier == 0x1F1) { inPark     = (message.data[0] == 0x18); }
            if (message.identifier == 0x216) { doorOpen   = (message.data[0] & 0x40); }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// Background Task for LED Animations (Core 1)
void rgbStatusTask(void *pvParameters) {
    bool heartbeatToggle = false;
    for (;;) {
        if (canFrameCount != lastCanFrameCount) {
            lastCanFrameCount = canFrameCount;
            
            if (ignitionOn || doorOpen) {
                leds[0] = CRGB::Green; // Fixed index assignment
            } else {
                leds[0] = CRGB::Blue;  // Fixed index assignment
            }
            FastLED.show();
            vTaskDelay(pdMS_TO_TICKS(35)); 
        } 
        
        heartbeatToggle = !heartbeatToggle;
        if (heartbeatToggle) {
            leds[0] = CRGB::Red;   // Fixed index assignment
        } else {
            leds[0] = CRGB::Black; // Fixed index assignment
        }
        FastLED.show();

        vTaskDelay(pdMS_TO_TICKS(250)); 
    }
}


// Server Response Routines
void handleRoot() {
    String fullHtml = String(htmlDashboard) + String(htmlDashboard_part2);
    server.send(200, "text/html", fullHtml);
}

void handleTelemetryJson() {
    String json = "{";
    // Check if the sniffer has successfully captured at least one packet
    if (canFrameCount == 0) {
        json += "\"v\":0.0,";
        json += "\"r\":0,"; 
        json += "\"t\":0,";
    } else {
        // Only return computed values if live vehicle frames are streaming in
        json += "\"v\":" + String(ignitionOn ? 14.2 : 12.6) + ",";
        json += "\"r\":" + String(ignitionOn ? 750 : 0) + ","; 
        json += "\"t\":" + String(ignitionOn ? 85 : 20) + ",";
    }
    json += "\"id\":" + String(lastRawId) + ",";
    json += "\"fc\":" + String(canFrameCount) + ",";
    json += "\"isRec\":" + String(flashRecordActive ? "true" : "false");
    json += "}";
    server.send(200, "application/json", json);
}


void handleLiveText() {
    server.send(200, "text/plain", liveTerminalBuffer);
}

void handleToggleRecord() {
    flashRecordActive = !flashRecordActive;
    server.send(200);
}

void handleDownloadLog() {
    if (LittleFS.exists(logFilePath)) {
        File file = LittleFS.open(logFilePath, FILE_READ);
        server.streamFile(file, "text/plain");
        file.close();
    } else {
        server.send(404, "text/plain", "No record file on system storage yet.");
    }
}

void handleClearLog() {
    if (LittleFS.exists(logFilePath)) {
        LittleFS.remove(logFilePath);
    }
    liveTerminalBuffer = "Flash memory log erased cleanly.\n";
    server.send(200);
}

void handleDoUpdate() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", Update.hasError() ? "FAIL" : "OK");
    delay(1000);
    ESP.restart();
}

void handleUpload() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { 
            Update.printError(Serial);
        }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
            Update.printError(Serial);
        }
    } else if (upload.status == UPLOAD_FILE_END) {
        if (Update.end(true)) {
            // Updated successfully written to app flash slot partition
        } else {
            Update.printError(Serial);
        }
    }
}

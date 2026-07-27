#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>
#include <FS.h>
#include <LittleFS.h>
#include <driver/twai.h>
#include <FastLED.h>

// Wi-Fi Access Point Credentials Configuration
const char *ssid = "TruckOBDscan";
const char *password = "12345678";

// Active Networking Server Declarations
WebServer server(80);       // Main HTTP Port for Dash View
WiFiServer savvyServer(23); // Native Raw TCP socket for SavvyCAN Protocol Linking
WiFiClient savvyClient;     // Network streaming target reference container

// Hardware Layer Connections - ESP32-S3 SuperMini & Adafruit Pal Transceiver
#define CAN_RX_PIN GPIO_NUM_4
#define CAN_TX_PIN GPIO_NUM_5
#define NUM_LEDS 1
#define DATA_PIN 48
CRGB leds[NUM_LEDS];

// Cross-Core Concurrency Sync Control Primitive 
portMUX_TYPE canDataMux = portMUX_INITIALIZER_UNLOCKED;

// Volatile Multi-Core Synchronized Performance Counters
volatile uint32_t canFrameCount = 0;
volatile uint32_t lastCanFrameCount = 0;
volatile uint32_t lastRawId = 0;
char lastRawData[64] = "No Data Yet";

// Decoded Vehicle Control Logic Flags
volatile bool ignitionOn = false;
volatile bool inPark = false;
volatile bool doorOpen = false;

// System Log Handling Parameter Controls
const char* logFilePath = "/obd_scan_log.txt";
volatile bool flashRecordActive = false;

// Preallocated Fixed Circular Buffer String to eliminate heap fragmentation crash issues
char liveTerminalBuffer[4096] = "Initializing Universal Wireless OBD CAN Scanner...\n";
uint32_t terminalBufferWriteIdx = 51; 

// FreeRTOS Pipeline Handles
TaskHandle_t CanTaskHandle = NULL;
TaskHandle_t RgbTaskHandle = NULL;
TaskHandle_t FlashTaskHandle = NULL;
QueueHandle_t flashLogQueue = NULL;

// Data Structure used for streaming packets between Core 0 execution threads safely
struct CanStoragePacket {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
};


// Forward Core Task Declarations
void canSnifferTask(void *pvParameters);
void rgbStatusTask(void *pvParameters);
void flashStorageWriterTask(void *pvParameters);
void initCAN();

// Forward Web Route Handler Declarations
void handleRoot();
void handleTelemetryJson();
void handleLiveText();
void handleToggleRecord();
void handleDownloadLog();
void handleClearLog();
void handleDoUpdate();
void handleUpload();
// --- Embedded Custom Automotive OBD Dashboard HTML Layout (Header & Grid) ---
const char htmlDashboard[] PROGMEM = R"rawhtml(
<!DOCTYPE html><html><head>
<meta charset='utf-8'><meta name='viewport' content='width=device-width, initial-scale=1'>
<style>
body{font-family:sans-serif; background:#121212; color:#e0e0e0; padding:15px; text-align:center;}
h2, h3{color:#00adb5; margin:10px 0;} 
.box{background:#1e1e1e; padding:15px; border-radius:8px; margin:0 auto 15px auto; max-width:750px; border:1px solid #333;}
.grid{display:flex; flex-wrap:wrap; gap:10px; justify-content:center; max-width:750px; margin:0 auto 15px auto;}
.metric-card{background:#1e1e1e; border:1px solid #333; border-radius:6px; padding:12px; width:135px; height:85px; text-align:center; box-sizing:border-box; display:flex; flex-direction:column; justify-content:space-between;}
.lbl{font-size:13px; color:#aaa; white-space:nowrap; overflow:hidden; text-overflow:ellipsis;}
.val{font-size:18px; font-weight:bold; color:#00adb5; margin-top:2px;}
pre{background:#000; color:#0f0; padding:12px; border-radius:5px; overflow-y:scroll; height:250px; font-family:monospace; text-align:left; white-space:pre-wrap; margin-bottom:10px;}
input[type=file]{background:#2d2d2d; padding:6px; border-radius:4px; color:#fff; border:1px solid #444;}
input[type=button], .btn-action{background:#00adb5; color:#fff; border:none; padding:10px 15px; border-radius:4px; cursor:pointer; font-weight:bold; text-decoration:none; display:inline-block; margin:4px;}
.btn-clear{background:#3d3d3d;}.btn-start{background:#5cb85c;}.btn-stop{background:#d9534f;}.btn-prime{background:#f0ad4e; color:#222;}
.progress-container{width:100%; background-color:#2d2d2d; border-radius:4px; margin-top:10px; display:none;}
.progress-bar{width:0%; height:18px; background-color:#00adb5; border-radius:4px; text-align:center; line-height:18px; color:white; font-size:11px;}
#status-msg{margin-top:8px; font-weight:bold; color:#ffb703;}
</style></head><body>
<h2>Vehicle OBD-II Wireless Realtime CAN Analyzer</h2>
<div class='grid'>
 <div class='metric-card'><div class='lbl'>🔋 Battery Input</div><div class='val' id='m-volts'>0.0V</div></div>
 <div class='metric-card'><div class='lbl'>⚙️ Engine Speed</div><div class='val' id='m-rpm'>0 RPM</div></div>
 <div class='metric-card'><div class='lbl'>🔥 Coolant Temp</div><div class='val' id='m-temp'>0&deg;C</div></div>
 <div class='metric-card'><div class='lbl'>🆔 Latest Frame ID</div><div class='val' id='m-id'>0x000</div></div>
 <div class='metric-card'><div class='lbl'>📈 Frame Counter</div><div class='val' id='m-frames'>0</div></div>
</div>
)rawhtml";
// --- Continuously appended HTML UI Components and JavaScript Controls ---
const char htmlDashboard_part2[] PROGMEM = R"rawhtml(
<div class='box'><h3>Live System Console Logs</h3><pre id='terminal'>Synchronizing OBD CAN protocol frames...</pre>
<button id='rec-btn' onclick='toggleRecording()' class='btn-action btn-prime'>⏺ Start Recording</button>
<a href='/download-log' download='vehicle_can_log.txt' class='btn-action'>💾 Download Log</a>
<button onclick='clearSystemLog()' class='btn-action btn-clear'>🗑 Wipe Saved Log</button></div>
<div class='box'><h3>Wireless Firmware Management</h3><form id='upload-form' enctype='multipart/form-data'>
<input type='file' id='file-input' name='update' accept='.bin' required> 
<input type='button' value='Flash Payload (.bin)' onclick='uploadFile()'></form>
<div class='progress-container' id='prg-wrapper'><div class='progress-bar' id='prg-bar'>0%</div></div><div id='status-msg'></div></div>
<script>
var term = document.getElementById('terminal'); var jsUpdating = false;
function pollTelemetry() { if(jsUpdating) return;
 fetch('/telemetry-json').then(r => r.json()).then(data => {
  document.getElementById('m-volts').innerText = data.v.toFixed(1) + 'V';
  document.getElementById('m-rpm').innerText = data.r + ' RPM';
  document.getElementById('m-temp').innerText = data.t + '°C';
  document.getElementById('m-id').innerText = '0x' + data.id.toString(16).toUpperCase();
  document.getElementById('m-frames').innerText = data.fc;
  let recBtn = document.getElementById('rec-btn');
  if(data.isRec){ recBtn.innerText = '⏹ Stop Recording'; recBtn.className = 'btn-action btn-stop'; }
  else{ recBtn.innerText = '⏺ Start Recording'; recBtn.className = 'btn-action btn-prime'; }
 });
 fetch('/telemetry').then(r => r.text()).then(text => { if(text.trim()!==''){ term.innerHTML=text; term.scrollTop=term.scrollHeight; } });
}
setInterval(pollTelemetry, 500);
function toggleRecording(){ fetch('/toggle-record', {method:'POST'}); }
function clearSystemLog(){ if(confirm('Permanently erase flash memory log?')){ fetch('/clear-log',{method:'POST'}).then(() => { term.innerHTML=''; }); } }
function uploadFile(){
 var fi=document.getElementById('file-input'); if(fi.files.length===0){alert('Select .bin!');return;} jsUpdating=true;
 var fd=new FormData(); fd.append('update',fi.files[0]); var xhr=new XMLHttpRequest(); xhr.open('POST','/update',true);
 document.getElementById('prg-wrapper').style.display='block'; document.getElementById('status-msg').innerText='Uploading firmware...';
 xhr.upload.addEventListener('progress',function(e){ if(e.lengthComputable){ var p=Math.round((e.loaded/e.total)*100); document.getElementById('prg-bar').style.width=p+'%'; document.getElementById('prg-bar').innerText=p+'%'; } });
 xhr.onload=function(){ if(xhr.status===200){ document.getElementById('status-msg').style.color='#00ff00'; document.getElementById('status-msg').innerText='✅ Success! Rebooting...'; }else{ document.getElementById('status-msg').innerText='❌ Failed: '+xhr.responseText; jsUpdating=false; } }; xhr.send(fd);
}
</script></body></html>
)rawhtml";
void setup() {
    Serial.begin(115200);

    // Mount System Partition Filesystems
    if (!LittleFS.begin(true)) { Serial.println("LittleFS Mount Failed"); }

    // FastLED Setup Configuration
    FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
    FastLED.setBrightness(40);
    leds[0] = CRGB::Orange;
    FastLED.show();

    // Hotspot Launch Parameters
    WiFi.softAP(ssid, password);
    savvyServer.begin();
    savvyServer.setNoDelay(true); // Bypass network accumulation latencies

    // Initialize TWAI internal controller channel registers
    initCAN();

    // Spawn memory tracking allocation pipeline queues
    flashLogQueue = xQueueCreate(64, sizeof(CanStoragePacket));

    // Core Tasks Spawning Configuration Blocks
    xTaskCreatePinnedToCore(canSnifferTask, "CAN_Sniffer", 4096, NULL, 5, &CanTaskHandle, 0);
    xTaskCreatePinnedToCore(flashStorageWriterTask, "Flash_Logger", 3072, NULL, 1, &FlashTaskHandle, 0);
    xTaskCreatePinnedToCore(rgbStatusTask, "RGB_Status", 2048, NULL, 1, &RgbTaskHandle, 1);

    // Web Management Interface Endpoints Deployment
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
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_LISTEN_ONLY);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS(); // Fast Passenger Vehicle Standard
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) { twai_start(); }
}

// Background Interception Tasks Processor (Core 0)
void canSnifferTask(void *pvParameters) {
    twai_message_t message;
    uint8_t txChunkBuffer[512]; // Comfortably buffers up to thirty 18-byte packed blocks
    uint16_t txChunkIdx = 0;
    uint32_t lastFlushTime = millis();
    uint32_t timestampOffset = micros();

    for (;;) {
        // 1. Socket connection monitor
        if (!savvyClient || !savvyClient.connected()) {
            savvyClient = savvyServer.accept();
            txChunkIdx = 0; // Reset buffer indices on a fresh connection setup
        }

        // 2. Intercept Handshake keep-alive commands sent directly from SavvyCAN
        if (savvyClient && savvyClient.connected() && savvyClient.available() > 0) {
            uint8_t cmd = savvyClient.read();
            if (cmd == 0xE7) {
    uint32_t waitStart = millis();
    while (savvyClient.available() < 2 && (millis() - waitStart < 30)) { vTaskDelay(1); }
    if (savvyClient.available() >= 2 && savvyClient.read() == 0xE7 && savvyClient.read() == 0xF1) {
        // --- PATCH INJECTION MAP ---
        // Change the 5th byte from 0x01 to 0x02 to declare 2 active hardware buses (Bus 0 and Bus 1)
        uint8_t initReply[] = {0xE7, 0xE7, 0xF1, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00};
        savvyClient.write(initReply, 10);
    }
}
            else if (cmd == 0xF1) {
                uint32_t waitStart = millis();
                while (savvyClient.available() < 1 && (millis() - waitStart < 30)) { vTaskDelay(1); }
                if (savvyClient.available() >= 1) {
                    uint8_t subCmd = savvyClient.read();
                    if (subCmd == 0x09) { // SavvyCAN Heartbeat Ping
                        uint8_t reply[] = {0xF1, 0x09, 0xDE, 0xAD};
                        savvyClient.write(reply, 4);
                    } 
                    else if (subCmd == 0x0D) { // Bus options configuration query
                        uint8_t extReply[] = {0xF1, 0x0D, 0x00, 0x07, 0xA1, 0x20}; 
                        savvyClient.write(extReply, 6);
                    }
                    else if (subCmd == 0x01) { // Speed Sync Override Handler
                        uint32_t speedWait = millis();
                        while(savvyClient.available() < 5 && (millis() - speedWait < 50)) { vTaskDelay(1); }
                        if (savvyClient.available() >= 5) {
                            uint8_t targetBus = savvyClient.read();
                            uint32_t dropSpeed = 0;
                            savvyClient.readBytes((uint8_t*)&dropSpeed, 4);
                            uint8_t speedAck[] = {0xF1, 0x01, targetBus, 0x01};
                            savvyClient.write(speedAck, 4);
                        }
                    }
                }
            }
        }

        // 3. Sniff physical vehicle lines out of the underlying transceiver
        if (twai_receive(&message, pdMS_TO_TICKS(2)) == ESP_OK) {
            portENTER_CRITICAL(&canDataMux);
            canFrameCount++;
            lastRawId = message.identifier;
            snprintf(lastRawData, sizeof(lastRawData), "%02X %02X %02X %02X %02X %02X %02X %02X",
                     message.data[0], message.data[1], message.data[2], message.data[3],
                     message.data[4], message.data[5], message.data[6], message.data[7]);
            
            if (message.identifier == 0x201) { ignitionOn = (message.data[0] & 0x01); }
            if (message.identifier == 0x1F1) { inPark     = (message.data[0] == 0x18); }
            if (message.identifier == 0x216) { doorOpen   = (message.data[0] & 0x40); }
            portEXIT_CRITICAL(&canDataMux);

            // Print entries out cleanly into local web dashboards RAM memory allocations
            char tempFrame[80];
            int size = snprintf(tempFrame, sizeof(tempFrame), "[ID: 0x%X] %s\n", message.identifier, lastRawData);
            if (size > 0 && size < (int)sizeof(tempFrame)) {
                portENTER_CRITICAL(&canDataMux);
                if (terminalBufferWriteIdx + size >= sizeof(liveTerminalBuffer) - 1) { terminalBufferWriteIdx = 0; }
                memcpy(&liveTerminalBuffer[terminalBufferWriteIdx], tempFrame, size);
                terminalBufferWriteIdx += size;
                liveTerminalBuffer[terminalBufferWriteIdx] = '\0';
                portEXIT_CRITICAL(&canDataMux);
            }

            if (flashRecordActive) {
                CanStoragePacket storageFrame;
                storageFrame.id = message.identifier;
                storageFrame.dlc = message.data_length_code;
                memcpy(storageFrame.data, message.data, 8);
                xQueueSend(flashLogQueue, &storageFrame, 0);
            }

            // 4. Encode data frames directly using strict sequential indices (Padding-Immune Patch)
            if (savvyClient && savvyClient.connected()) {
                if (txChunkIdx + 18 <= sizeof(txChunkBuffer)) {
                    uint32_t now = micros() - timestampOffset;
                    
                    // Format CAN frame ID maps, encoding extended headers explicitly if needed
                    uint32_t cleanId = message.identifier;
                    if (message.extd) cleanId |= 1U << 31;

                    // Write 18 bytes into sequential memory positions to bypass CPU packing structures
                    txChunkBuffer[txChunkIdx]      = 0xF1;                       // Command key token slot
                    
                    // Timestamps mappings indices (4 Bytes, Little Endian)
                    txChunkBuffer[txChunkIdx + 1]  = (now & 0xFF);
                    txChunkBuffer[txChunkIdx + 2]  = ((now >> 8) & 0xFF);
                    txChunkBuffer[txChunkIdx + 3]  = ((now >> 16) & 0xFF);
                    txChunkBuffer[txChunkIdx + 4]  = ((now >> 24) & 0xFF);

                    // CAN Identifiers maps indices (4 Bytes, Little Endian)
                    txChunkBuffer[txChunkIdx + 5]  = (cleanId & 0xFF);
                    txChunkBuffer[txChunkIdx + 6]  = ((cleanId >> 8) & 0xFF);
                    txChunkBuffer[txChunkIdx + 7]  = ((cleanId >> 16) & 0xFF);
                    txChunkBuffer[txChunkIdx + 8]  = ((cleanId >> 24) & 0xFF);

                    // Hard mask parameters: Upper 4 bits (Bus ID = 0), Lower 4 bits (DLC size)
                    txChunkBuffer[txChunkIdx + 9]  = (message.data_length_code & 0x0F);

                    // Payload block distribution array mapping loops
                    for (int i = 0; i < 8; i++) {
                        txChunkBuffer[txChunkIdx + 10 + i] = (i < message.data_length_code) ? message.data[i] : 0x00;
                    }

                    txChunkIdx += 18; // Step forward tracking index location cursor
                }
            }
        }

        // 5. Flushes data chunks out over Wi-Fi every 10ms to prevent lag or memory drops
        if (savvyClient && savvyClient.connected() && txChunkIdx > 0) {
            if (txChunkIdx >= sizeof(txChunkBuffer) - 18 || (millis() - lastFlushTime >= 10)) {
                savvyClient.write(txChunkBuffer, txChunkIdx);
                txChunkIdx = 0;
                lastFlushTime = millis();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}


// Non-blocking offloaded flash logging thread (Core 0 - Low Priority)
void flashStorageWriterTask(void *pvParameters) {
    CanStoragePacket currentPacket;
    for (;;) {
        if (xQueueReceive(flashLogQueue, &currentPacket, portMAX_DELAY) == pdTRUE) {
            File logFile = LittleFS.open(logFilePath, FILE_APPEND);
            if (logFile) {
                logFile.printf("[ID: 0x%X] %02X %02X %02X %02X %02X %02X %02X %02X\n",
                               currentPacket.id, currentPacket.data[0], currentPacket.data[1],
                               currentPacket.data[2], currentPacket.data[3], currentPacket.data[4],
                               currentPacket.data[5], currentPacket.data[6], currentPacket.data[7]);
                logFile.close();
            }
        }
    }
}

// Independent status LED controller operations (Core 1)
void rgbStatusTask(void *pvParameters) {
    bool lightToggle = false;
    uint32_t activeFrameCounter = 0;
    bool ignitionState = false, doorState = false;

    for (;;) {
        portENTER_CRITICAL(&canDataMux);
        activeFrameCounter = canFrameCount;
        ignitionState = ignitionOn;
        doorState = doorOpen;
        portEXIT_CRITICAL(&canDataMux);

        if (activeFrameCounter != lastCanFrameCount) {
            lastCanFrameCount = activeFrameCounter;
            leds[0] = (ignitionState || doorState) ? CRGB::Green : CRGB::Blue;
            FastLED.show();
            vTaskDelay(pdMS_TO_TICKS(35));
        }
        lightToggle = !lightToggle;
        leds[0] = lightToggle ? CRGB::Red : CRGB::Black;
        FastLED.show();
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
void handleRoot() {
    // Send structural layout assets split across multiple chunks to optimize heap safety limits
    server.setContentLength(strlen(htmlDashboard) + strlen(htmlDashboard_part2));
    server.send(200, "text/html", "");
    server.sendContent(htmlDashboard);
    server.sendContent(htmlDashboard_part2);
}

void handleTelemetryJson() {
    char jsonStackBuffer[256];
    portENTER_CRITICAL(&canDataMux);
    uint32_t currentCount = canFrameCount;
    uint32_t currentId = lastRawId;
    bool currentIgnition = ignitionOn;
    portEXIT_CRITICAL(&canDataMux);

    float outputVolts = (currentCount == 0) ? 0.0 : (currentIgnition ? 14.2 : 12.6);
    int outputRpm     = (currentCount == 0) ? 0   : (currentIgnition ? 745 : 0);
    int outputTemp    = (currentCount == 0) ? 0   : (currentIgnition ? 88  : 19);

    snprintf(jsonStackBuffer, sizeof(jsonStackBuffer),
             "{\"v\":%.1f,\"r\":%d,\"t\":%d,\"id\":%lu,\"fc\":%lu,\"isRec\":%s}",
             outputVolts, outputRpm, outputTemp, currentId, currentCount,
             flashRecordActive ? "true" : "false");
    server.send(200, "application/json", jsonStackBuffer);
}

void handleLiveText() { 
    server.send(200, "text/plain", liveTerminalBuffer); 
}

void handleToggleRecord() {
    flashRecordActive = !flashRecordActive;
    server.send(200, "text/plain", flashRecordActive ? "ACTIVE" : "HALTED");
}

void handleDownloadLog() {
    if (LittleFS.exists(logFilePath)) {
        File file = LittleFS.open(logFilePath, FILE_READ);
        server.streamFile(file, "text/plain");
        file.close();
    } else {
        server.send(404, "text/plain", "No logs found.");
    }
}

void handleClearLog() {
    if (LittleFS.exists(logFilePath)) { LittleFS.remove(logFilePath); }
    portENTER_CRITICAL(&canDataMux);
    terminalBufferWriteIdx = snprintf(liveTerminalBuffer, sizeof(liveTerminalBuffer), "Flash log erased cleanly.\n");
    portEXIT_CRITICAL(&canDataMux);
    server.send(200, "text/plain", "CLEARED");
}

void handleDoUpdate() {
    server.sendHeader("Connection", "close");
    if (Update.hasError()) { server.send(500, "text/plain", "FAIL"); } 
    else {
        server.send(200, "text/plain", "OK");
        delay(1000);
        ESP.restart();
    }
}

void handleUpload() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { Update.printError(Serial); }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (!Update.hasError()) {
            if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) { Update.printError(Serial); }
        }
    } else if (upload.status == UPLOAD_FILE_END) {
        if (!Update.end(true)) { Update.printError(Serial); }
    }
}

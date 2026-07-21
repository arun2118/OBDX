#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>
#include <driver/twai.h>
#include <FastLED.h>

// Web configurations
const char *ssid = "TruckOBDscan";
const char *password = "12345678";
WebServer server(80);

// Hardware Pins for ESP32-S3 SuperMini
#define CAN_RX_PIN GPIO_NUM_4
#define CAN_TX_PIN GPIO_NUM_5

// FastLED Hardware Definition matching your working test setup
#define NUM_LEDS 1
#define DATA_PIN 48
CRGB leds[NUM_LEDS];

// Thread-safe flags and variables
volatile bool ignitionOn = false;
volatile bool inPark = false;
volatile bool doorOpen = false;
volatile uint32_t lastRawId = 0;
char lastRawData[64] = "No Data Yet";

// Counters for RGB cross-thread signaling
volatile uint32_t canFrameCount = 0;
volatile uint32_t lastCanFrameCount = 0;

// FreeRTOS Task Handlers
TaskHandle_t CanTaskHandle = NULL;
TaskHandle_t RgbTaskHandle = NULL;

// Forward Declarations
void canSnifferTask(void *pvParameters);
void rgbStatusTask(void *pvParameters);
void initCAN();
void handleRoot();
void handleData();
void handleUpdatePage();
void handleDoUpdate();
void handleUpload();

void setup() {
    Serial.begin(115200);

    // 1. Initialize FastLED matching your working test setup
    FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
    FastLED.setBrightness(40);
    leds[0] = CRGB::Black;
    FastLED.show();

    // 2. Initialize Access Point
    WiFi.softAP(ssid, password);

    // 3. Setup CAN Drivers
    initCAN();

    // 4. CAN Sniffer Task - High Priority on Core 0 (Protects frame capture)
    xTaskCreatePinnedToCore(
        canSnifferTask, "CAN_Sniffer", 4096, NULL, 3, &CanTaskHandle, 0
    );

    // 5. RGB Status Task - Medium Priority on Core 1 (Handles FastLED loops)
    xTaskCreatePinnedToCore(
        rgbStatusTask, "RGB_Status", 2048, NULL, 1, &RgbTaskHandle, 1
    );

    // 6. Web Server Endpoints
    server.on("/", HTTP_GET, handleRoot);
    server.on("/data", HTTP_GET, handleData);
    server.on("/update", HTTP_GET, handleUpdatePage);
    server.on("/update", HTTP_POST, handleDoUpdate, handleUpload);

    server.begin();
}

void loop() {
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(2)); 
}

void initCAN() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_LISTEN_ONLY);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS(); 
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        twai_start();
    }
}

// Background Task for CAN Handling (Core 0)
void canSnifferTask(void *pvParameters) {
    twai_message_t message;
    for (;;) {
        if (twai_receive(&message, pdMS_TO_TICKS(5)) == ESP_OK) {
            canFrameCount++; // Signal the LED task that traffic dropped in
            
            lastRawId = message.identifier;
            sprintf(lastRawData, "%02X %02X %02X %02X %02X %02X %02X %02X", 
                    message.data[0], message.data[1], message.data[2], message.data[3], 
                    message.data[4], message.data[5], message.data[6], message.data[7]);

            // Add reverse engineered ID targets here 
            if (message.identifier == 0x201) { ignitionOn = (message.data[0] & 0x01); }
            if (message.identifier == 0x1F1) { inPark     = (message.data[0] == 0x18); }
            if (message.identifier == 0x216) { doorOpen   = (message.data[0] & 0x40); }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// Background Task for LED Animations (Core 1) using verified FastLED Methods
void rgbStatusTask(void *pvParameters) {
    bool heartbeatToggle = false;
    for (;;) {
        // Step A: Active CAN Bus traffic burst processing
        if (canFrameCount != lastCanFrameCount) {
            lastCanFrameCount = canFrameCount;
            
            if (ignitionOn || doorOpen) {
                leds[0] = CRGB::Green;
            } else {
                leds[0] = CRGB::Blue;
            }
            FastLED.show();
            vTaskDelay(pdMS_TO_TICKS(30)); // Quick frame-pulse flash length
        } 
        
        // Step B: Heartbeat execution (Red flashing)
        heartbeatToggle = !heartbeatToggle;
        if (heartbeatToggle) {
            leds[0] = CRGB::Red;
        } else {
            leds[0] = CRGB::Black;
        }
        FastLED.show();

        vTaskDelay(pdMS_TO_TICKS(250)); 
    }
}

// Web Server Implementation
void handleRoot() {
    String html = "<!DOCTYPE html><html><head>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
    html += "<title>Truck Sniffer</title>";
    html += "<style>";
    html += "body{background:#121212;color:#e0e0e0;font-family:sans-serif;text-align:center;padding:10px;margin:0;}";
    html += "nav{padding:10px;text-align:right;} a{color:#2196f3;text-decoration:none;}";
    html += ".card{background:#1e1e1e;padding:15px;margin:12px auto;max-width:400px;border-radius:8px;box-shadow:0 4px 6px rgba(0,0,0,0.3);}";
    html += ".status{font-size:22px;font-weight:bold;margin:8px 0;}";
    html += ".on{color:#00e676;} .off{color:#ff1744;}";
    html += ".raw{font-family:monospace;background:#2d2d2d;padding:10px;border-radius:4px;font-size:13px;word-break:break-all;}";
    html += "</style>";
    html += "<script>setInterval(()=>{fetch('/data').then(r=>r.json()).then(d=>{";
    html += "document.getElementById('ign').className=d.ign?'status on':'status off';";
    html += "document.getElementById('ign').innerText=d.ign?'ON':'OFF';";
    html += "document.getElementById('park').className=d.park?'status on':'status off';";
    html += "document.getElementById('park').innerText=d.park?'PARK':'DRIVE/OTHER';";
    html += "document.getElementById('door').className=d.door?'status on':'status off';";
    html += "document.getElementById('door').innerText=d.door?'OPEN':'CLOSED';";
    html += "document.getElementById('raw_id').innerText='0x'+d.id.toString(16).toUpperCase();";
    html += "document.getElementById('raw_data').innerText=d.data;";
    html += "});},400);</script>";
    html += "</head><body>";
    html += "<nav><a href='/update'>⚙ Update Firmware</a></nav>";
    html += "<h2>Truck Dashboard</h2>";
    html += "<div class='card'><div>Ignition</div><div id='ign' class='status off'>OFF</div></div>";
    html += "<div class='card'><div>Gear State</div><div id='park' class='status off'>DRIVE/OTHER</div></div>";
    html += "<div class='card'><div>Door Status</div><div id='door' class='status off'>CLOSED</div></div>";
    html += "<div class='card'><div>Latest Frame</div><p>ID: <span id='raw_id' style='color:#2196f3;'>0x00</span></p><div id='raw_data' class='raw'>Waiting...</div></div>";
    html += "</body></html>";
    server.send(200, "text/html", html);
}

void handleData() {
    String json = "{";
    json += "\"ign\":" + String(ignitionOn ? "true" : "false") + ",";
    json += "\"park\":" + String(inPark ? "true" : "false") + ",";
    json += "\"door\":" + String(doorOpen ? "true" : "false") + ",";
    json += "\"id\":" + String(lastRawId) + ",";
    json += "\"data\":\"" + String(lastRawData) + "\"";
    json += "}";
    server.send(200, "application/json", json);
}

void handleUpdatePage() {
    String html = "<!DOCTYPE html><html><head>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
    html += "<style>body{background:#121212;color:#e0e0e0;font-family:sans-serif;text-align:center;padding-top:50px;}";
    html += "form{background:#1e1e1e;padding:30px;border-radius:8px;display:inline-block;max-width:90%;}";
    html += "input[type=file]{margin:20px 0;display:block;}";
    html += "input[type=submit]{background:#2196f3;color:#fff;border:0;padding:10px 20px;border-radius:4px;cursor:pointer;}";
    html += "a{color:#aaa;display:block;margin-top:20px;text-decoration:none;}</style></head><body>";
    html += "<h2>Firmware Upload (.bin)</h2>";
    html += "<form method='POST' action='/update' enctype='multipart/form-data'>";
    html += "<input type='file' name='update' accept='.bin'>";
    html += "<input type='submit' value='Flash Firmware'>";
    html += "</form><a href='/'>◁ Back to Dashboard</a></body></html>";
    server.send(200, "text/html", html);
}

void handleDoUpdate() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", Update.hasError() ? "Flash Failed! <a href='/update'>Try again</a>" : "Flash Success! Rebooting device...");
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
            // Success
        } else {
            Update.printError(Serial);
        }
    }
}

#include <WiFi.h>
#include <WebServer.h>
#include "driver/twai.h"
#include <FastLED.h>

// Wi-Fi Access Point Configuration
const char *ssid = "TruckOBDscan";
const char *password = "12345678";

// Server Instantiations
WebServer server(80);
WiFiServer savvyServer(23); // SavvyCAN GVRET Default TCP Port
WiFiClient savvyClient;

// Hardware Pins (ESP32-S3 SuperMini)
#define CAN_RX_PIN GPIO_NUM_4
#define CAN_TX_PIN GPIO_NUM_5
#define DATA_PIN 48
#define NUM_LEDS 1
CRGB leds[NUM_LEDS];

// GVRET Protocol Constants
#define GVRET_MAGIC 0x11B0
#define GVRET_CMD_BUILD_STREAM 0x00

void initTWAI() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_LISTEN_ONLY); 
    // Note: Use TWAI_MODE_LISTEN_ONLY for safe decoding so you don't accidentally disrupt truck networks.
    
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS(); // Ford/Dodge/Chevy primary networks usually run at 500kbps
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        twai_start();
    }
}

void setup() {
    Serial.begin(115200);
    
    // Status LED Setup
    FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
    leds[0] = CRGB::Red; // Red = Booting/No Client
    FastLED.show();

    // Start Wi-Fi Access Point
    WiFi.softAP(ssid, password);
    savvyServer.begin();
    
    // Quick Web Dashboard Stub
    server.on("/", []() {
        server.send(200, "text/html", "<h1>ESP32-S3 CAN Bridge Active</h1>");
    });
    server.begin();

    initTWAI();
    leds[0] = CRGB::Blue; // Blue = Waiting for SavvyCAN
    FastLED.show();
}

void loop() {
    server.handleClient();

    // Check for incoming SavvyCAN connection
    if (!savvyClient || !savvyClient.connected()) {
        savvyClient = savvyServer.available();
        if (savvyClient) {
            leds[0] = CRGB::Green; // Green = SavvyCAN Connected
            FastLED.show();
        }
    }

    // Process Incoming Vehicle Messages
    twai_message_t message;
    if (twai_receive(&message, 0) == ESP_OK) {
        if (savvyClient && savvyClient.connected()) {
            // Build GVRET binary frame format expected by SavvyCAN
            uint8_t buf[20];
            uint32_t timeStamp = millis();

            buf[0] = 0xF1; // Frame Start Indicator
            buf[1] = 0x00; // Command: CAN Frame Data
            
            // Timestamp (4 bytes)
            memcpy(&buf[2], &timeStamp, 4);
            
            // CAN ID (4 bytes) - set MSB if Extended ID
            uint32_t id = message.identifier;
            if (message.extd) id |= 0x80000000;
            memcpy(&buf[6], &id, 4);

            // Length and Bus Information
            buf[10] = message.data_length_code & 0x0F; // Length
            buf[11] = 0x00;                            // Bus Number (Bus 0)

            // Copy Data Payload
            memcpy(&buf[12], message.data, message.data_length_code);

            // Write raw binary packet to network stream
            savvyClient.write(buf, 12 + message.data_length_code);
        }
    }
}

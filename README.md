<div align="center">

# 🚚 ESP32-S3 Wireless CAN-to-SavvyCAN Bridge

A high-performance, wireless CAN bus sniffing and streaming tool utilizing the built-in TWAI (Two-Wire Automotive Interface) controller of the ESP32-S3 to stream live vehicle data directly to SavvyCAN via Wi-Fi.

</div>

---

## 📊 Overview

This firmware transforms an ESP32-S3 (such as the SuperMini layout) into a wireless network adapter for automotive reverse engineering. It initializes a localized Wi-Fi Access Point and sets up a raw TCP streaming socket. Vehicle network traffic captured by the transceiver is immediately packaged into the standard **GVRET binary protocol** and streamed wirelessly over the network on Port 23.

This allows real-time data capture, logging, and reverse-engineering of modern vehicle networks (Ford, GM, Stellantis/Dodge, etc.) in SavvyCAN without needing a physical USB tether to the vehicle's diagnostic port.

---

## ⚙️ Core Functionality

* **Passive Listen-Only Stream:** The CAN hardware initializes explicitly in `TWAI_MODE_LISTEN_ONLY`. The microcontroller remains completely silent and invisible on the vehicle network, eliminating the risk of injecting error frames or disrupting critical truck communications.
* **GVRET Binary Packaging:** Dynamically encapsulates standard and extended CAN IDs, lengths, payloads, and high-precision timestamps into the native binary frame format expected by SavvyCAN.
* **On-Board Diagnostics LEDs:** Integrates automated visual feedback using an on-board addressable RGB LED to communicate the system's operational status instantly.
* **Dual-Service Server Loop:** Runs a standard Web Server on Port 80 for secondary dashboards alongside the high-priority raw TCP data stream on Port 23.

---

## 🛠️ Hardware Requirements

* **Microcontroller:** ESP32-S3 (e.g., SuperMini or development board variant)
* **CAN Transceiver:** 3.3V compatible CAN Transceiver (e.g., SN65HVD230, TJA1050 with logic shifters)
* **Status LED:** WS2812B Addressable RGB LED (built into pin `GPIO 48` on SuperMini boards)

### Pin Map Configuration
* **CAN RX:** `GPIO 4`
* **CAN TX:** `GPIO 5`
* **WS2812B Data:** `GPIO 48`

---

## 🟢 Status LED Meanings

* 🔴 **Red:** System booting; Wi-Fi network initializing.
* 🔵 **Blue:** Wi-Fi active (`TruckOBDscan`), awaiting connection from SavvyCAN client.
* 🟢 **Green:** Active TCP handshake established; streaming vehicle frames live.

---

## 🚀 Quick Start Guide

### 1. Flash the Firmware
Upload the provided sketch to your ESP32-S3 using the Arduino IDE or PlatformIO. Ensure you have the `FastLED` library installed for status updates.

### 2. Connect to the Adapter
1. Supply power to the microcontroller via USB or a regulated vehicle power source.
2. Open your PC or tablet's Wi-Fi menu and connect to the access point:
   * **SSID:** `TruckOBDscan`
   * **Password:** `12345678`

### 3. Configure SavvyCAN
1. Launch **SavvyCAN** on your computer.
2. Navigate to **Connection** ➡️ **Open Connection Window**.
3. Click **Add New Connection** and select **Network (GVRET)**.
4. Input the following target networking credentials:
   * **IP Address:** `192.168.4.1`
   * **Port:** `23`
5. Click **Create**. Check the **Active** checkbox to initiate the live data stream.

---

## 🔍 Reverse Engineering Best Practices

When sorting through high-volume truck communications (e.g., capturing a brake pedal toggle or door lock switch):
1. Open **RE Tools** ➡️ **Sniffer** in SavvyCAN.
2. Lower the interface **Timeout** to `300 ms` to drop static lines rapidly.
3. Leave the vehicle controls idle for 3 seconds and click **Notch** to filter out persistent background heartbeats.
4. Operate the vehicle control to observe the newly populated target IDs instantly.

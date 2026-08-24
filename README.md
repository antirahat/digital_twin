# Embedded Digital Twin for Real-Time Industrial Machine Monitoring (ESP32)

[![Platform: ESP32](https://img.shields.io/badge/Platform-ESP32%20DevKit%20V1-blue.svg?logo=espressif)](https://www.espressif.com/en/products/socs/esp32)
[![Framework: Arduino / C++](https://img.shields.io/badge/Framework-Arduino%20%2F%20C%2B%2B-00979D.svg?logo=arduino)](https://www.arduino.cc/)
[![Display: 1.3" SH1106 OLED](https://img.shields.io/badge/HMI-1.3%22%20I2C%20OLED%20(SH1106)-darkgreen.svg)](#local-oled-interface)
[![Sensors: MPU6050 & DHT22](https://img.shields.io/badge/Sensors-MPU6050%20%7C%20DHT22%20%7C%20ACS712-orange.svg)](#hardware-bill-of-materials-bom)
[![Paper: IEEE Format](https://img.shields.io/badge/Paper-IEEE%20Format%20(LaTeX)-red.svg)](#academic-paper--citation)
[![Course: EEE4103](https://img.shields.io/badge/Course-EEE4103%20Capstone%20Project-purple.svg)](#project-metadata--authors)

A low-cost, self-contained **Embedded Digital Twin** edge node powered by the dual-core 32-bit Tensilica Xtensa ESP32 microcontroller. The system synchronizes the physical operating state of rotating industrial machinery (induction motors, pumps, compressors) with an active digital replica, executing multi-parameter edge sensing, real-time threshold analytics, closed-loop relay cutoff safety, on-device OLED graphics, and a zero-dependency embedded web dashboard with real-time oscilloscope waveforms.

---

## 📑 Table of Contents
1. [Project Overview & Objectives](#-project-overview--objectives)
2. [Key System Features](#-key-system-features)
3. [System Architecture](#-system-architecture)
4. [Hardware Bill of Materials (BOM)](#-hardware-bill-of-materials-bom)
5. [Complete Pinout & Wiring Map](#-complete-pinout--wiring-map)
6. [Interactive Visualizer & Schematics](#-interactive-visualizer--schematics)
7. [Firmware & Edge Logic](#-firmware--edge-logic)
8. [Embedded Web Dashboard & Telemetry API](#-embedded-web-dashboard--telemetry-api)
9. [Quick Start & Flashing Guide](#-quick-start--flashing-guide)
10. [Repository Structure](#-repository-structure)
11. [Academic Paper & Citation](#-academic-paper--citation)
12. [Troubleshooting & FAQ](#-troubleshooting--faq)
13. [Project Metadata & Authors](#-project-metadata--authors)

---

## 🌟 Project Overview & Objectives

Unplanned downtime and mechanical breakdowns in industrial plants lead to severe financial losses, equipment damage, and workplace hazards. While high-end Supervisory Control and Data Acquisition (SCADA) and PLC solutions exist, they remain cost-prohibitive and complex for Small and Medium Enterprises (SMEs).

This project implements an **autonomous, low-cost Embedded Digital Twin edge node** that delivers:
* **Heterogeneous Edge Sensing:** Captures high-frequency multi-axis mechanical vibration (MPU6050 IMU), thermal and relative humidity gradients (DHT22), and electrical load currents.
* **Deterministic Safety Cutoff:** Evaluates dynamic motion scores and environmental thresholds at edge level (100ms loop), instantly triggering synchronized alarms and an optocoupled relay safety trip.
* **Dual Human-Machine Interfaces:**
  1. *Local On-Device HMI:* 1.3" 128x64 I2C OLED with boot progress animations, telemetry status badges, and dynamic randomized-jitter shake alert screens.
  2. *Remote Web HMI:* Embedded HTTP server serving a lightweight, zero-external-CDN web dashboard featuring dual HTML5 Canvas oscilloscopes (Accelerometer & Gyroscope waveforms).
* **Non-Volatile Configuration:** Custom threshold values configured via the web UI are stored into ESP32 Flash memory (`Preferences.h` / NVS) to persist across power cycles.
* **Dual-Mode Networking:** Works seamlessly in local Wi-Fi router (Station) mode or standalone SoftAP (`ESP32_DigitalTwin` / `192.168.4.1`) with Captive Portal and mDNS (`http://digitaltwin.local`).

---

## ⚡ Key System Features

| Feature | Implementation Details |
| :--- | :--- |
| **Microcontroller** | Dual-core Tensilica Xtensa 32-bit LX6 ESP32 @ 240 MHz, 520 KB SRAM, 4 MB Flash |
| **Vibration & Dynamics** | MPU6050 (6-DOF Accelerometer + Gyroscope) sampled at 100ms (10 Hz nominal, scalable to 200 Hz) |
| **Dynamic Motion Metric** | $\text{MotionScore} = \|a_y\| + 2.0 \cdot \|g_y\|$ for robust impact and vibration anomaly detection |
| **Thermal & Humidity** | DHT22 / AM2302 high-precision sensor sampled every 1.5s (Range: -40°C to 80°C, 0–100% RH) |
| **Safety Actuation** | 5V Optocoupled Relay switch (COM / NO) driving high-power cutoff / Alert LED + Audible active buzzer |
| **Local Display HMI** | 1.3" SH1106 128x64 Monochrome I2C OLED (Address `0x3C`) with animated boot bar & physical jitter effects |
| **Web Oscilloscope** | Pure HTML5 Canvas 60 FPS rolling waveforms with dynamic threshold overlay lines (zero internet/CDN required) |
| **Threshold Storage** | Non-Volatile Storage (NVS) via ESP32 `Preferences.h` |
| **Network Redundancy** | Wi-Fi Station mode with automatic fallback to SoftAP Mode + DNS Captive Portal + mDNS |
| **Mathematical Modeling** | Linear Discrete Kalman Filter sensor fusion for noise filtering & machine degradation forecasting |

---

## 🏗 System Architecture

```
                                  PHYSICAL ASSET (Machine / Motor)
                       ┌────────────────────────────────────────────────────┐
                       │   [Vibrations]        [Temperature/RH]   [Current]  │
                       └────────┬─────────────────────┬──────────────┬──────┘
                                │                     │              │
                                ▼                     ▼              ▼
                       ┌─────────────────┐   ┌─────────────────┐    │
                       │ MPU6050 (6-DOF) │   │  DHT22 / AM2302 │    │
                       │ I2C (0x68)      │   │  Digital 1-Wire │    │
                       └────────┬────────┘   └────────┬────────┘    │
                                │ (SDA:21/SCL:22)     │ (GPIO 4)     │
                                └──────────┬──────────┘              │
                                           ▼                         ▼
  ╔═══════════════════════════════════════════════════════════════════════════════════════╗
  ║                               ESP32 DEVKIT V1 EDGE NODE                               ║
  ║  ┌─────────────────────────────────────────────────────────────────────────────────┐  ║
  ║  │ Core 0: Wireless Networking & Web Server                                        │  ║
  ║  │  • Station Mode / SoftAP (192.168.4.1) • DNS Captive Portal • mDNS Daemon       │  ║
  ║  │  • REST Endpoints: GET /, GET /api/data, POST /api/set                          │  ║
  ║  ├─────────────────────────────────────────────────────────────────────────────────┤  ║
  ║  │ Core 1: Deterministic Real-Time Control & Edge DSP                              │  ║
  ║  │  • 100ms Sensor Acquisition Loop • Dynamic Motion Metric Computation            │  ║
  ║  │  • Discrete Kalman Filter Fusion • NVS Threshold Persistence (Preferences.h)     │  ║
  ║  │  • Closed-Loop State Machine Evaluation (OK vs CRITICAL ALERT)                  │  ║
  ║  └─────────────────────────────────────────────────────────────────────────────────┘  ║
  ╚═══════════════════════════════════════════════════════════════════════════════════════╝
               │ (I2C Bus: 21/22)               │ (GPIO 18 / 26)                │ (Wi-Fi 802.11 b/g/n)
               ▼                                ▼                               ▼
   ┌───────────────────────┐        ┌───────────────────────┐       ┌───────────────────────┐
   │    1.3" I2C OLED      │        │ CLOSED-LOOP ACTUATION │       │ EMBEDDED WEB DASHBOARD│
   │  • Animated Boot Bar  │        │  • Optocoupled Relay  │       │  • HTML5 Oscilloscope │
   │  • Telemetry Status   │        │  • Synchronized Buzzer│       │  • Real-Time Polling  │
   │  • Jitter Shake Alert │        │  • Flashing Alert LED │       │  • Flash Config Form  │
   └───────────────────────┘        └───────────────────────┘       └───────────────────────┘
```

---

## 📦 Hardware Bill of Materials (BOM)

| Component | Part / Model | Operating Voltage | Interface Type | Purpose |
| :--- | :--- | :--- | :--- | :--- |
| **Microcontroller** | ESP32 DevKit V1 (30-pin) | 5.0V (VIN) / 3.3V (Logic) | USB / Wi-Fi / I2C / GPIO | Main dual-core edge processor |
| **IMU / Vibration** | MPU6050 (GY-521 breakout) | 3.3V | I2C (`0x68`) | 3-axis acceleration & 3-axis angular velocity |
| **Thermal / Humidity** | DHT22 / AM2302 | 3.3V – 5.0V | 1-Wire Digital | Ambient & machine thermal monitoring |
| **Display (HMI)** | 1.3-inch Monochrome OLED | 3.3V | I2C (`0x3C`, SH1106) | On-site operator display & alert screens |
| **Relay Module** | 5V Single-Channel Relay | 5.0V (Coil) / 3.3V (Logic) | Digital Output | Industrial machine cutoff & alert LED driving |
| **Acoustic Alarm** | 5V Active Buzzer (2-wire) | 3.3V – 5.0V | Digital Output | Synchronized audible alert alarm |
| **Visual Indicator** | 5mm High-Brightness LED | 2.0V – 3.2V (via 330Ω) | Switched 5V Rail | Relay-switched visual alarm indicator |
| **Resistors** | 330Ω (1/4W) & 10kΩ (Optional)| — | Passive | LED current limiting & DHT pull-up |
| **Prototyping** | Breadboard & Jumper Wires | — | Dupont M-M / M-F | Circuit assembly |

---

## 🔌 Complete Pinout & Wiring Map

### Master Connection Table

| Peripheral | Pin Label | ESP32 DevKit V1 Pin | Power Rail | Interface / Signal | Notes |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **ESP32** | `VIN` | — | External 5.0V (USB) | Power Input | Feeds onboard 3.3V LDO regulator |
| | `GND` | — | Common Ground (0V) | Ground Reference | Common ground across all modules |
| **DHT22** | `VCC` (Pin 1) | **`3V3`** (or 5V) | 3.3V Rail | Power | Power input for humidity sensor |
| | `DATA` (Pin 2)| **`GPIO 4`** | — | Digital 1-Wire | 4.7kΩ–10kΩ pull-up to 3.3V if standalone |
| | `NC` (Pin 3)  | *Leave Unconnected* | — | Not Connected | No internal connection |
| | `GND` (Pin 4) | **`GND`** | Common Ground | Ground | Ground return |
| **MPU6050** | `VCC` | **`3V3`** | 3.3V Rail | Power | Power input (onboard LDO supports 3.3V) |
| | `GND` | **`GND`** | Common Ground | Ground | Ground return |
| | `SCL` | **`GPIO 22`** | — | I2C Clock | Shared hardware I2C bus |
| | `SDA` | **`GPIO 21`** | — | I2C Data | Shared hardware I2C bus |
| | `AD0` / `INT`| *Unconnected* | — | Default Address | Pulls low internally for address `0x68` |
| **1.3" OLED** | `VCC` | **`3V3`** | 3.3V Rail | Power | SH1106 driver logic supply |
| | `GND` | **`GND`** | Common Ground | Ground | Ground return |
| | `SCL` | **`GPIO 22`** | — | I2C Clock | Shared with MPU6050 |
| | `SDA` | **`GPIO 21`** | — | I2C Data | Shared with MPU6050 |
| **Buzzer** | `(+)` (Long Leg) | **`GPIO 18`** | — | Digital Out | Active High trigger |
| | `(-)` (Short Leg)| **`GND`** | Common Ground | Ground | Ground return |
| **5V Relay** | `VCC` / `DC+` | **`VIN` (5V)** | 5V Rail | Coil Power | Direct 5V power from USB / VIN |
| | `GND` / `DC-` | **`GND`** | Common Ground | Ground | Optocoupler ground |
| | `IN` / Signal | **`GPIO 26`** | — | Digital Out | Relay trigger signal |
| **Alert LED** | Anode `(+)` | Relay **`NO`** | Switched 5V Rail | Switched Power | Connected via Relay `COM` (to 5V) |
| | Cathode `(-)`| 330Ω Resistor | — | Current Limit | Connects through 330Ω resistor to GND |
| **330Ω Resistor**| Leg 1 / Leg 2 | LED `(-)` $\rightarrow$ **`GND`** | Common Ground | Current Limiting | Completes path to common ground |

> [!IMPORTANT]
> Both the **MPU6050** and the **1.3" OLED Display** share the same hardware I2C bus on **`GPIO 21 (SDA)`** and **`GPIO 22 (SCL)`**. The ESP32 I2C hardware controller natively arbitrates communication via unique device addresses (`0x68` for MPU6050, `0x3C` for SH1106).

---

## 💻 Interactive Visualizer & Schematics

This repository provides multiple circuit visualization assets:

* **Interactive Circuit Visualizer ([`circuit_visualizer.html`](file:///C:/Users/rahat/OneDrive/Desktop/InterestingStuffs/thesis_and_research_gig/digital_twin/circuit_visualizer.html)):** A self-contained, browser-based hardware schematic visualizer. Open this file in any web browser to view color-coded wiring paths, isolate individual component circuits, inspect pinouts, and test interactive connection states.
* **Clean Vector Schematic ([`circuit_diagram_clean.drawio`](file:///C:/Users/rahat/OneDrive/Desktop/InterestingStuffs/thesis_and_research_gig/digital_twin/circuit_diagram_clean.drawio)):** Professional Draw.io / Diagrams.net vector circuit layout.
* **Direct Pin-to-Pin Reference ([`pin_to_pin_guide.md`](file:///C:/Users/rahat/OneDrive/Desktop/InterestingStuffs/thesis_and_research_gig/digital_twin/pin_to_pin_guide.md)):** Detailed table with color coding, breadboard layout instructions, and direct verification steps.

---

## ⚙️ Firmware & Edge Logic

The firmware is located in [`esp32_dht11/esp32_dht11.ino`](file:///C:/Users/rahat/OneDrive/Desktop/InterestingStuffs/thesis_and_research_gig/digital_twin/esp32_dht11/esp32_dht11.ino).

```
esp32_dht11.ino Execution Flow
├── 1. setup()
│   ├── Initialize GPIOs (Buzzer: 18, Relay: 26)
│   ├── Initialize Wire (I2C: 21, 22), DHT22 & MPU6050
│   ├── Initialize SH1106 OLED & execute playBootAnimation()
│   ├── Load saved thresholds from Flash via Preferences.h ("twin_config")
│   ├── Connect to Wi-Fi Station (or launch SoftAP 'ESP32_DigitalTwin' + DNS server)
│   ├── Start WebServer on Port 80 & register REST endpoints
│   └── Trigger startup confirmation audio/visual chirp
│
└── 2. loop()
    ├── DNS & HTTP Client Handling (dnsServer.processNextRequest(), server.handleClient())
    ├── 100ms High-Frequency Motion Task:
    │   ├── Read MPU6050 (Y-accel & Y-gyro)
    │   └── Compute MotionScore = |a_y| + 2.0 * |g_y|
    ├── 1500ms Environmental Sampling Task:
    │   └── Read DHT22 Temperature & Humidity
    ├── Threshold State Machine:
    │   └── Evaluate isAlertActive = (tempAlert || humAlert || motionAlert)
    ├── Local HMI Refresh:
    │   └── updateOLED() -> Normal HUD vs Animated Shaking Coordinate Jitter Alert
    └── Actuation Loop:
        └── If isAlertActive -> triggerAlertPing() every 2.0s (Buzzer chirp + Relay LED blink)
```

### Local OLED Display States
1. **Bootup Sequence:** Smooth animated rounded progress bar (0% to 100%) with course welcome text and completion chirp.
2. **Normal Operating HUD:** Displays telemetry (`T: 28.5C`, `H: 74%`, `Y-Acc: 0.2`, `RLY: OK`), dynamic `[ OK ]` badge, and permanent active IP address.
3. **Motion Alert HUD:** Switches to a high-visibility warning screen (*"!! MOTION ALERT !! System is experiencing unexpected motion!"*) with real-time **randomized coordinate jitter ($\pm 2\text{px}$)** to physically mirror the machine vibration.

---

## 🌐 Embedded Web Dashboard & Telemetry API

The ESP32 hosts a fully responsive, dark-mode web application directly from flash memory (`PROGMEM`).

```
                    ┌──────────────────────────────────────────────┐
                    │            ESP32 DIGITAL TWIN NODE           │
                    │   Real-Time Condition Monitoring & Telemetry │
                    └──────────────────────────────────────────────┘
                    ┌──────────────────────────────────────────────┐
                    │               [ SYSTEM NORMAL ]              │
                    │  ┌──────────────────┐  ┌──────────────────┐  │
                    │  │   TEMPERATURE    │  │     HUMIDITY     │  │
                    │  │     28.4 °C      │  │      65.0 %      │  │
                    │  │  (Limit: 35.0°C) │  │  (Limit: 85.0%)  │  │
                    │  └──────────────────┘  └──────────────────┘  │
                    │  ┌──────────────────┐  ┌──────────────────┐  │
                    │  │  Y-AXIS MOTION   │  │ RELAY & ALERT LED│  │
                    │  │    0.35 m/s²     │  │      NORMAL      │  │
                    │  │  (Limit: 4.0)    │  │  (GPIO 26 OK)    │  │
                    │  └──────────────────┘  └──────────────────┘  │
                    └──────────────────────────────────────────────┘
                    ┌──────────────────────────────────────────────┐
                    │     Real-Time MPU6050 Waveform Analysis      │
                    │                                              │
                    │  Y-Axis Acceleration (m/s²)  [Threshold: 4.0]│
                    │  ┌────────────────────────────────────────┐  │
                    │  │~~~\_/\_/\_/\_/\_/\_/\_/\_/\_/\_/\_/\_  │  │
                    │  └────────────────────────────────────────┘  │
                    │  Y-Axis Gyroscope (rad/s)                    │
                    │  ┌────────────────────────────────────────┐  │
                    │  │─────────────────────────────────────── │  │
                    │  └────────────────────────────────────────┘  │
                    └──────────────────────────────────────────────┘
                    ┌──────────────────────────────────────────────┐
                    │          Configure Alert Thresholds          │
                    │  Max Temperature (°C): [ 35.0 ]              │
                    │  Max Humidity (%):     [ 85.0 ]              │
                    │  Motion Limit (m/s²):  [ 4.0  ]              │
                    │  [ SAVE THRESHOLD SETTINGS (NVS FLASH) ]     │
                    └──────────────────────────────────────────────┘
```

### REST API Reference

#### 1. Fetch Live Telemetry
* **Endpoint:** `GET /api/data`
* **Response:** `application/json`
```json
{
  "temperature": 28.4,
  "humidity": 65.0,
  "y_accel": 0.35,
  "y_gyro": 0.02,
  "y_motion": 0.39,
  "temp_th": 35.0,
  "hum_th": 85.0,
  "motion_th": 4.0,
  "temp_alert": false,
  "hum_alert": false,
  "motion_alert": false,
  "relay_state": false,
  "alert": false
}
```

#### 2. Update Threshold Configuration
* **Endpoint:** `POST /api/set` or `GET /api/set?temp=38.0&hum=80.0&motion=5.5`
* **Parameters:**
  * `temp` *(float)*: New maximum temperature limit (°C)
  * `hum` *(float)*: New maximum relative humidity limit (%)
  * `motion` *(float)*: New dynamic motion score limit ($m/s^2$)
* **Action:** Automatically updates active thresholds and commits values to ESP32 Flash memory via `Preferences.h`.
* **Response:** `200 OK` (`text/plain: OK`)

---

## 🚀 Quick Start & Flashing Guide

### 1. Prerequisites & Software Installation
1. Install the latest [Arduino IDE](https://www.arduino.cc/en/software) (v2.x recommended).
2. Install the **ESP32 Board Package**:
   * Open `File` $\rightarrow$ `Preferences`.
   * Add to *Additional Boards Manager URLs*:
     ```text
     https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
     ```
   * Go to `Tools` $\rightarrow$ `Board` $\rightarrow$ `Boards Manager...`, search for `esp32`, and install **esp32 by Espressif Systems**.
3. Install required libraries via `Tools` $\rightarrow$ `Manage Libraries...`:
   * `Adafruit SH110X` (by Adafruit)
   * `Adafruit GFX Library` (by Adafruit)
   * `Adafruit MPU6050` (by Adafruit)
   * `Adafruit Unified Sensor` (by Adafruit)
   * `DHT sensor library` (by Adafruit)

### 2. Hardware Wiring Checklist
* Connect the circuit according to the [Complete Pinout & Wiring Map](#-complete-pinout--wiring-map).
* Ensure standard 3.3V is supplied to MPU6050, DHT22, and OLED.
* Ensure **5V VIN** is supplied to the Relay module `VCC` and Relay `COM` terminal.
* Double-check that common Ground is linked across all modules.

### 3. Configure Wi-Fi & Flash Firmware
1. Open [`esp32_dht11/esp32_dht11.ino`](file:///C:/Users/rahat/OneDrive/Desktop/InterestingStuffs/thesis_and_research_gig/digital_twin/esp32_dht11/esp32_dht11.ino) in Arduino IDE.
2. Configure your Wi-Fi credentials on lines 16–17:
   ```cpp
   const char* ssid     = "YOUR_WIFI_SSID";     // Leave blank "" to use standalone SoftAP mode
   const char* password = "YOUR_WIFI_PASSWORD";
   ```
3. Set the board parameters under `Tools`:
   * **Board:** `DOIT ESP32 DEVKIT V1` (or `ESP32 Dev Module`)
   * **Upload Speed:** `921600` (or `115200`)
   * **Flash Frequency:** `80MHz`
   * **Partition Scheme:** `Default 4MB with spiffs (1.2MB APP/1.5MB SPIFFS)`
   * **Port:** Select your ESP32 COM port
4. Click **Upload** (Hold `BOOT` button on ESP32 if upload fails to start).

### 4. Accessing the Digital Twin Dashboard
* **Station Mode:** Look at the OLED footer or Serial Monitor (115200 baud) for the assigned IP address (e.g., `http://192.168.1.150` or `http://digitaltwin.local`).
* **SoftAP Mode:** Connect your phone/laptop to the Wi-Fi network:
  * **SSID:** `ESP32_DigitalTwin`
  * **Password:** `12345678`
  * Navigate to `http://192.168.4.1` (Captive portal will automatically redirect requests).

---

## 📁 Repository Structure

```
digital_twin/
├── esp32_dht11/
│   └── esp32_dht11.ino             # Main C++/Arduino firmware with embedded dashboard
├── circuit_visualizer.html         # Interactive web-based wiring schematic viewer
├── circuit_diagram_clean.drawio    # Vector Draw.io circuit diagram schematic
├── circuit_diagram.drawio          # Raw Draw.io diagram file
├── pin_to_pin_guide.md             # Complete step-by-step physical connection guide
├── context.md                      # Technical project specifications & architecture notes
├── report.tex                      # Full IEEE conference paper (LaTeX source with TikZ figures)
├── digital_twin.pdf                # Compiled IEEE research paper / project documentation
├── Project_Proposal_Digital_Twin_ESP32 (1).docx # Original capstone proposal document
├── extracted_proposal.txt          # Text extract of course proposal & literature survey
├── extracted_media/                # Figures, diagrams, and schematics
└── README.md                       # Comprehensive project documentation (this file)
```

---

## 📄 Academic Paper & Citation

This project is fully documented in an IEEE two-column conference paper located at [`report.tex`](file:///C:/Users/rahat/OneDrive/Desktop/InterestingStuffs/thesis_and_research_gig/digital_twin/report.tex), featuring mathematical formulations for **Kalman Filter sensor fusion**, edge execution latency benchmarks, and comparative literature analysis.

```bibtex
@inproceedings{morshed2026digitaltwin,
  title     = {Embedded Digital Twin Architecture for Real-Time Industrial Machine Health Monitoring, Safety Cutoff, and Longevity Optimization Using ESP32},
  author    = {Morshed, Md Abdullah All and Akter, Tamanna and Fuad, Faisal Ahmed and Rafi, Rafi Ul Islam},
  booktitle = {EEE4103 Microprocessor and Embedded Systems Capstone Proceedings},
  institution = {American International University-Bangladesh (AIUB)},
  year      = {2026}
}
```

---

## 🔧 Troubleshooting & FAQ

<details>
<summary><b>1. The OLED display is completely blank upon boot.</b></summary>

* Verify that `GPIO 21 (SDA)` and `GPIO 22 (SCL)` are securely connected.
* Ensure your OLED driver is the **SH1106** with address `0x3C`. If you are using an SSD1306 display, use the `Adafruit_SSD1306` library and verify whether the I2C address is `0x3C` or `0x3D`.
* Ensure `VCC` is supplied with stable 3.3V.
</details>

<details>
<summary><b>2. The Serial Monitor displays "[WARNING] MPU6050 not found".</b></summary>

* Check that the MPU6050 `VCC` is connected to 3.3V and `GND` is connected to common ground.
* Confirm that the `AD0` pin on the MPU6050 is floating or grounded (setting address `0x68`). If `AD0` is tied to 3.3V, its address shifts to `0x69`.
</details>

<details>
<summary><b>3. DHT22 returns "SENSOR READ ERROR!" or NaN.</b></summary>

* DHT22 sensors require at least 1.0 to 1.5 seconds between successive read commands.
* If using a raw 4-pin DHT22 package (without a breakout board), ensure a **4.7kΩ to 10kΩ pull-up resistor** is connected between the `DATA` pin (GPIO 4) and `3V3`.
</details>

<details>
<summary><b>4. Relay clicks continuously or resets the ESP32.</b></summary>

* A 5V relay module draws significant coil current (~70-80mA) upon energization. Always power the relay coil from the **5V VIN** pin (or an external 5V supply), never from the ESP32 `3V3` pin.
* Ensure common ground between the external power supply and the ESP32.
</details>

---

## 👥 Project Metadata & Authors

* **Course:** Microprocessor and Embedded System (`EEE4103`)
* **Capstone Term:** Summer 2025–26
* **Institution:** Faculty of Engineering, **American International University-Bangladesh (AIUB)**

### Project Team Members

| # | Name | Student ID | Department / Program |
| :-: | :--- | :---: | :--- |
| 1 | **Md Abdullah All Morshed** | `23-50774-1` | Electrical & Electronic Engineering (EEE) |
| 2 | **Tamanna Akter** | `23-53525-3` | Computer Science & Engineering (CSE) |
| 3 | **Faisal Ahmed Fuad** | `23-50946-1` | Computer Science & Engineering (CSE) |
| 4 | **Rafi Ul Islam Rafi** | `22-49568-3` | Electrical & Electronic Engineering (EEE) |

---
*Developed for AIUB EEE4103 Capstone Project. Open source and available for research and education.*

---
publishDate: 2026-05-24

title: MediTrack - Intelligent Medication Habit Monitor

excerpt: MediTrack is a smart IoT healthcare system built on the MYOSA platform that detects real medication-taking behavior using motion sensing, promotes adherence through intelligent feedback, and provides real-time monitoring through an OLED interface and live web dashboard.

image: meditrack-logo.png

tags:
  - healthcare
  - iot
  - esp32
  - medtech
  - myosa
---

> A smarter approach to medication adherence — combining real motion sensing, intelligent monitoring, and accessible healthcare technology.


---

## Acknowledgements

This project was developed as part of the IEEE MYOSA prototype initiative.

MediTrack was inspired by a deeply personal experience within our team — witnessing the daily challenges of managing lifelong medication schedules for a family member. Even with care and attention, missing or delaying medication can happen easily in busy routines, and over time these inconsistencies can significantly impact health and quality of life.

We would like to thank our mentors, peers, and the MYOSA community for their support, guidance, and encouragement throughout the development process.

---

## Overview

Medication non-adherence remains one of the most overlooked healthcare challenges worldwide. Millions of patients with chronic illnesses struggle to consistently follow prescribed medication schedules due to forgetfulness, routine disruption, or lack of monitoring systems.

MediTrack was designed to address this issue through a practical, low-cost, and intelligent embedded healthcare solution.

Unlike traditional reminder systems that only notify users at scheduled times, MediTrack actively monitors medication-taking behavior using real motion detection. The system combines sensor-based activity tracking, real-time feedback, adherence scoring, and live monitoring into one compact IoT platform.

Built using the MYOSA ESP32 platform, MediTrack demonstrates how accessible embedded systems can support preventive healthcare and encourage long-term medication consistency.

### What MediTrack Does

* Detects real medicine bottle movement using MPU6050 motion sensing
* Tracks whether medication is taken on time, delayed, or missed
* Provides real-time visual and audible feedback
* Displays adherence information on OLED and web dashboard
* Encourages healthier medication habits through monitoring and accountability

### Why This Matters

Medication adherence directly affects treatment effectiveness, recovery outcomes, and patient safety. Missed doses can result in:

* worsening medical conditions
* avoidable hospital visits
* reduced treatment effectiveness
* increased healthcare costs

MediTrack aims to bridge the gap between simple reminders and actual behavioral adherence monitoring.

### Intended Users

* Patients requiring long-term medication routines
* Elderly individuals living independently
* Caregivers and family members
* Smart healthcare and IoT education environments

---

## Demo

### Images

<p align="center">
  <img src="./assets/images/meditrack/meditrack-hardware.jpg" width="800"><br/>
  <i>MediTrack hardware prototype using MYOSA ESP32, MPU6050, OLED display, and buzzer modules</i>
</p>

<p align="center">
  <img src="./assets/images/meditrack/meditrack-dashboard.jpg" width="800"><br/>
  <i>Real-time local dashboard displaying adherence tracking and medication status</i>
</p>

### Videos

<video controls width="100%">
  <source src="./meditrack-demo.mp4" type="video/mp4">
</video>

[Download / View Demo Video](./meditrack-demo.mp4)

---

## Features (Detailed)

### 1. Intelligent Dose Scheduling

MediTrack operates using configurable medication intervals and monitoring windows.

For demonstration purposes:

* Dose interval: **60 seconds**
* Grace period: **15 seconds**

The system automatically transitions through medication states:

`Waiting for Dose → Dose Due → Dose Taken / Dose Delayed / Missed Dose`

This structure allows real-time simulation of medication adherence scenarios during demonstrations.

---

### 2. Real Motion-Based Intake Detection

Instead of relying on simple button presses or timers, MediTrack uses the MPU6050 accelerometer and gyroscope to detect actual bottle movement and tilt.

The system:

* tracks acceleration and orientation changes
* detects intentional bottle movement
* differentiates between valid intake actions and accidental disturbances
* records medication events accordingly

This creates a more realistic and reliable medication adherence monitoring approach.

---

### 3. OLED Feedback Interface

The SSD1306 OLED display provides immediate user feedback including:

* medication status
* adherence percentage
* next dose countdown
* system notifications

This allows users to quickly understand their medication schedule and intake history without requiring external devices.

---

### 4. Audible Alert System

The buzzer module provides simple audio-based medication alerts:

* **1 beep** → Dose Taken
* **2 beeps** → Dose Delayed
* **3 beeps** → Missed Dose

This improves accessibility and ensures users receive immediate feedback during medication events.

---

### 5. Live Local Web Dashboard

Using the ESP32’s built-in Wi-Fi capability, MediTrack hosts a local healthcare dashboard accessible directly from a browser.

The dashboard displays:

* current medication status
* adherence score
* missed dose count
* delayed intake events
* live medication timing information

This creates a caregiver-friendly monitoring interface without requiring cloud infrastructure.

---

### 6. Reliable Embedded Healthcare Prototype

The system was designed with simplicity, reliability, and accessibility in mind.

Features include:

* real-time embedded processing
* local-only operation
* low hardware cost
* modular sensor integration
* scalable IoT architecture for future healthcare applications

---

## Usage Instructions

### Hardware Connections

The MPU6050 sensor and SSD1306 OLED display are connected using a breadboard-based shared I2C setup.

On the MYOSA ESP32 board, the I2C communication pins are labeled directly as:

* `SDA`
* `SCL`

These correspond internally to:

* SDA → GPIO 21
* SCL → GPIO 22

Both the OLED display and MPU6050 sensor share the same SDA and SCL communication lines through the breadboard, along with shared 3.3V and GND rails.

Additional connections:

* Buzzer connected to GPIO 18

### Running the System

1. Open `meditrack.ino` in Arduino IDE
2. Select the ESP32 board and correct COM port
3. Install required libraries
4. Upload firmware to the MYOSA ESP32 board
5. Connect to the local MediTrack hotspot or Wi-Fi network
6. Open dashboard IP in browser
7. Wait until the medication becomes due
8. Tilt or move the medicine bottle to simulate intake

---

## Tech Stack

* **ESP32 (MYOSA Platform)**
* **Arduino / Embedded C++**
* **MPU6050 Accelerometer + Gyroscope**
* **SSD1306 OLED Display**
* **ESP32 WebServer**
* **I2C Communication Protocol**
* **Wi-Fi Local Dashboard**

---

## Requirements / Installation

Install the following Arduino libraries:

```bash
Adafruit GFX Library
Adafruit SSD1306
Adafruit MPU6050
Adafruit Unified Sensor
```

Install ESP32 board support in Arduino IDE:

```bash
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
```

Then:

```bash
1. Select ESP32 Dev Module
2. Select COM Port
3. Verify Code
4. Upload Firmware
```

---

## Future Improvements

Potential future extensions include:

* multi-user medication profiles
* cloud synchronization
* caregiver notifications
* mobile application support
* medication analytics
* AI-assisted adherence prediction

---

## Conclusion

MediTrack demonstrates how embedded IoT systems can be used to create meaningful healthcare solutions that are practical, affordable, and impactful.

By moving beyond simple reminder alarms and focusing on actual medication-taking behavior, MediTrack encourages accountability, consistency, and healthier long-term habits.

This project highlights the potential of the MYOSA platform in building real-world smart healthcare technologies that can improve quality of life, support caregivers, and contribute toward more preventive and connected healthcare systems.

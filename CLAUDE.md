# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This project is based on the ELEGOO smart robot car v4 hardware, but is customized to be part of a larger project and will not use the elegoo mobile app. The hardware consists of a ELEGOO branded Arduino Uno R3, shield/hat, and camera module running on an ESP32-S3-WROOM-1 chip.

## Build Commands

### ESP32-S3 BLE Relay Firmware

```bash
cd ~/workspace/elegoo-custom-firmware/firmware/esp32s3-ble-relay
pio run                    # Build
pio run -t upload          # Upload to device
pio device monitor         # Serial monitor
```

### ESP32-S3 WiFi/Camera Firmware

```bash
cd ~/workspace/elegoo-custom-firmware/firmware/esp32s3
pio run                    # Build
pio run -t upload          # Upload to device
pio device monitor         # Serial monitor
```

### Arduino Firmware

```bash
cd ~/workspace/elegoo-custom-firmware/firmware/arduino-bare
pio run                    # Build
pio run -t upload          # Upload to device
```

## Architecture

```
iPhone (iOS / ARKit)
│
│  BLE (Nordic UART Service)
│
ESP32-S3 (WROOM-1, no PSRAM) — esp32s3-ble-relay firmware
│
│  UART (GPIO3 RX, GPIO40 TX @ 115200 baud)
│
Arduino Uno (Elegoo Smart Robot Car V4.0 shield) — arduino-bare firmware
├── TB6612FNG Motor Driver (pins 3,5,6,7,8)
├── HC-SR04 Ultrasonic (pins 12,13)
├── WS2812B RGB LED (pin 4)
└── Battery voltage (A3)
```

### Legacy: WiFi/Camera Architecture (esp32s3 firmware)

```
PC (Ubuntu 24.04)
├── TCP:100 → ESP32-S3 → UART → Arduino
├── http://192.168.0.217
│    ├── /stream, /capture, /sensors, /state, /logs
```

## Hardware Notes

- ESP32-S3 UART to Arduino: GPIO3 (RX), GPIO40 (TX) @ 115200 baud
- No wheel encoders - iPhone ARKit provides spatial tracking
- BLE device name: "ElegooRelay"

## Behavioral Guidelines

- State assumptions explicitly before implementing
- Minimum code that solves the problem - no speculative features
- Touch only what you must - don't "improve" adjacent code
- Match existing style even if you'd do it differently
- Every changed line should trace directly to the user's request

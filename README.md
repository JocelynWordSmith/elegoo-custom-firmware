# elegoo-custom-firmware

Custom firmware for the ELEGOO Smart Robot Car V4.0 that replaces the stock Elegoo mobile app control with a stripped-down implementation designed for ROS 2 integration. The camera module is repurposed as a bottom-mounted optical flow sensor for visual odometry.

## Goals

- Replace Elegoo app control with direct TCP/HTTP interfaces
- Use the ESP32-S3 camera as an optical flow sensor for velocity estimation
- Provide clean integration points for ROS 2 nodes
- Maintain simple, minimal firmware focused on robotics use cases

## Directory Structure

```
elegoo-custom-firmware/
├── firmware/
│   ├── arduino/          # Arduino Uno firmware (motor control, sensors)
│   │   ├── src/          # Source code
│   │   └── platformio.ini
│   └── esp32s3/          # ESP32-S3 camera firmware (HTTP server, optical flow)
│       ├── src/          # Source code
│       └── platformio.ini
```

## Hardware

### Main Controller - Arduino Uno R3

| Component | Part | Notes |
|-----------|------|-------|
| Board | ELEGOO UNO R3 Car V2.0 | ATmega328P-based |
| MCU | AIMEL MEGA328P | Main processor |
| USB-Serial | WCH CH340C | USB to UART bridge |

### Shield - ELEGOO SmartCar-Shield-V1.1

| Component | Part | Notes |
|-----------|------|-------|
| Motor Driver | TB6612FNG | Dual H-bridge, pins 3,5,6,7,8 |
| IMU | MPU-6050 (GY-521 module) | 6-axis, I2C on A4/A5 |
| Ultrasonic | HC-SR04 | Pins 12,13 |
| IR Sensors | ITR20001 x3 | Line tracking, A0/A1/A2 |
| RGB LED | WS2812B | Pin 4 |
| Servos | Standard hobby servos x2 | Pan/tilt, pins 10,11 |

### Camera Module - ESP32-S3

| Component | Part | Notes |
|-----------|------|-------|
| MCU | ESP32-S3-WROOM-1 (N8R8) | 8MB Flash, 8MB PSRAM |
| Camera | OV2640 | 2MP, JPEG compression |
| Interface | WiFi 802.11 b/g/n | HTTP server on port 80 |

## Build Commands

### ESP32-S3 Firmware

```bash
cd firmware/esp32s3
pio run                    # Build
pio run -t upload          # Upload via USB
pio device monitor         # Serial monitor
```

### Arduino Firmware

```bash
cd firmware/arduino
pio run                    # Build
pio run -t upload          # Upload via USB
```

## Architecture

TBD

## License

This project uses code based on Espressif's esp32-camera examples (Apache 2.0) and is derived from ELEGOO's Smart Robot Car V4.0 firmware.

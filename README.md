# elegoo-custom-firmware

Custom firmware for the ELEGOO Smart Robot Car V4.0 that replaces the stock mobile app with HTTP/TCP interfaces for robotics applications. Designed for ROS 2 integration.

## Features

- **HTTP API** - RESTful endpoints for camera streaming, settings, and sensor readings.
- **TCP Bridge** - Serial passthrough on port 100 for low-latency control
- **MJPEG Streaming** - Real-time camera feed
- **OTA Updates** - Over-the-air firmware updates for the ESP32-S3
- **Motor Watchdog** - Automatic motor shutoff if communication is lost

## Prerequisites

- [PlatformIO](https://platformio.org/) (CLI or VS Code extension)
- ELEGOO Smart Robot Car V4.0 hardware

## Getting Started

### 1. Create credentials file

Create `firmware/esp32s3/src/credentials.h` (this file is not tracked in git):

```cpp
#ifndef CREDENTIALS_H
#define CREDENTIALS_H

const char *ssid = "your_wifi_ssid";
const char *password = "your_wifi_password";

const char *otaHostName = "ESP32-S3-Camera";
const char *otaPassword = "your_ota_password";

#endif
```

### 2. Upload Arduino firmware

Ensure the `upload`/`cam` switch on the shield is set to `upload`
Connect the Arduino Uno via USB and upload:

```bash
cd firmware/arduino
pio run -t upload
```

Move the switch on the shield to `cam` for ESP32S3 integration

### 3. Upload ESP32-S3 firmware

- Connect the ESP32-S3 camera module via USB
- Ensure the ESP32-S3 is in firmware upload mode (press and hold `boot`, press `rst`, release both)
- upload the firmware:

```bash
cd firmware/esp32s3
pio run -t upload
```

press `rst` to restart the module (reset by pin doesnt always work)

### 4. Verify connection

The ESP32-S3 will print its IP address to serial on boot:

```bash
pio device monitor
```

Open `http://<ip_address>/` in a browser to see the dashboard.

## Usage

### HTTP Endpoints (port 80)

| Endpoint               | Method | Description                                  |
| ---------------------- | ------ | -------------------------------------------- |
| `/`                    | GET    | Dashboard with all available endpoints       |
| `/info`                | GET    | Firmware version, WiFi info, camera settings |
| `/status`              | GET    | Uptime, heap memory, WiFi signal strength    |
| `/capture`             | GET    | Single JPEG image                            |
| `/stream`              | GET    | MJPEG video stream                           |
| `/control?var=X&val=Y` | GET    | Adjust camera settings                       |
| `/sensors`             | GET    | Read all Arduino sensors                     |
| `/state`               | GET    | Current motor/servo/LED state                |
| `/logs`                | GET    | Recent log messages (JSON)                   |
| `/restart`             | GET    | Reboot ESP32-S3                              |

### Camera Control Parameters

```
/control?var=framesize&val=8    # Resolution (0=96x96, 8=VGA, 13=UXGA)
/control?var=quality&val=10     # JPEG quality (0-63, lower=better)
/control?var=brightness&val=0   # Brightness (-2 to 2)
/control?var=vflip&val=1        # Vertical flip (0/1)
/control?var=hmirror&val=1      # Horizontal mirror (0/1)
```

### TCP Interface (port 100)

Connect via TCP for direct Arduino serial passthrough. Commands are JSON:

```json
{ "N": 2, "D1": 150 }
```

### Arduino Commands

| Cmd | Function                | Parameters                                  |
| --- | ----------------------- | ------------------------------------------- |
| 1   | Ping                    | -                                           |
| 2   | Forward                 | D1=speed (0-255)                            |
| 3   | Backward                | D1=speed                                    |
| 4   | Turn Left               | D1=speed                                    |
| 5   | Turn Right              | D1=speed                                    |
| 6   | Stop                    | -                                           |
| 7   | Tank Drive              | D1=left speed, D2=right speed (-255 to 255) |
| 8   | Set Default Speed       | D1=speed                                    |
| 10  | Get Ultrasonic Distance | -                                           |
| 11  | Get IR Sensors          | -                                           |
| 12  | Get IMU Data            | -                                           |
| 13  | Get Battery Voltage     | -                                           |
| 20  | Set LED Color           | D1=R, D2=G, D3=B                            |
| 21  | Set LED Brightness      | D1=brightness                               |
| 30  | Pan Servo               | D1=angle (0-180)                            |
| 31  | Tilt Servo              | D1=angle (0-180)                            |
| 100 | Get All Sensors         | -                                           |
| 101 | Get Current State       | -                                           |
| 102 | Set Watchdog Timeout    | D1=ms (0=disable)                           |

### Example: Control via curl

```bash
# Read all sensors
curl http://192.168.1.100/sensors

# View camera stream
vlc http://192.168.1.100/stream

# Send motor command via TCP
echo '{"N":2,"D1":100}' | nc 192.168.1.100 100
```

## OTA Updates

After initial USB upload, the ESP32-S3 supports over-the-air updates:

```bash
cd firmware/esp32s3
ESP_OTA_PASSWORD=your_ota_pw pio run -e esp32-s3-ota -t upload
```

Default OTA password is set in `credentials.h`.

## Architecture

```
┌─────────────────────────────────────────┐
│            Host Computer                │
│   (HTTP client / TCP client / ROS 2)    │
└──────────────────┬──────────────────────┘
                   │ WiFi
      ┌────────────┴────────────┐
      │      ESP32-S3           │
      │  ├─ HTTP Server (:80)   │
      │  ├─ TCP Bridge (:100)   │
      │  ├─ OV2640 Camera       │
      │  └─ OTA Updates         │
      └────────────┬────────────┘
                   │ UART (115200 baud)
      ┌────────────┴────────────┐
      │     Arduino Uno R3      │
      │  ├─ TB6612FNG Motors    │
      │  ├─ MPU-6050 IMU        │
      │  ├─ HC-SR04 Ultrasonic  │
      │  ├─ ITR20001 IR x3      │
      │  ├─ WS2812B LED         │
      │  └─ Pan/Tilt Servos     │
      └─────────────────────────┘
```

## Hardware Reference

### Main Controller - Arduino Uno R3

| Component  | Part                   | Notes              |
| ---------- | ---------------------- | ------------------ |
| Board      | ELEGOO UNO R3 Car V2.0 | ATmega328P-based   |
| MCU        | AIMEL MEGA328P         | Main processor     |
| USB-Serial | WCH CH340C             | USB to UART bridge |

### Shield - ELEGOO SmartCar-Shield-V1.1

| Component    | Part                     | Notes                         |
| ------------ | ------------------------ | ----------------------------- |
| Motor Driver | TB6612FNG                | Dual H-bridge, pins 3,5,6,7,8 |
| IMU          | MPU-6050 (GY-521 module) | 6-axis, I2C on A4/A5          |
| Ultrasonic   | HC-SR04                  | Pins 12,13                    |
| IR Sensors   | ITR20001 x3              | Line tracking, A0/A1/A2       |
| RGB LED      | WS2812B                  | Pin 4                         |
| Servos       | Standard hobby servos x2 | Pan/tilt, pins 10,11          |

### Camera Module - ESP32-S3

| Component | Part                    | Notes                  |
| --------- | ----------------------- | ---------------------- |
| MCU       | ESP32-S3-WROOM-1 (N8R8) | 8MB Flash, 8MB PSRAM   |
| Camera    | OV2640                  | 2MP, JPEG compression  |
| Interface | WiFi 802.11 b/g/n       | HTTP server on port 80 |

### Pin Assignments

**Arduino Uno:**

- Motors: PWM (5, 6), Direction (7, 8), Standby (3)
- Ultrasonic: Trigger (13), Echo (12)
- IR Sensors: A0, A1, A2
- IMU: I2C (A4/SDA, A5/SCL)
- Battery: A3
- RGB LED: 4
- Servos: Pan (10), Tilt (11)

**ESP32-S3:**

- Arduino UART: RX (GPIO3), TX (GPIO40)
- Camera: 8-bit parallel bus, I2C control

## Project Structure

```
elegoo-custom-firmware/
├── firmware/
│   ├── arduino/
│   │   ├── src/main.cpp       # Motor control, sensors, JSON protocol
│   │   └── platformio.ini
│   └── esp32s3/
│       ├── src/
│       │   ├── main.cpp           # WiFi, camera init, OTA
│       │   ├── http_server.cpp    # HTTP endpoint handlers
│       │   ├── camera_handlers.cpp
│       │   ├── arduino_bridge.cpp # UART/TCP bridge
│       │   ├── logging.cpp        # Circular log buffer
│       │   ├── credentials.h      # WiFi config
│       │   └── camera_pins.h      # GPIO mapping
│       └── platformio.ini
└── README.md
```

## License

This project uses code based on Espressif's esp32-camera examples (Apache 2.0) and is derived from ELEGOO's Smart Robot Car V4.0 firmware.

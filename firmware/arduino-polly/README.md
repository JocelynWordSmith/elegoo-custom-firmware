# Firmware

## What This Is

Arduino Uno firmware for the ELEGOO Smart Robot Car V4.0. Handles motor control, ultrasonic distance sensing, IMU (MPU-6050), battery monitoring, and an RGB LED. Communicates with the Android phone via JSON over USB serial at 115200 baud.

## Tech Stack

- Arduino Uno (ATmega328P, 8 MHz)
- PlatformIO build system
- C++ with Arduino framework
- Dependencies: FastLED 3.6.0

## Key Files

| File | Purpose | When to read |
|------|---------|-------------|
| `src/main.cpp` | All firmware logic (single file) | Always |
| `platformio.ini` | Build config, board, deps, serial port | Build issues |
| `test_serial.py` | Python script to test serial protocol | Testing serial |
| `test_sonar_detail.py` | Python script to test ultrasonic sensor | Testing sensors |

## Build & Run

```bash
cd firmware/arduino-polly
pio run                    # Build (no hardware needed)
pio run -t upload          # Flash to Arduino
pio device monitor         # Open serial monitor
pio test -e native         # Run unit tests
```

## Hardware Pin Assignments

| Pin | Function | Component |
|-----|----------|-----------|
| 3 | STBY (motor standby) | TB6612FNG |
| 4 | LED data | WS2812B RGB LED |
| 5 | PWMA (left motor speed) | TB6612FNG |
| 6 | PWMB (right motor speed) | TB6612FNG |
| 7 | AIN1 (left motor direction) | TB6612FNG |
| 8 | BIN1 (right motor direction) | TB6612FNG |
| 12 | ECHO (ultrasonic input) | HC-SR04 |
| 13 | TRIG (ultrasonic output) | HC-SR04 |
| A3 | Battery voltage | Voltage divider (1:2) |
| SDA/SCL | I2C | MPU-6050 (addr 0x68) |

## Serial Protocol

JSON commands at 115200 baud. Format: `{"N": <cmd>, "D1": <val>, "D2": <val>, "D3": <val>}`

### Commands (send to Arduino)

| N | Command | D1 | D2 | D3 |
|---|---------|----|----|-----|
| 1 | Ping | — | — | — |
| 2 | Forward | speed (opt) | — | — |
| 3 | Backward | speed (opt) | — | — |
| 4 | Turn left | speed (opt) | — | — |
| 5 | Turn right | speed (opt) | — | — |
| 6 | Stop (instant) | — | — | — |
| 7 | Tank drive | left (-255..255) | right (-255..255) | — |
| 8 | Set default speed | speed (0-255) | — | — |
| 10 | Get distance | — | — | — |
| 12 | Get MPU data | — | — | — |
| 13 | Get battery voltage | — | — | — |
| 14 | Calibrate battery | actual_V * 100 | — | — |
| 15 | Set motor bias | left% (80-120) | right% (80-120) | — |
| 20 | Set LED color | R (0-255) | G (0-255) | B (0-255) |
| 21 | Set LED brightness | brightness (0-255) | — | — |
| 100 | Get all sensors | — | — | — |
| 101 | Get state | — | — | — |
| 102 | Set watchdog timeout | timeout_ms (0=off) | — | — |
| 103 | Set sensor stream | interval_ms (0=off) | — | — |
| 104 | Set acceleration | — | maxAccelPerTick | — |
| 105 | Get firmware version | — | — | — |

### Key Responses

**All sensors (N=100):**
```json
{"t":12345,"u":5648,"d":18,"a":[888,-192,17936],"g":[-611,122,-114],"c":26.9,"b":1.14,"m":1}
```

**State (N=101):**
```json
{"t":12345,"M":[0,0],"T":[0,0],"l":[0,0,0],"B":50,"s":150,"w":1000,"mb":[1.00,1.00],"br":2.000,"mp":1,"ma":20,"st":0}
```

**Firmware version (N=105):**
```json
{"fv":"0.1.2"}
```

## Safety Features

- **Motor watchdog:** Auto-stops if no motor command received within timeout (default 1s)
- **Speed ramping:** 50Hz ramp loop limits acceleration to `maxAccelPerTick` per 20ms
- **E-stop:** Command N=6 immediately zeroes both target and current speeds

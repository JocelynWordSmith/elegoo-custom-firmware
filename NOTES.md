# NOTES.md - Planned Improvements

Revised: 2026-02-18

---

## Part 1: ESP32-S3 Streaming & Real-Time Data

### WebSocket Push Sensor Data

**Problem:** Every `/sensors` call is a full HTTP request/response cycle. Real-time robotics needs push-based data.

**Fix:** Add WebSocket endpoint:

```cpp
// New endpoint: ws://<ip>/ws
// ESP32 pushes sensor data at fixed interval (e.g., 50ms)
// Client subscribes to specific data types
```

**Implementation notes:**
- `ESPAsyncWebServer` has native WebSocket support
- Protocol: JSON messages with `type` field for subscription management

---

### Alternative: UDP Sensor Broadcast

- UDP port broadcasting sensor JSON at ~20Hz to multicast address
- Clients listen passively, no handshaking overhead
- Simpler than WebSocket; good for multiple read-only subscribers

---

### RTSP or WebSocket Video Streaming

**Current:** MJPEG over HTTP with a 3-client limit.

**Options:**
1. **RTSP server** — esp32-camera library supports it; compatible with VLC/GStreamer/OpenCV
2. **WebSocket-based streaming** — lower overhead than MJPEG HTTP chunking; easier browser integration

---

### Resolution Presets Endpoint

```
/preset?mode=navigation  → QVGA, quality=15, 30fps target
/preset?mode=inspection  → VGA, quality=8, 15fps target
/preset?mode=detail      → XGA, quality=5, 10fps target
```

---

### Binary Protocol Option

- JSON is readable but wasteful (~200 bytes per sensor read)
- Add packed binary mode: ~24 bytes for same data
- New endpoint: `/sensors?format=binary`

---

## Part 2: Arduino On-Device Functionality

### IMU Filtering

**Problem:** Raw MPU-6050 data is noisy and unusable for orientation estimation.

**Fix:** On-device filtering:

```cpp
// Options (in order of complexity):
// 1. Simple moving average (least CPU)
// 2. Complementary filter (good balance)
// 3. Madgwick/Mahony filter (best quality, still fits on Uno)
```

**Note:** Madgwick filter needs ~1KB RAM, runs at 100Hz on Uno.

**Command number conflict:** Cases 14, 15, 16 are already in use (battery calibrate, motor bias,
IR threshold). New IMU commands should use 17+:

```cpp
case 17: // Get filtered orientation: {"orientation":[roll, pitch, yaw]} (degrees)
case 18: // Calibrate IMU: sets current orientation as zero reference
case 19: // Set filter type: D1=0(none), 1(average), 2(complementary), 3(madgwick)
```

---

### Autonomous Behaviors

**Purpose:** Run on Arduino so they continue working if WiFi drops.

```cpp
case 50: // Line follow: D1=speed, D2=Kp*100
case 51: // Obstacle avoidance: D1=speed, D2=min_distance_cm
case 52: // Wall follow: D1=speed, D2=target_distance_cm, D3=side(0=left,1=right)
case 54: // Stop autonomous mode
case 55: // Get autonomous status: returns mode, progress, etc.
```

---

### PID Motor Control

**Problem:** PWM ≠ speed. Output varies with battery voltage, friction, and motor differences.

**Prerequisite:** Requires encoder motors (hardware upgrade). Current motor bias correction
(`case 15`) is a workaround, not closed-loop control.

```cpp
case 40: // Set target velocity (cm/s): D1=left, D2=right
case 41: // Set PID gains: D1=Kp*100, D2=Ki*100, D3=Kd*100
case 42: // Enable/disable PID: D1=0/1
case 43: // Get PID status: returns current gains, error, output
```

---

### Safety Features

**Not yet implemented:**

```cpp
// Command number conflict: 103 and 104 are already used in arduino-bare
// (stream interval and acceleration ramp). Use 105+ for new safety commands.
case 105: // Emergency stop (immediate halt, clears autonomous modes)
case 106: // Get fault status: {"faults":{"stall":false,"low_battery":false,"imu_error":false}}
case 107: // Set speed limit: D1=max_speed (0-255)
```

**Battery low-voltage protection** (voltage already read via `case 13`, just needs threshold logic):
- Warning push notification via ESP32 at 6.8V
- Auto-stop motors at 6.4V to prevent cell damage

**Note on E-stop:** Previously implemented and removed due to responsiveness issues (commit
`e4e4c28`). Any re-implementation needs to address the latency problem before re-adding.

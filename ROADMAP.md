# Testing Roadmap

Future testing work that cannot be safely done with the current USB-tethered setup.

## Motor Command Tests (N=2-7)

The robot must be either on a test stand or have wheels off the ground to safely test forward/backward/left/right/stop commands while connected via USB. Options:

- **Test jig**: 3D-print a stand that lifts the wheels off the surface
- **BLE-based testing**: Run motor tests wirelessly through the full iOS → BLE → ESP32 → UART → Arduino chain, removing the USB tether constraint
- **Programmatic safety**: Add a firmware test mode that limits motor duration to 500ms and speed to 50

## BLE End-to-End Tests

Test the full iPhone → BLE → ESP32 → UART → Arduino path. Requires a BLE test client:

- Python `bleak` library script that connects to "ElegooRelay" and sends commands
- Swift command-line tool using CoreBluetooth
- Validates that BLE fragmentation/reassembly works for large responses (N=101 full state)

## Streaming Under Load (N=103)

Validate that sensor streaming doesn't cause motor ramp jitter during simultaneous motor commands. Requires:

- Concurrent stream + motor command test
- Timing analysis of response latencies under load
- Check for dropped or corrupted UART frames

## Watchdog Safety Test

Verify motors stop on connection loss:

- Connect via BLE, start motors, disconnect BLE abruptly
- Confirm motors stop within watchdog timeout period
- Requires wireless testing (can't simulate BLE disconnect over USB)

## Battery Calibration Test (N=14)

Requires a known reference voltage source to validate the ADC-to-voltage conversion and calibrate `BATTERY_DIVIDER_RATIO`.

## Dependency Injection Refactoring

Extract `processCommand()` response-building into pure functions that return strings instead of calling `Serial.println()` directly. This would enable Tier 1 testing of the full command dispatch logic without hardware. Currently deferred because:

- The existing `robot_logic.h` extraction covers the most error-prone math
- Full DI would require significant refactoring of `main.cpp`
- Risk of introducing bugs in the production path outweighs the testing benefit right now

#!/usr/bin/env python3
"""Serial test script for arduino-bare firmware.
Validates all commands and sensor readings."""

import serial
import json
import os
import time
import sys

PORT = os.environ.get("ARDUINO_PORT", "/dev/cu.usbserial-14130")
BAUD = 115200
TIMEOUT = 3  # seconds per read

def send_cmd(ser, cmd_str, label=""):
    """Send a JSON command and return parsed response."""
    ser.reset_input_buffer()
    ser.write((cmd_str + "\n").encode())
    time.sleep(0.1)
    line = ser.readline().decode(errors='replace').strip()
    if label:
        print(f"  [{label}] sent: {cmd_str}")
        print(f"  [{label}] recv: {line}")
    try:
        return json.loads(line)
    except json.JSONDecodeError:
        print(f"  WARNING: could not parse JSON: {line!r}")
        return None

def main():
    print(f"Opening {PORT} at {BAUD} baud...")
    ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)

    # Wait for Arduino bootloader to finish
    print("Waiting 3s for bootloader...")
    time.sleep(3)
    ser.reset_input_buffer()

    passed = 0
    failed = 0

    def check(condition, name):
        nonlocal passed, failed
        if condition:
            print(f"  PASS: {name}")
            passed += 1
        else:
            print(f"  FAIL: {name}")
            failed += 1

    # === Test 1: Ping ===
    print("\n=== Test 1: Ping (N:1) ===")
    resp = send_cmd(ser, '{"N":1}', "ping")
    check(resp and resp.get("ok") == True, "ping responds ok:true")

    # === Test 2: Get all sensors (N:100) ===
    # Wire format uses short keys: t, u, d, a, g, c, b, m
    print("\n=== Test 2: Sensor dump (N:100) ===")
    resp = send_cmd(ser, '{"N":100}', "sensors")
    if resp:
        check("t" in resp, "has timestamp (t)")
        check("u" in resp, "has exec time (u)")
        check("d" in resp, "has distance (d)")
        check("a" in resp and len(resp["a"]) == 3, "has accel[3] (a)")
        check("g" in resp and len(resp["g"]) == 3, "has gyro[3] (g)")
        check("c" in resp, "has temp (c)")
        check("b" in resp, "has battery (b)")
        check("m" in resp, "has mpuValid (m)")

        # Validate no IR fields present (removed)
        check("ir" not in resp, "IR field removed")
        check("onLine" not in resp, "onLine field removed")

        # Check sensor values
        dist = resp.get("d", -1)
        print(f"\n  Sonar distance: {dist} cm")
        check(dist >= 0, "sonar >= 0 (0=out of range is ok)")

        batt = resp.get("b", 0)
        print(f"  Battery voltage: {batt}V")
        check(batt > 0, "battery > 0V")
        if batt < 5.0:
            print(f"  NOTE: Battery reads {batt}V — may need calibration (expected 6.0-8.4V for 2x18650)")

        accel = resp.get("a", [0,0,0])
        gyro = resp.get("g", [0,0,0])
        temp = resp.get("c", 0)
        mpu_valid = resp.get("m", 0)
        print(f"  MPU valid: {mpu_valid}")
        print(f"  Accel: {accel}")
        print(f"  Gyro: {gyro}")
        print(f"  Temp: {temp}C")

        if mpu_valid:
            # At rest, az should be ~16384 (1g at default ±2g scale)
            check(abs(accel[2]) > 10000, "az ~16384 (gravity)")
            check(abs(accel[0]) < 5000 and abs(accel[1]) < 5000, "ax,ay near zero at rest")
            check(temp > 10 and temp < 50, "temp in reasonable range (10-50C)")
        else:
            print("  SKIP: MPU not detected, skipping accel/gyro checks")

        exec_us = resp.get("u", 0)
        print(f"  Exec time: {exec_us} us")
        check(exec_us > 0 and exec_us < 50000, "exec time < 50ms")
    else:
        check(False, "N:100 returned valid response")

    # === Test 3: Battery raw ADC (N:13) ===
    # Wire format: {"b":<volts>,"r":<raw>}
    print("\n=== Test 3: Battery with raw ADC (N:13) ===")
    resp = send_cmd(ser, '{"N":13}', "battery")
    if resp:
        raw = resp.get("r", 0)
        batt = resp.get("b", 0)
        print(f"  Raw ADC: {raw} (0-1023)")
        print(f"  Calculated voltage: {batt}V")
        print(f"  ADC voltage: {raw * 5.0 / 1023:.3f}V")
        print(f"  With ratio {2.0}: {raw * 5.0 / 1023 * 2.0:.2f}V")
        check(raw > 0, "raw ADC > 0")

    # === Test 4: Sonar (N:10) ===
    # Wire format: {"d":<cm>}
    print("\n=== Test 4: Sonar direct read (N:10) ===")
    readings = []
    for i in range(5):
        resp = send_cmd(ser, '{"N":10}')
        if resp:
            readings.append(resp.get("d", -1))
        time.sleep(0.05)
    print(f"  5 readings: {readings}")
    zero_count = readings.count(0)
    print(f"  Zero count (out-of-range): {zero_count}/5")
    check(len(readings) == 5, "got 5 readings")
    # If all zeros, sensor might not be connected or everything is far away
    if all(r == 0 for r in readings):
        print("  NOTE: All readings are 0 — nothing in range or sensor disconnected")

    # === Test 5: State dump (N:101) ===
    # Wire format uses short keys: t, M, T, l, B, s, w, mb, br, mp, ma, st
    print("\n=== Test 5: State dump (N:101) ===")
    resp = send_cmd(ser, '{"N":101}', "state")
    if resp:
        check("M" in resp and len(resp["M"]) == 2, "has motors[2] (M)")
        check("T" in resp and len(resp["T"]) == 2, "has targets[2] (T)")
        check("st" in resp, "has stream field (st)")
        check("w" in resp, "has watchdog field (w)")
        check("s" in resp, "has speed field (s)")
        check("ma" in resp, "has maxAccel field (ma)")
        check("mp" in resp, "has mpuPresent field (mp)")

    # === Test 6: Acceleration config (N:104) ===
    print("\n=== Test 6: Acceleration config (N:104) ===")
    resp = send_cmd(ser, '{"N":104,"D2":5}', "set_accel")
    if resp:
        check(resp.get("ma") == 5, "maxAccel set to 5")

    # Reset to default
    send_cmd(ser, '{"N":104,"D2":20}', "reset_accel")

    # === Test 7: Streaming mode (N:103) ===
    # Wire format: {"st":<ms>}, streamed N=100 uses short keys
    print("\n=== Test 7: Streaming mode (N:103) ===")
    resp = send_cmd(ser, '{"N":103,"D1":200}', "stream_on")
    check(resp and resp.get("st") == 200, "stream interval set to 200ms")

    # Read a few streamed messages
    print("  Reading 3 streamed messages...")
    stream_msgs = []
    for i in range(3):
        line = ser.readline().decode(errors='replace').strip()
        if line:
            try:
                msg = json.loads(line)
                stream_msgs.append(msg)
                print(f"  stream[{i}]: t={msg.get('t')} d={msg.get('d')} b={msg.get('b')}")
            except json.JSONDecodeError:
                print(f"  stream[{i}]: (not JSON) {line!r}")

    check(len(stream_msgs) >= 2, "received at least 2 stream messages")
    if len(stream_msgs) >= 2:
        dt = stream_msgs[1].get("t", 0) - stream_msgs[0].get("t", 0)
        print(f"  Time between stream msgs: {dt}ms (expected ~200ms)")
        check(dt > 100 and dt < 400, "stream interval roughly 200ms")

    # Disable streaming
    resp = send_cmd(ser, '{"N":103,"D1":0}', "stream_off")
    check(resp and resp.get("st") == 0, "streaming disabled")
    time.sleep(0.3)
    ser.reset_input_buffer()

    # === Test 8: Watchdog config (N:102) ===
    # Wire format: {"w":<ms>}
    print("\n=== Test 8: Watchdog config (N:102) ===")
    resp = send_cmd(ser, '{"N":102,"D1":1000}', "watchdog")
    check(resp and resp.get("w") == 1000, "watchdog set to 1000ms")
    # Reset to default
    send_cmd(ser, '{"N":102,"D1":1000}', "reset_watchdog")

    # === Test 9: Motor commands (target-based) ===
    print("\n=== Test 9: Motor commands (target-based, wheels off ground) ===")

    # Disable watchdog for motor test
    send_cmd(ser, '{"N":102,"D1":0}', "disable_watchdog")

    # Forward
    resp = send_cmd(ser, '{"N":2,"D1":100}', "forward")
    check(resp and resp.get("cmd") == "forward", "forward acknowledged")
    time.sleep(0.3)  # let ramp apply
    state = send_cmd(ser, '{"N":101}', "state_during_forward")
    if state:
        targets = state.get("T", [0,0])
        motors = state.get("M", [0,0])
        print(f"  targets: {targets}, motors: {motors}")
        check(targets[0] == 100 and targets[1] == 100, "targets set to 100,100")
        check(motors[0] > 0 and motors[1] > 0, "motors ramping up")

    # Stop
    resp = send_cmd(ser, '{"N":6}', "stop")
    check(resp and resp.get("cmd") == "stop", "stop acknowledged")
    state = send_cmd(ser, '{"N":101}', "state_after_stop")
    if state:
        targets = state.get("T", [1,1])
        motors = state.get("M", [1,1])
        check(targets == [0,0], "targets zeroed on stop")
        check(motors == [0,0], "motors zeroed on stop (instant)")

    # Tank drive (N=7 has no serial response)
    send_cmd(ser, '{"N":7,"D1":80,"D2":-80}', "tank")
    time.sleep(0.3)

    # Stop
    send_cmd(ser, '{"N":6}', "stop")
    time.sleep(0.1)

    # Re-enable watchdog
    send_cmd(ser, '{"N":102,"D1":1000}', "restore_watchdog")

    # === Test 10: Removed commands should return error ===
    print("\n=== Test 10: Removed commands ===")
    for cmd_n in [11, 16, 30, 31]:
        resp = send_cmd(ser, f'{{"N":{cmd_n}}}', f"removed_N{cmd_n}")
        check(resp and "error" in resp, f"N:{cmd_n} returns error (removed)")

    # === Test 11: LED control ===
    # Wire format: {"l":[R,G,B]}
    print("\n=== Test 11: LED control (N:20/21) ===")
    resp = send_cmd(ser, '{"N":20,"D1":0,"D2":0,"D3":255}', "led_blue")
    check(resp and resp.get("l") == [0, 0, 255], "LED set to blue")
    time.sleep(0.5)
    send_cmd(ser, '{"N":20,"D1":0,"D2":0,"D3":0}', "led_off")

    # === Test 12: MPU direct read (N:12) ===
    # Wire format: {"a":[...],"c":<temp>,"g":[...]}
    print("\n=== Test 12: MPU direct read (N:12) ===")
    resp = send_cmd(ser, '{"N":12}', "mpu")
    if resp and "a" in resp:
        print(f"  Accel: {resp['a']}")
        print(f"  Gyro: {resp['g']}")
        print(f"  Temp: {resp['c']}C")
        check(True, "MPU data received")
    else:
        print("  MPU might not be connected")

    # === Test 13: Firmware version (N:105) ===
    # Wire format: {"fv":"<semver>"}
    print("\n=== Test 13: Firmware version (N:105) ===")
    resp = send_cmd(ser, '{"N":105}', "version")
    if resp:
        ver = resp.get("fv", "")
        print(f"  Firmware version: {ver}")
        check(isinstance(ver, str) and len(ver) > 0, "version is non-empty string")
        check("." in ver, "version contains dot (semver)")
    else:
        check(False, "N:105 returned valid response")

    # === Summary ===
    print(f"\n{'='*50}")
    print(f"RESULTS: {passed} passed, {failed} failed")
    print(f"{'='*50}")

    ser.close()
    return 0 if failed == 0 else 1

if __name__ == "__main__":
    sys.exit(main())

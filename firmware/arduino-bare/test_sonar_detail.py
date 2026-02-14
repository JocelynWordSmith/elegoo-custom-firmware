#!/usr/bin/env python3
"""Detailed sonar analysis to understand 0-reading behavior."""

import serial
import json
import time

PORT = "/dev/cu.usbserial-14130"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=3)
time.sleep(3)
ser.reset_input_buffer()

print("=== Sonar Direct Read Test (N:10, 20 readings, 100ms apart) ===")
readings = []
for i in range(20):
    ser.reset_input_buffer()
    ser.write(b'{"N":10}\n')
    time.sleep(0.1)
    line = ser.readline().decode(errors='replace').strip()
    try:
        d = json.loads(line).get("distance", "?")
        readings.append(d)
    except:
        readings.append("ERR")

print(f"Readings: {readings}")
zeros = [i for i, r in enumerate(readings) if r == 0]
print(f"Zero indices: {zeros}")
print(f"Zero count: {len(zeros)}/20")
non_zero = [r for r in readings if isinstance(r, int) and r > 0]
if non_zero:
    print(f"Non-zero range: {min(non_zero)}-{max(non_zero)} cm")

print("\n=== Sonar via Streaming (50ms interval, safety loop reads) ===")
ser.reset_input_buffer()
ser.write(b'{"N":103,"D1":100}\n')
time.sleep(0.1)
ser.readline()  # consume the {"stream":100} response

stream_dists = []
for i in range(20):
    line = ser.readline().decode(errors='replace').strip()
    try:
        msg = json.loads(line)
        d = msg.get("dist_f", "?")
        stream_dists.append(d)
    except:
        stream_dists.append("ERR")

print(f"Streamed dist_f: {stream_dists}")
zeros = [i for i, r in enumerate(stream_dists) if r == 0]
print(f"Zero indices: {zeros}")
print(f"Zero count: {len(zeros)}/20")

# Disable streaming
ser.write(b'{"N":103,"D1":0}\n')
time.sleep(0.2)
ser.readline()

print("\n=== Battery Deep Dive ===")
ser.reset_input_buffer()
# Read battery 5 times
for i in range(5):
    ser.write(b'{"N":13}\n')
    time.sleep(0.1)
    line = ser.readline().decode(errors='replace').strip()
    print(f"  Battery [{i}]: {line}")

# Check what actual voltage would need the ratio to be
print("\n  If actual battery is 7.4V (nominal 2x18650):")
print(f"  Required ratio = 7.4 / 0.572 = {7.4 / 0.572:.1f}")
print(f"  If actual battery is 8.0V (freshly charged):")
print(f"  Required ratio = 8.0 / 0.572 = {8.0 / 0.572:.1f}")
print("  The Elegoo shield likely has a larger voltage divider than 2:1")

ser.close()

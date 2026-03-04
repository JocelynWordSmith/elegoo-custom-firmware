#!/usr/bin/env python3
"""Record serial command/response pairs from live Arduino for Tier 2 replay.

Sends the full command suite to the Arduino via USB serial, captures tx/rx
pairs to a JSONL fixture file.

Usage:
    python3 scripts/record_serial.py --port /dev/cu.usbserial-14130
    ELEGOO_PORT=/dev/cu.usbserial-14130 python3 scripts/record_serial.py
"""

import argparse
import json
import os
import sys
import time
import serial


def send_and_record(ser, cmd_str, out_file, timeout=2.0):
    """Send a command and record the tx/rx pair. Returns response or None."""
    ser.reset_input_buffer()
    t_tx = time.time()
    entry_tx = {"t": t_tx, "dir": "tx", "raw": cmd_str}
    out_file.write(json.dumps(entry_tx) + "\n")

    ser.write((cmd_str + "\n").encode())

    deadline = time.time() + timeout
    buf = b""
    while time.time() < deadline:
        if ser.in_waiting:
            buf += ser.read(ser.in_waiting)
            if b"\n" in buf:
                break
        time.sleep(0.01)

    if not buf.strip():
        return None

    # Take first complete line
    raw_resp = buf.split(b"\n")[0].decode(errors="replace").strip()
    t_rx = time.time()
    entry_rx = {"t": t_rx, "dir": "rx", "raw": raw_resp}
    out_file.write(json.dumps(entry_rx) + "\n")
    return raw_resp


# Commands to record (N value, JSON command string, description)
COMMANDS = [
    (1,   '{"N":1}',                         "ping"),
    (105, '{"N":105}',                       "firmware version"),
    (10,  '{"N":10}',                        "distance"),
    (13,  '{"N":13}',                        "battery"),
    (100, '{"N":100}',                       "all sensors"),
    (101, '{"N":101}',                       "full state"),
    (8,   '{"N":8,"D1":150}',                "set speed 150"),
    (102, '{"N":102,"D1":5000}',             "set watchdog 5s"),
    (103, '{"N":103,"D1":0}',                "stream off"),
    (104, '{"N":104,"D1":20}',               "set accel 20"),
    (15,  '{"N":15,"D1":100,"D2":100}',      "motor bias 1.0/1.0"),
    (14,  '{"N":14,"D1":200}',               "battery ratio 2.0"),
    (20,  '{"N":20,"D1":0,"D2":0,"D3":0}',   "LED off"),
    (21,  '{"N":21,"D1":128}',               "brightness 128"),
    (2,   '{"N":2}',                         "forward"),
    (6,   '{"N":6}',                         "stop"),
    (3,   '{"N":3}',                         "backward"),
    (6,   '{"N":6}',                         "stop"),
    (4,   '{"N":4}',                         "left"),
    (6,   '{"N":6}',                         "stop"),
    (5,   '{"N":5}',                         "right"),
    (6,   '{"N":6}',                         "stop"),
    (999, '{"N":999}',                       "unknown command"),
    (None, '{"foo":"bar"}',                  "missing N"),
]


def main():
    parser = argparse.ArgumentParser(description="Record serial fixture from live Arduino")
    parser.add_argument("--port", default=os.environ.get("ELEGOO_PORT"),
                        help="Serial port (or set ELEGOO_PORT env var)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--output", "-o",
                        help="Output JSONL path (default: auto-named in test-fixtures/recordings/)")
    args = parser.parse_args()

    if not args.port:
        print("Error: no port specified. Use --port or set ELEGOO_PORT.", file=sys.stderr)
        sys.exit(1)

    if args.output:
        out_path = args.output
    else:
        date_str = time.strftime("%Y-%m-%d")
        out_path = f"test-fixtures/recordings/{date_str}_recording.jsonl"

    print(f"Port: {args.port}", file=sys.stderr)
    print(f"Output: {out_path}", file=sys.stderr)

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except serial.SerialException as e:
        print(f"Error opening port: {e}", file=sys.stderr)
        sys.exit(1)

    # Wait for bootloader
    print("Waiting 3s for bootloader...", file=sys.stderr)
    time.sleep(3)
    ser.reset_input_buffer()

    recorded = 0
    timeouts = 0

    with open(out_path, "w") as out_file:
        for n, cmd, desc in COMMANDS:
            label = f"N={n}" if n is not None else "no-N"
            resp = send_and_record(ser, cmd, out_file)
            if resp:
                recorded += 1
                print(f"  {label:>6}  {desc:<20}  -> {resp[:80]}", file=sys.stderr)
            else:
                timeouts += 1
                print(f"  {label:>6}  {desc:<20}  -> TIMEOUT", file=sys.stderr)
            time.sleep(0.1)

    ser.close()

    summary = {
        "sent": len(COMMANDS),
        "recorded": recorded,
        "timeouts": timeouts,
        "output": out_path,
    }
    print(json.dumps(summary))


if __name__ == "__main__":
    main()

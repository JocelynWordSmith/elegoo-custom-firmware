#!/usr/bin/env python3
"""Tier 3: Hardware integration tests using safe (non-motor) commands.

Connects to the Arduino via USB serial and runs diagnostic sequences
that do NOT cause the robot to move. Safe to run while tethered by USB.

Usage:
    python3 scripts/test_hardware.py --port /dev/cu.usbserial-14130
    ELEGOO_PORT=/dev/cu.usbserial-14130 python3 scripts/test_hardware.py
"""

import argparse
import json
import os
import re
import sys
import time
import serial


def send_cmd(ser, cmd_str, timeout=2.0):
    """Send a JSON command and return parsed response dict, or None on timeout."""
    ser.reset_input_buffer()
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

    raw = buf.split(b"\n")[0].decode(errors="replace").strip()
    try:
        return json.loads(raw)
    except json.JSONDecodeError:
        return {"_raw": raw}


class HardwareTest:
    """Run safe hardware tests and collect results."""

    def __init__(self, ser, record_file=None):
        self.ser = ser
        self.record_file = record_file
        self.results = []

    def _record(self, direction, raw_str):
        if self.record_file:
            entry = {"t": time.time(), "dir": direction, "raw": raw_str}
            self.record_file.write(json.dumps(entry) + "\n")

    def send(self, cmd_str, timeout=2.0):
        """Send command with optional recording."""
        self._record("tx", cmd_str)
        resp = send_cmd(self.ser, cmd_str, timeout)
        if resp is not None:
            self._record("rx", json.dumps(resp))
        return resp

    def check(self, name, cmd_str, validator, timeout=2.0):
        """Run a single test: send command, validate response."""
        resp = self.send(cmd_str, timeout)
        if resp is None:
            self.results.append({"test": name, "status": "fail", "reason": "timeout"})
            return False

        ok, reason = validator(resp)
        if ok:
            self.results.append({"test": name, "status": "pass"})
            return True
        else:
            self.results.append({"test": name, "status": "fail", "reason": reason})
            return False

    def run_all(self):
        """Run all safe hardware tests."""
        # 1. Ping
        self.check("ping", '{"N":1}',
                    lambda r: (r.get("ok") is True, f"expected ok:true, got {r}"))

        # 2. Firmware version
        self.check("version", '{"N":105}',
                    lambda r: (isinstance(r.get("fv"), str) and re.match(r"\d+\.\d+\.\d+", r["fv"]),
                               f"expected fv:semver, got {r.get('fv')}"))

        # 3. Distance sensor
        self.check("distance", '{"N":10}',
                    lambda r: (isinstance(r.get("d"), int) and r["d"] >= -1,
                               f"expected d:int >= -1, got {r.get('d')}"))

        # 4. Battery voltage
        self.check("battery", '{"N":13}',
                    lambda r: ("b" in r and "raw" in r and isinstance(r["raw"], int),
                               f"expected b+raw keys, got {list(r.keys())}"))

        # 5. All sensors
        self.check("all_sensors", '{"N":100}',
                    lambda r: (all(k in r for k in ["t", "u", "d", "b"]),
                               f"expected t,u,d,b keys, got {list(r.keys())}"))

        # 6. Full state
        self.check("full_state", '{"N":101}',
                    lambda r: (all(k in r for k in ["t", "M", "T", "l", "B", "s", "w", "mb", "br", "ma", "st"]),
                               f"missing state keys, got {list(r.keys())}"))

        # 7. LED color (set then reset)
        self.check("led_color", '{"N":20,"D1":0,"D2":50,"D3":0}',
                    lambda r: (isinstance(r.get("l"), list) and len(r["l"]) == 3,
                               f"expected l:[r,g,b], got {r.get('l')}"))
        self.send('{"N":20,"D1":0,"D2":0,"D3":0}')  # reset LED off

        # 8. Brightness (set then reset)
        self.check("brightness", '{"N":21,"D1":64}',
                    lambda r: (isinstance(r.get("B"), int) and 0 <= r["B"] <= 255,
                               f"expected B:int 0-255, got {r.get('B')}"))
        self.send('{"N":21,"D1":128}')  # restore default

        # 9. Set speed (config only, no motion)
        self.check("set_speed", '{"N":8,"D1":150}',
                    lambda r: (isinstance(r.get("s"), int) and 0 <= r["s"] <= 255,
                               f"expected s:int 0-255, got {r.get('s')}"))

        # 10. Watchdog (set then restore)
        # First read current value
        state = self.send('{"N":101}')
        orig_watchdog = state.get("w", 5000) if state else 5000

        self.check("watchdog", '{"N":102,"D1":8000}',
                    lambda r: (isinstance(r.get("w"), int) and r["w"] >= 0,
                               f"expected w:int >= 0, got {r.get('w')}"))
        self.send(f'{{"N":102,"D1":{orig_watchdog}}}')  # restore

        # 11. Stream interval (set then disable)
        self.check("stream", '{"N":103,"D1":500}',
                    lambda r: (isinstance(r.get("st"), int) and r["st"] >= 0,
                               f"expected st:int >= 0, got {r.get('st')}"))
        self.send('{"N":103,"D1":0}')  # disable streaming

        # 12. Max acceleration (set then restore)
        orig_accel = state.get("ma", 20) if state else 20
        self.check("max_accel", '{"N":104,"D1":30}',
                    lambda r: (isinstance(r.get("ma"), int) and r["ma"] > 0,
                               f"expected ma:int > 0, got {r.get('ma')}"))
        self.send(f'{{"N":104,"D1":{orig_accel}}}')  # restore

        # 13. Motor bias (set then restore)
        orig_bias = state.get("mb", [100, 100]) if state else [100, 100]
        self.check("motor_bias", '{"N":15,"D1":110,"D2":90}',
                    lambda r: (isinstance(r.get("mb"), list) and len(r["mb"]) == 2,
                               f"expected mb:[l,r], got {r.get('mb')}"))
        lb = orig_bias[0] if isinstance(orig_bias, list) else 100
        rb = orig_bias[1] if isinstance(orig_bias, list) else 100
        self.send(f'{{"N":15,"D1":{lb},"D2":{rb}}}')  # restore

        # 14. Unknown command (error handling)
        self.check("unknown_cmd", '{"N":999}',
                    lambda r: ("err" in r, f"expected err key, got {list(r.keys())}"))

        # 15. Missing N field (error handling)
        self.check("missing_n", '{"foo":"bar"}',
                    lambda r: ("err" in r, f"expected err key, got {list(r.keys())}"))

        # Ensure motors are stopped
        self.send('{"N":6}')

    def summary(self):
        """Return compact JSON summary."""
        passed = sum(1 for r in self.results if r["status"] == "pass")
        failed = sum(1 for r in self.results if r["status"] == "fail")
        errors = [r for r in self.results if r["status"] == "fail"]
        return {
            "tier": 3,
            "total": len(self.results),
            "pass": passed,
            "fail": failed,
            "errors": errors,
        }


def main():
    parser = argparse.ArgumentParser(description="Tier 3: Safe hardware tests (no motor commands)")
    parser.add_argument("--port", default=os.environ.get("ELEGOO_PORT"),
                        help="Serial port (or set ELEGOO_PORT env var)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--record", metavar="PATH",
                        help="Simultaneously save tx/rx pairs to JSONL file")
    args = parser.parse_args()

    if not args.port:
        print(json.dumps({"tier": 3, "status": "skipped", "reason": "no port specified"}))
        sys.exit(0)

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except serial.SerialException:
        print(json.dumps({"tier": 3, "status": "skipped", "reason": "port unavailable"}))
        sys.exit(0)

    # Wait for bootloader
    time.sleep(3)
    ser.reset_input_buffer()

    record_file = None
    if args.record:
        record_file = open(args.record, "w")

    try:
        tester = HardwareTest(ser, record_file)
        tester.run_all()
        result = tester.summary()
        print(json.dumps(result))
        sys.exit(0 if result["fail"] == 0 else 1)
    finally:
        ser.close()
        if record_file:
            record_file.close()


if __name__ == "__main__":
    main()

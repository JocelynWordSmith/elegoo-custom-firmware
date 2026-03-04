#!/usr/bin/env python3
"""Replay and validate serial fixture recordings (no hardware needed).

Reads JSONL fixture files and validates that every response matches
the expected schema for the arduino-bare firmware protocol.

Usage:
    python3 scripts/replay_serial.py test-fixtures/recordings/*.jsonl
"""

import json
import sys
import re
import os


def extract_n(raw):
    """Extract N value from a tx command string."""
    try:
        data = json.loads(raw)
        return data.get("N")
    except (json.JSONDecodeError, TypeError):
        return None


def validate_response(n, resp):
    """Validate a response dict against expected schema for command N.
    Returns (ok: bool, reason: str)."""
    if resp is None:
        return False, "response is not valid JSON"

    validators = {
        1:   lambda r: (r.get("ok") is True, "expected ok:true"),
        2:   lambda r: (r.get("cmd") == "fwd", f"expected cmd:fwd, got {r.get('cmd')}"),
        3:   lambda r: (r.get("cmd") == "bwd", f"expected cmd:bwd, got {r.get('cmd')}"),
        4:   lambda r: (r.get("cmd") == "left", f"expected cmd:left, got {r.get('cmd')}"),
        5:   lambda r: (r.get("cmd") == "right", f"expected cmd:right, got {r.get('cmd')}"),
        6:   lambda r: (r.get("cmd") == "stop", f"expected cmd:stop, got {r.get('cmd')}"),
        8:   lambda r: (isinstance(r.get("s"), int) and 0 <= r["s"] <= 255,
                        f"expected s:int 0-255, got {r.get('s')}"),
        10:  lambda r: (isinstance(r.get("d"), int) and r["d"] >= -1,
                        f"expected d:int >= -1, got {r.get('d')}"),
        13:  lambda r: ("b" in r and "raw" in r and isinstance(r["raw"], int),
                        f"expected b+raw keys, got {list(r.keys())}"),
        14:  lambda r: ("br" in r or "err" in r,
                        f"expected br or err key, got {list(r.keys())}"),
        15:  lambda r: (isinstance(r.get("mb"), list) and len(r["mb"]) == 2,
                        f"expected mb:[l,r], got {r.get('mb')}"),
        20:  lambda r: (isinstance(r.get("l"), list) and len(r["l"]) == 3,
                        f"expected l:[r,g,b], got {r.get('l')}"),
        21:  lambda r: (isinstance(r.get("B"), int) and 0 <= r["B"] <= 255,
                        f"expected B:int 0-255, got {r.get('B')}"),
        100: lambda r: (all(k in r for k in ["t", "u", "d", "b"]),
                        f"expected t,u,d,b keys, got {list(r.keys())}"),
        101: lambda r: (all(k in r for k in ["t", "M", "T", "l", "B", "s", "w", "mb", "br", "ma", "st"]),
                        f"missing state keys, got {list(r.keys())}"),
        102: lambda r: (isinstance(r.get("w"), int) and r["w"] >= 0,
                        f"expected w:int >= 0, got {r.get('w')}"),
        103: lambda r: (isinstance(r.get("st"), int) and r["st"] >= 0,
                        f"expected st:int >= 0, got {r.get('st')}"),
        104: lambda r: (isinstance(r.get("ma"), int) and r["ma"] > 0,
                        f"expected ma:int > 0, got {r.get('ma')}"),
        105: lambda r: ("fv" in r and isinstance(r["fv"], str) and re.match(r"\d+\.\d+\.\d+", r["fv"]),
                        f"expected fv:semver, got {r.get('fv')}"),
    }

    if n in validators:
        return validators[n](resp)

    # Unknown command — expect error response
    if "err" in resp:
        return True, ""
    return False, f"expected err key for unknown N={n}, got {list(resp.keys())}"


def validate_no_n(resp):
    """Validate response when N field is missing."""
    if resp and "err" in resp:
        return True, ""
    return False, f"expected err for missing N, got {resp}"


def replay_fixture(fixture_path):
    """Replay a single fixture file and return results."""
    basename = os.path.basename(fixture_path)
    pairs = []
    current_tx = None

    with open(fixture_path) as f:
        for line_no, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            try:
                entry = json.loads(line)
            except json.JSONDecodeError:
                pairs.append({"line": line_no, "error": "invalid JSONL"})
                continue

            if entry["dir"] == "tx":
                current_tx = entry
            elif entry["dir"] == "rx" and current_tx is not None:
                pairs.append({
                    "line": line_no,
                    "tx": current_tx["raw"],
                    "rx": entry["raw"],
                })
                current_tx = None

    total = len(pairs)
    passed = 0
    errors = []

    for pair in pairs:
        if "error" in pair:
            errors.append({"line": pair["line"], "reason": pair["error"]})
            continue

        tx_raw = pair["tx"]
        rx_raw = pair["rx"]

        try:
            resp = json.loads(rx_raw)
        except json.JSONDecodeError:
            errors.append({"line": pair["line"], "cmd": tx_raw, "reason": "rx not valid JSON"})
            continue

        n = extract_n(tx_raw)
        if n is None:
            ok, reason = validate_no_n(resp)
        else:
            ok, reason = validate_response(n, resp)

        if ok:
            passed += 1
        else:
            errors.append({"line": pair["line"], "cmd": f"N={n}", "reason": reason})

    return {
        "fixture": basename,
        "total": total,
        "pass": passed,
        "fail": total - passed,
        "errors": errors,
    }


def main():
    if len(sys.argv) < 2:
        print("Usage: replay_serial.py <fixture.jsonl> [fixture2.jsonl ...]", file=sys.stderr)
        sys.exit(1)

    all_pass = True
    for path in sys.argv[1:]:
        result = replay_fixture(path)
        print(json.dumps(result))
        if result["fail"] > 0:
            all_pass = False

    sys.exit(0 if all_pass else 1)


if __name__ == "__main__":
    main()

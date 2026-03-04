# Test Fixtures

Recorded serial command/response pairs for offline replay testing (Tier 2).

## Format

JSONL (one JSON object per line):

```json
{"t": 1709500000.0, "dir": "tx", "raw": "{\"N\":1}"}
{"t": 1709500000.1, "dir": "rx", "raw": "{\"ok\":true}"}
```

- `t` — Unix timestamp (float)
- `dir` — `"tx"` (command sent to Arduino) or `"rx"` (response received)
- `raw` — the raw JSON string sent/received

## Naming Convention

`<date>_<description>.jsonl` — e.g. `2026-03-03_baseline.jsonl`

The `seed_baseline.jsonl` file is a hand-written golden fixture for smoke-testing the replay validator.

## Recording

```bash
make record                                          # auto-detect port
python3 scripts/record_serial.py --port /dev/cu.usbserial-14130  # explicit port
```

## Replaying

```bash
make test-replay                                     # all fixtures
python3 scripts/replay_serial.py test-fixtures/recordings/seed_baseline.jsonl
```

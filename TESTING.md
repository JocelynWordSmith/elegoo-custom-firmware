# Testing

Three-tier test harness for the elegoo-custom-firmware project.

## Quick Start

```bash
make check    # runs all tiers; skips hardware if not connected
```

## Tiers

| Tier | What | Hardware? | Command |
|------|------|-----------|---------|
| T1 | Unit tests (pure logic) | No | `make test-unit` |
| T2 | Serial protocol replay | No | `make test-replay` |
| T3 | Live hardware integration | Yes (USB) | `make test-hw` |

### Tier 1 — Unit Tests

Tests pure functions in `robot_logic.h`: JSON parsing, motor ramping/clamping, battery ADC conversion, sonar distance, median filter.

```bash
cd firmware/arduino-bare && pio test -e native
```

56 tests, runs in seconds, no hardware needed.

### Tier 2 — Serial Protocol Replay

Validates recorded serial tx/rx pairs against the expected response schema for each command. No hardware needed.

```bash
# Replay all fixtures
make test-replay

# Replay a specific fixture
python3 scripts/replay_serial.py test-fixtures/recordings/seed_baseline.jsonl
```

**Recording a new fixture** (requires Arduino connected via USB):

```bash
make record
# or: python3 scripts/record_serial.py --port /dev/cu.usbserial-14130
```

Fixtures are stored in `test-fixtures/recordings/` as JSONL files. See [test-fixtures/README.md](test-fixtures/README.md) for format details.

### Tier 3 — Hardware Integration

Runs safe (non-motor) commands against a live Arduino. Tests sensor reads, config get/set, error handling, LED control. Restores all changed settings after each test.

```bash
make test-hw                                    # auto-detect port
python3 scripts/test_hardware.py --port /dev/cu.usbserial-14130
python3 scripts/test_hardware.py --port /dev/cu.usbserial-14130 --record fixture.jsonl
```

Exits cleanly (exit 0, "skipped") when no hardware is connected.

## Environment Variables

| Variable | Purpose | Default |
|----------|---------|---------|
| `ELEGOO_PORT` | Serial port for Arduino | Auto-detect `/dev/cu.usbserial*` |

## How to Add Tests

- **T1**: Add test functions to files in `firmware/arduino-bare/test/` and register them with `RUN_TEST()` in `main()`.
- **T2**: Record a new fixture with `make record`, or hand-write JSONL following the format in `test-fixtures/README.md`.
- **T3**: Add a `self.check()` call in `scripts/test_hardware.py` `run_all()` method. Only use commands that are safe to run while tethered by USB (no motor movement).

## LLM Agent Guidance

1. **Start with `make check`** — get the full picture cheaply.
2. If a T1 test fails, fix it before touching hardware.
3. If the bug only shows on hardware, record a session (`make record`), then write a replay test that fails before fixing the bug.
4. Use T3 only to confirm a fix works on the live system.

**Token budget:**
- T1: always run; ~50 tokens of output per run
- T2: run when a specific scenario needs regression coverage; ~200 tokens per fixture
- T3: run sparingly; structure commands to get maximum signal per round-trip

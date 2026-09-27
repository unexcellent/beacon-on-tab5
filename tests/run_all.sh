#!/usr/bin/env bash
#
# run_all.sh — run the whole hardware-in-the-loop suite (tests/) with pytest
# and print a per-test pass/fail/skip line for each one.
#
# Unlike scripts/run_hil_tests.sh (which runs each file standalone), this shares
# one board across the session via the `board` fixture in tests/conftest.py, so
# the ESP is opened once. The SSTV image test records this laptop's microphone,
# so run in a quiet room with the mic close to the Tab5's speaker.
#
# Usage: tests/run_all.sh [PYTEST_ARGS...]
#   PROFILE=release        use the release build for the OTA image (default: debug)
#   tests/run_all.sh -k update      only tests whose name matches "update"
set -uo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."

PROFILE="${PROFILE:-debug}"
ELF="target/riscv32imafc-esp-espidf/$PROFILE/beacon-on-tab5"
if [ ! -f "$ELF" ]; then
    echo "no $ELF — run 'cargo build' first" >&2
    exit 1
fi

# The OTA tests push this firmware's own app image over the link.
espflash save-image --chip esp32p4 -s 16mb "$ELF" /tmp/beacon_ota.bin
export BEACON_OTA_IMAGE=/tmp/beacon_ota.bin

# The Python deps live in this repo's .venv (create it with:
#   python3 -m venv .venv && .venv/bin/pip install -r tests/requirements.txt).
# Fall back to whatever python3 is on PATH.
PY=".venv/bin/python"
[ -x "$PY" ] || PY=python3

exec "$PY" -m pytest tests/ -v -ra "$@"

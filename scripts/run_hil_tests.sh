#!/usr/bin/env bash
#
# run_hil_tests.sh — run this repo's hardware-in-the-loop test suite (tests/)
# against a Tab5 running this firmware, connected via USB-C.
#
# The tests drive the payload link (CSP/KISS over the Tab5's USB-Serial-JTAG
# port). The SSTV image test records this laptop's microphone, so run it in a
# quiet room with the microphone close to the Tab5's speaker.
#
# Usage: scripts/run_hil_tests.sh [FILTER]
#   PROFILE=release   use the release build for the OTA image (default: debug)
#   FILTER            only run test files whose name contains FILTER
set -uo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."
REPO="$PWD"

PROFILE="${PROFILE:-debug}"
ELF="target/riscv32imafc-esp-espidf/$PROFILE/beacon-on-tab5"
if [ ! -f "$ELF" ]; then
    echo "no $ELF — run 'cargo build' first" >&2
    exit 1
fi

# The OTA test pushes this firmware's own app image over the link.
espflash save-image --chip esp32p4 -s 16mb "$ELF" /tmp/beacon_ota.bin
export BEACON_OTA_IMAGE=/tmp/beacon_ota.bin

# The Python deps (pyserial, sstv, Pillow, numpy, sounddevice, scipy) live in the
# beacon repo's venv; fall back to whatever python3 is on PATH.
PY="../beacon/.venv/bin/python"
[ -x "$PY" ] || PY=python3

filter="${1:-}"
pass=0; fail=0; skip=0; failed=()
for f in tests/test_*.py; do
    name="$(basename "$f" .py)"
    if [ -n "$filter" ]; then case "$name" in *"$filter"*) ;; *) continue ;; esac; fi
    echo "… $name"
    start=$SECONDS
    "$PY" -u "$f"
    rc=$?
    dur=$((SECONDS-start))
    case "$rc" in
        0)  echo "✔ $name (${dur}s)"; pass=$((pass+1)) ;;
        77) echo "○ $name (skipped, ${dur}s)"; skip=$((skip+1)) ;;
        *)  echo "✖ $name (${dur}s)"; fail=$((fail+1)); failed+=("$name") ;;
    esac
done

echo
echo "Summary: $pass passed, $fail failed, $skip skipped"
[ "$fail" -eq 0 ] || printf 'failed: %s\n' "${failed[@]}"
[ "$fail" -eq 0 ]

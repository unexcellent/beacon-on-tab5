#!/usr/bin/env bash
#
# Run the beacon test suite on the host and report a clear pass/fail summary.
#
# The firmware crates default to the ESP32-P4 RISC-V target, which can't run
# tests natively. The trait-driven logic lives in the `beacon` library and is
# written to compile for the host (esp-idf deps are gated on the espidf
# target), so we run its `--lib` tests against the host triple.
set -euo pipefail

host=$(rustc -vV | sed -n 's/^host: //p')
beacon_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../beacon" && pwd)"

echo "Running beacon tests (target: $host)"
echo

if cargo test --manifest-path "$beacon_dir/Cargo.toml" --target "$host" --lib "$@"; then
    echo
    echo "✅ All tests passed"
else
    status=$?
    echo
    echo "❌ Tests failed (exit $status)"
    exit "$status"
fi

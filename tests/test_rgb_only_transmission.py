#!/usr/bin/env python3
"""HIL test: the Tab5's camera image survives the SSTV round-trip over the air.

The Tab5 encodes its SC202CS camera frame as Robot36 audio and plays it through
its speaker; we record it with this laptop's microphone, decode it, and check it
looks like a real picture.

Requires:
  - A Tab5 running this firmware, connected over USB-C (the payload link).
  - The `sstv`, Pillow, numpy and sounddevice Python packages (scipy optional,
    for the band-pass pre-filter).
  - The microphone close to the Tab5's speaker (near-field), in a quiet room —
    room reverb, not loudness, is what makes an acoustic SSTV decode fail.

Skips cleanly (exit 77) when the link or microphone capture isn't available.

Run:  ../beacon/.venv/bin/python tests/test_rgb_only_transmission.py
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from tests.util.ota_hil import run_case
from tests.util.payload_board import MockPayloadBoard
from tests.util.sstv_capture import (
    assert_valid_image,
    capture_and_decode,
    send_sstv_command,
)

CAMERA_HINT = "Is the Tab5 camera working, and is the microphone close to its speaker?"


def test_rgb_only_transmission(board: MockPayloadBoard) -> None:
    send_sstv_command(board)
    image = capture_and_decode(board, camera_hint=CAMERA_HINT)
    assert_valid_image(image, min_std=6.0, min_smoothness=0.35)


if __name__ == "__main__":
    sys.exit(run_case(test_rgb_only_transmission))

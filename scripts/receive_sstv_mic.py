#!/usr/bin/env python3
"""Trigger an SSTV transmission on the Tab5 and decode it from this laptop's
microphone (Robot36, via the `sstv` PyPI package).

Sends the SSTV command over the USB-C payload link, records the microphone
while the Tab5 plays the ~36 s Robot36 audio through its speaker, decodes the
image and writes it next to the recording.

The acoustic decode quality is set by the microphone's position: SSTV is FM, so
loudness barely matters, but room reverb smears the tones. Put the mic (or a
phone) within a few centimetres of the Tab5's speaker grille, in a quiet room —
near-field capture is the single biggest factor. For a guaranteed-perfect image
that bypasses the speaker/mic entirely, use the digital path (see
scripts/decode_usb_dump.py and the `usb-audio-dump` feature).

Usage:
    ../beacon/.venv/bin/python scripts/receive_sstv_mic.py [PORT] [-o OUT.png]
    ... --listen-only     just record + decode, don't command the Tab5

Requires the beacon repo's venv (pyserial, sstv, Pillow, numpy) plus
`sounddevice`; `scipy` is optional and enables the band-pass pre-filter.
"""

import argparse
import sys
import time
import wave
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent / "beacon"))

import numpy as np
import sounddevice as sd
import sstv
from tests.util.payload_board import MockPayloadBoard, detect_port

# Capture at 48 kHz (not the ESP's 16 kHz output rate): more samples per SSTV
# pixel gives the decoder finer timing to lock onto through acoustic noise.
CAPTURE_RATE = 48_000
RECORD_SECONDS = 42
AVAILABLE_TIMEOUT = 70.0


def bandpass(samples: np.ndarray, rate: int) -> np.ndarray:
    """Restrict to the SSTV tone band (1.05–2.45 kHz), dropping out-of-band
    room noise and hum that would otherwise confuse sync detection. No-op if
    scipy is unavailable."""
    try:
        from scipy.signal import butter, filtfilt
    except ImportError:
        return samples
    b, a = butter(4, [1050 / (rate / 2), 2450 / (rate / 2)], btype="band")
    return filtfilt(b, a, samples)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Trigger an SSTV transmission on the Tab5 and decode it from the microphone."
    )
    parser.add_argument("port", nargs="?", help="serial device (default: auto-detect)")
    parser.add_argument("-o", "--out", default="sstv_received.png", help="decoded image path")
    parser.add_argument("--wav", default=None, help="keep the raw recording at this path")
    parser.add_argument("--listen-only", action="store_true",
                        help="record + decode without sending the SSTV command")
    args = parser.parse_args()

    board = None
    if not args.listen_only:
        board = MockPayloadBoard(args.port or detect_port())
        board.start()
        time.sleep(0.3)
        if not board.ping():
            print("Tab5 not answering pings on the payload link", file=sys.stderr)
            return 1
        board.send_sstv()
        if board.wait_for_text(b"BUSY", timeout=10.0) is None:
            print("no BUSY after the SSTV command", file=sys.stderr)
            return 1
        print("Tab5 is BUSY — transmitting; recording microphone...")

    recording = sd.rec(RECORD_SECONDS * CAPTURE_RATE, samplerate=CAPTURE_RATE,
                       channels=1, dtype="int16")
    if board is not None:
        if board.wait_for_text(b"AVAILABLE", timeout=AVAILABLE_TIMEOUT) is None:
            print("warning: no AVAILABLE — decoding anyway", file=sys.stderr)
        board.close()
    sd.wait()

    raw = recording[:, 0].astype(float)
    peak = int(np.abs(raw).max())
    print(f"recorded {RECORD_SECONDS}s @ {CAPTURE_RATE} Hz, peak {peak} "
          f"({100 * peak / 32768:.1f}% FS)")
    if peak < 200:
        print("recording is essentially silent — is the speaker on and the mic close?",
              file=sys.stderr)

    if args.wav:
        with wave.open(args.wav, "wb") as w:
            w.setnchannels(1)
            w.setsampwidth(2)
            w.setframerate(CAPTURE_RATE)
            w.writeframes(recording.tobytes())
        print(f"recording saved to {args.wav}")

    filtered = bandpass(raw, CAPTURE_RATE)
    peak_f = np.abs(filtered).max() or 1.0
    samples = np.clip(filtered / peak_f * 20000, -32000, 32000).astype(int)

    images = sstv.decode([int(s) for s in samples], CAPTURE_RATE, mode=sstv.Mode.ROBOT_36)
    if not images:
        print("no SSTV image could be decoded", file=sys.stderr)
        return 1
    images[0].save(args.out)
    print(f"decoded {images[0].size[0]}x{images[0].size[1]} image -> {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

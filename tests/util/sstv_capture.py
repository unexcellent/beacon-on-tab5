#!/usr/bin/env python3
"""Building blocks for the SSTV image-capture HIL test on the Tab5.

The test sends the SSTV command, records the Tab5's speaker through this laptop's
microphone while it transmits, decodes the Robot36 image and checks it is a real
picture. Unlike the MOVE-IIIa bench rig (which taps the ESP's I2S digitally over
a Pi), the Tab5 has an actual speaker, so the capture is acoustic.

Acoustic decode quality is set by microphone placement: SSTV is FM, so room
reverb — not loudness — is what wrecks the image. Put the microphone (or a phone)
within a few centimetres of the Tab5's speaker grille, in a quiet room.
"""

import logging

from tests.util.ota_hil import drain, skip
from tests.util.payload_board import NODE_PAYLOAD, MockPayloadBoard

log = logging.getLogger("sstv_capture")

# Capture at 48 kHz (not the Tab5's 16 kHz output rate): more samples per SSTV
# pixel give the decoder finer timing to lock onto through acoustic noise.
CAPTURE_RATE = 48000
RECORD_SECONDS = 42       # Robot36 is ~36 s; record a bit longer
AVAILABLE_TIMEOUT = 60.0  # SSTV takes ~36 s before AVAILABLE
ROBOT36 = (320, 240)


def send_sstv_command(board: MockPayloadBoard) -> None:
    """Send the SSTV command and confirm the Tab5 acknowledged it with BUSY."""
    drain(board)
    board.send_sstv()
    busy = board.wait_for_text(b"BUSY", timeout=5.0)
    assert busy is not None and busy.dst == NODE_PAYLOAD, "no BUSY after the SSTV command"
    log.info("SSTV command sent; Tab5 is BUSY, transmitting (~36s) ...")


def wait_for_available(board: MockPayloadBoard) -> None:
    """Wait for the Tab5's AVAILABLE status, i.e. transmit_sstv finished."""
    available = board.wait_for_text(b"AVAILABLE", timeout=AVAILABLE_TIMEOUT)
    assert available is not None and available.dst == NODE_PAYLOAD, (
        f"no AVAILABLE within {AVAILABLE_TIMEOUT:.0f}s of BUSY — transmit_sstv never finished"
    )


def capture_and_decode(board: MockPayloadBoard, *, camera_hint: str):
    """Record the Tab5's speaker through the microphone, then decode the image.

    Waits for AVAILABLE (transmission done), checks the recording isn't silent,
    band-passes it to the SSTV tone band and decodes. `camera_hint` is shown if
    SSTV never ends.
    """
    import sstv

    samples, rate = _record_mic(board, camera_hint)

    peak = max((abs(s) for s in samples), default=0)
    log.info("recorded peak=%d (%.1f%% FS)", peak, 100 * peak / 32768)
    assert peak > 200, (
        f"capture is silent (peak={peak}). The link says SSTV ran but no audio "
        "arrived — is the Tab5's speaker on and the microphone close to it?"
    )

    images = sstv.decode(samples, rate, mode=sstv.Mode.ROBOT_36)
    assert images, "no SSTV image could be decoded from the recording"
    return images[0]


def _record_mic(board: MockPayloadBoard, camera_hint: str) -> tuple[list[int], int]:
    """Record the host microphone through the transmission, band-passed."""
    try:
        import numpy as np
        import sounddevice as sd
    except ImportError:
        skip("sounddevice/numpy not installed — microphone capture not set up")

    log.info("recording microphone for %ds @ %d Hz", RECORD_SECONDS, CAPTURE_RATE)
    recording = sd.rec(RECORD_SECONDS * CAPTURE_RATE, samplerate=CAPTURE_RATE,
                       channels=1, dtype="int16")
    available = board.wait_for_text(b"AVAILABLE", timeout=AVAILABLE_TIMEOUT)
    assert available is not None, f"no AVAILABLE — SSTV never finished. {camera_hint}"
    log.info("AVAILABLE received; finishing recording")
    sd.wait()

    raw = recording[:, 0].astype(float)
    filtered = _bandpass(raw, CAPTURE_RATE)
    peak = np.abs(filtered).max() or 1.0
    samples = np.clip(filtered / peak * 20000, -32000, 32000).astype(int)
    return [int(s) for s in samples], CAPTURE_RATE


def _bandpass(samples, rate: int):
    """Restrict to the SSTV tone band (1.05-2.45 kHz), dropping out-of-band room
    noise and hum that would otherwise confuse sync detection. No-op without scipy."""
    try:
        from scipy.signal import butter, filtfilt
    except ImportError:
        return samples
    b, a = butter(4, [1050 / (rate / 2), 2450 / (rate / 2)], btype="band")
    return filtfilt(b, a, samples)


def assert_valid_image(image, *, min_std: float, min_smoothness: float) -> None:
    """Check the decoded image is a real picture: right size, has content, not noise."""
    from PIL import ImageStat

    (w, h), std, smooth = _image_stats(image, ImageStat)
    log.info("decoded %dx%d std=(%.0f,%.0f,%.0f) smoothness=%.2f", w, h, *std, smooth)
    assert (w, h) == ROBOT36, f"decoded {w}x{h}, expected {ROBOT36}"
    assert max(std) > min_std, f"image is flat/blank (per-channel std {std}) — no real picture"
    assert smooth > min_smoothness, (
        f"image looks like noise (smoothness {smooth:.2f}) — move the microphone closer "
        "to the Tab5 speaker (near-field), in a quiet room"
    )


def _image_stats(image, ImageStat) -> tuple[tuple[int, int], list[float], float]:
    """(size, per-channel stddev, horizontal-neighbour similarity fraction)."""
    image = image.convert("RGB")
    w, h = image.size
    std = ImageStat.Stat(image).stddev
    px = image.load()
    similar = pairs = 0
    for y in range(h):
        prev = px[0, y]
        for x in range(1, w):
            cur = px[x, y]
            for c in range(3):
                if abs(cur[c] - prev[c]) <= 24:
                    similar += 1
                pairs += 1
            prev = cur
    smoothness = similar / pairs if pairs else 0.0
    return (w, h), std, smoothness

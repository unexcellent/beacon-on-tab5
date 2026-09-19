#!/usr/bin/env python3
"""Trigger an SSTV transmission and decode the EXACT digital samples the firmware
produced, captured over USB — bypassing the speaker, microphone and room.

This needs firmware built with the `usb-audio-dump` feature:

    NO_MONITOR=1 cargo run --features usb-audio-dump   # flash it

With that feature, after each transmission the firmware base64-dumps its PCM
sample stream over the USB console (offset-tagged, so a dropped line becomes a
tiny local gap instead of desyncing). This script sends the SSTV command, joins
the dump, and decodes it. The result is the reference "what the firmware would
sound like with a perfect analog path" — use it to prove the pipeline end to end
independently of acoustics.

Usage:
    ../beacon/.venv/bin/python scripts/decode_usb_dump.py [PORT] [-o OUT.png]
"""

import argparse
import base64
import re
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent / "beacon"))

import numpy as np
import sstv
from tests.util.payload_board import (
    EPHEMERAL_PORT, NODE_PAYLOAD, NODE_SSTV, PORT_CMD, PRIO_NORM,
    detect_port, kiss_encode, pack_header,
)

HEADER = re.compile(rb"<<<SSTV-PCM len=(\d+) rate=(\d+)>>>")
CHUNK = re.compile(rb"PCMB64:(\d+):([A-Za-z0-9+/=]+)")


def main() -> int:
    parser = argparse.ArgumentParser(description="Decode the firmware's USB PCM dump into an SSTV image.")
    parser.add_argument("port", nargs="?", help="serial device (default: auto-detect)")
    parser.add_argument("-o", "--out", default="sstv_digital.png", help="decoded image path")
    args = parser.parse_args()

    import serial

    port = args.port or detect_port()
    s = serial.Serial(port, 115200, timeout=0.1)
    time.sleep(0.3)
    s.reset_input_buffer()
    s.write(kiss_encode(
        pack_header(PRIO_NORM, NODE_PAYLOAD, NODE_SSTV, PORT_CMD, EPHEMERAL_PORT, 0) + b"SSTV"
    ))
    print("SSTV command sent; capturing PCM dump over USB...")

    buf = bytearray()
    deadline = time.time() + 90
    while time.time() < deadline:
        buf += s.read(65536)
        if b"SSTV-PCM-END" in buf:
            break
    s.close()

    header = HEADER.search(buf)
    if not header:
        print("no PCM dump seen — is the firmware built with --features usb-audio-dump?",
              file=sys.stderr)
        return 1
    total, rate = int(header.group(1)), int(header.group(2))

    raw = bytearray(total)
    seen = bytearray(total)
    dropped = 0
    for off, data in CHUNK.findall(buf):
        try:
            decoded = base64.b64decode(data, validate=True)
        except Exception:
            dropped += 1
            continue
        o = int(off)
        raw[o:o + len(decoded)] = decoded
        for i in range(o, min(o + len(decoded), total)):
            seen[i] = 1
    missing = total - sum(seen)
    print(f"reassembled {total} bytes, {missing} missing ({100 * missing / total:.2f}%), "
          f"{dropped} corrupt lines")

    samples = np.frombuffer(bytes(raw), dtype="<i2")
    images = sstv.decode([int(v) for v in samples], rate, mode=sstv.Mode.ROBOT_36)
    if not images:
        print("no SSTV image could be decoded from the dump", file=sys.stderr)
        return 1
    images[0].save(args.out)
    print(f"decoded {images[0].size[0]}x{images[0].size[1]} image -> {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

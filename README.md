# beacon-on-tab5

The [MOVE-III beacon firmware](../beacon) running on an M5Stack Tab5
(ESP32-P4 development kit), so the full mission loop can be exercised on a
desk with no carrier board: the Tab5's USB-C port is the payload link, its
built-in camera is the RGB camera, and its speaker is the SSTV downlink.

All mission logic (idle loop, Robot36 SSTV transmission, CSP/KISS link
protocol, OTA firmware update) is the `beacon` crate, imported unchanged.
This repo only adds the Tab5 bring-up:

| beacon role     | MOVE-IIIa carrier          | Tab5                                      |
| --------------- | -------------------------- | ----------------------------------------- |
| payload link    | RS422 (UART1)              | USB-Serial-JTAG on the USB-C port         |
| RGB camera      | SC850SL, 2-lane CSI, RAW10 | SC202CS ("SC2356"), 1-lane CSI, RAW10     |
| audio out       | PCM5102A DAC over I2S      | ES8388 codec over I2S + NS4150 speaker amp |
| board bring-up  | —                          | PI4IOE5V6408 IO expanders (speaker enable) |

## Build and flash

Uses `../beacon`'s ESP-IDF install via the `.embuild` symlink (create it with
`ln -s ../beacon/.embuild .embuild` on a fresh checkout).

```sh
cargo build            # or --release
cargo run              # flash + monitor over USB-C
NO_MONITOR=1 cargo run # flash only (leave the port free for the link)
```

## Try it

With the firmware running, ask for an SSTV transmission from the laptop:

```sh
cd ../beacon
./.venv/bin/python tests/util/payload_board.py /dev/cu.usbmodem* --interval 60
```

The Tab5 answers BUSY, plays ~36 s of Robot36 through its speaker, then
AVAILABLE. Any SSTV decoder (or the test suite below) can decode the audio.

## Image quality: acoustic vs. digital

SSTV is FM audio, so the decoded image quality is set almost entirely by the
audio channel, not the firmware. Two ways to receive:

- **Digital (perfect).** Build with `--features usb-audio-dump` and run
  `scripts/decode_usb_dump.py`. The firmware base64-dumps the exact PCM sample
  stream it generated over USB, bypassing the speaker, microphone and room. This
  decodes to a clean, sharp image every time — use it to verify the pipeline end
  to end (camera → encode → synth) independently of acoustics.

- **Acoustic (speaker → microphone).** `scripts/receive_sstv_mic.py`. Quality
  here depends on the microphone's placement: room reverb smears the tones and
  wrecks the decode, while loudness barely matters (FM). **Put the mic — or a
  phone running an SSTV app — within a few centimetres of the speaker grille, in
  a quiet room.** At desk distance in a normal room the image comes out heavily
  striped with separated colours; in the speaker's near field it cleans up. The
  receive script captures at 48 kHz and band-passes to the SSTV tone band, which
  is the most a receiver can do — the rest is physical placement.

If a decode looks striped, first confirm the firmware is fine with the digital
path; a clean digital image plus a striped acoustic one means the fault is the
acoustic setup, not the beacon.

## Hardware-in-the-loop tests

The `tests/` directory holds this repo's own HIL suite, adapted from the beacon
crate's tests for the Tab5: the payload link runs over USB-C (not RS422), the
SSTV image test records the laptop microphone (not a Pi's I2S tap), and the OTA
test builds and pushes this firmware. There is no thermal-camera test — the Tab5
has no thermal camera. The Python deps live in the beacon repo's venv
(`../beacon/.venv`).

```sh
cargo build                 # the OTA test pushes this image over the link
scripts/run_hil_tests.sh    # optionally: run_hil_tests.sh <filter>
```

Or run one standalone:

```sh
../beacon/.venv/bin/python tests/test_rgb_only_transmission.py
```

Console logs and the KISS link share the USB-C port; the test-side KISS decoder
skips log bytes between frames. For the image test, keep the microphone close to
the Tab5's speaker (see the image-quality section above).

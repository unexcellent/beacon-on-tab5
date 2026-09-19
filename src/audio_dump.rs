//! Diagnostic tee (feature `usb-audio-dump`): wraps the real audio channel and,
//! at the end of each transmission, dumps the exact PCM sample stream over the
//! USB-Serial-JTAG port so the digital signal can be decoded and compared with
//! the microphone recording of the same transmission, isolating whether a
//! quality problem is in the sample stream or in the analog playback path.
//!
//! The dump rides the normal console/log path (base64, chunked) so it uses the
//! same, known-working transport as every other log line. On the host, join all
//! `PCMB64:` lines between `<<<SSTV-PCM ...>>>` and `<<<SSTV-PCM-END>>>` and
//! base64-decode into i16 LE mono samples.

use beacon::audio::{AudioChannel, AudioError};

const B64: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

pub struct TeeDump<C: AudioChannel> {
    inner: C,
    samples: Vec<i16>,
}

impl<C: AudioChannel> TeeDump<C> {
    pub fn new(inner: C) -> Self {
        log::warn!("TeeDump audio wrapper active (usb-audio-dump)");
        Self {
            inner,
            samples: Vec::with_capacity(700_000),
        }
    }

    fn dump(&mut self) {
        let bytes: Vec<u8> = self.samples.iter().flat_map(|s| s.to_le_bytes()).collect();
        log::info!(
            "<<<SSTV-PCM len={} rate={}>>>",
            bytes.len(),
            self.inner.sample_rate()
        );
        // Each line carries its byte offset so the host can place it exactly;
        // a line dropped under USB burst pressure then becomes a small local
        // gap instead of desyncing everything after it. 384-byte chunks keep
        // each line short enough to survive the console path intact.
        for (i, chunk) in bytes.chunks(384).enumerate() {
            log::info!("PCMB64:{}:{}", i * 384, base64(chunk));
        }
        log::info!("<<<SSTV-PCM-END>>>");
        self.samples.clear();
    }
}

fn base64(data: &[u8]) -> String {
    let mut out = String::with_capacity(data.len().div_ceil(3) * 4);
    for group in data.chunks(3) {
        let n = group.len();
        let a = group[0];
        let b = if n > 1 { group[1] } else { 0 };
        let c = if n > 2 { group[2] } else { 0 };
        let v = (a as u32) << 16 | (b as u32) << 8 | c as u32;
        out.push(B64[(v >> 18 & 63) as usize] as char);
        out.push(B64[(v >> 12 & 63) as usize] as char);
        out.push(if n > 1 {
            B64[(v >> 6 & 63) as usize] as char
        } else {
            '='
        });
        out.push(if n > 2 {
            B64[(v & 63) as usize] as char
        } else {
            '='
        });
    }
    out
}

impl<C: AudioChannel> AudioChannel for TeeDump<C> {
    fn sample_rate(&self) -> u32 {
        self.inner.sample_rate()
    }

    fn transmit(&mut self, sample: i16) -> core::result::Result<(), AudioError> {
        self.samples.push(sample);
        self.inner.transmit(sample)
    }

    fn flush(&mut self) -> core::result::Result<(), AudioError> {
        self.dump();
        self.inner.flush()
    }
}

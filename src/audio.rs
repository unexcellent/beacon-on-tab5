//! Tab5 audio output: the ES8388 codec (I2S slave) feeding the NS4150 speaker
//! amplifier. Registers are configured once over the shared I2C bus at boot;
//! afterwards the codec just consumes the I2S stream.
//!
//! Register sequence follows Espressif's `esp_codec_dev` ES8388 driver in DAC
//! mode, with the speaker's mono use in mind: the sample is duplicated onto
//! both I2S slots so it reaches the amplifier regardless of which DAC output
//! the board routes to it.

use beacon::audio::{
    AudioChannel, AudioEncoder, AudioError, AudioInterface, I2sConfig, I2sInterface,
};
use beacon::error::{Error, Result};
use embedded_hal::i2c::I2c;

/// 7-bit I2C address of the ES8388 (CE strapped low).
const ES8388_ADDR: u8 = 0x10;

/// Philips I2S at 16 kHz: MCLK = 256 x fs = 4.096 MHz, BCLK = MCLK / 8 =
/// 512 kHz = 16 kHz x 2 channels x 16 bit. The ES8388 runs as clock slave with
/// its DAC FS ratio at 256.
const TAB5_I2S: I2sConfig = I2sConfig {
    sample_rate: 16_000,
    clock_divider: 8,
    mclk_pin: 30,
    bclk_pin: 27,
    dout_pin: 26,
    ws_pin: 29,
    chunk_size: 512,
};

/// Digital DAC attenuation in 0.5 dB steps (0x00 = 0 dB). -10 dB keeps the
/// speaker well out of clipping/rattle territory while staying loud enough for
/// a laptop microphone across a desk.
const DAC_VOLUME: u8 = 20;

/// Scale applied to every sample before it reaches the DAC. The SSTV synthesiser
/// emits full-scale (±32767) sine tones; leaving a little headroom keeps the
/// whole chain (DAC → NS4150 amp → speaker) out of hard clipping. The decode
/// quality over the air is set by the acoustic path (near-field miking beats
/// loudness), so this is just a sensible-headroom default, not a tuning knob.
const SAMPLE_SCALE_NUM: i32 = 3;
const SAMPLE_SCALE_DEN: i32 = 5;

/// Bring up the ES8388 over I2C and open the I2S transmit channel. With the
/// `usb-audio-dump` feature the channel is wrapped in the diagnostic tee.
pub fn initialize_audio_channel<B: I2c>(i2c: &mut B) -> Result<impl AudioChannel + use<B>> {
    configure_codec(i2c).map_err(|_| Error::AudioInit)?;
    let interface = I2sInterface::new(&Es8388::<I2sInterface>::ENCODER, &TAB5_I2S)?;
    let audio = Es8388::new(interface, TAB5_I2S.sample_rate, TAB5_I2S.chunk_size);
    #[cfg(feature = "usb-audio-dump")]
    let audio = crate::audio_dump::TeeDump::new(audio);
    Ok(audio)
}

/// Write the DAC-mode register sequence (esp_codec_dev's es8388_open +
/// es8388_start, DAC path only) and unmute.
fn configure_codec<B: I2c>(i2c: &mut B) -> core::result::Result<(), B::Error> {
    let mut write = |reg: u8, val: u8| i2c.write(ES8388_ADDR, &[reg, val]);

    write(0x19, 0x04)?; // DACCONTROL3: DAC muted during setup
    write(0x01, 0x50)?; // CONTROL2: analog power reference
    write(0x02, 0x00)?; // CHIPPOWER: everything up
    write(0x35, 0xA0)?; // internal DLL bypass (low-rate stability)
    write(0x37, 0xD0)?;
    write(0x39, 0xD0)?;
    write(0x08, 0x00)?; // MASTERMODE: I2S slave
    write(0x04, 0xC0)?; // DACPOWER: outputs off while configuring
    write(0x00, 0x12)?; // CONTROL1
    write(0x17, 0x18)?; // DACCONTROL1: 16-bit, I2S (Philips) format
    write(0x18, 0x02)?; // DACCONTROL2: MCLK/LRCK ratio 256
    write(0x26, 0x00)?; // mixer source select
    write(0x27, 0x90)?; // left DAC -> left mixer, 0 dB
    write(0x2a, 0x90)?; // right DAC -> right mixer, 0 dB
    write(0x2b, 0x80)?; // DAC/ADC share LRCK
    write(0x2d, 0x00)?; // vroi
    write(0x1a, DAC_VOLUME)?; // left DAC digital volume
    write(0x1b, DAC_VOLUME)?; // right DAC digital volume
    write(0x2e, 0x1E)?; // LOUT1 0 dB
    write(0x2f, 0x1E)?; // ROUT1 0 dB
    write(0x30, 0x1E)?; // LOUT2 0 dB
    write(0x31, 0x1E)?; // ROUT2 0 dB

    write(0x02, 0xF0)?; // restart the state machine...
    write(0x02, 0x00)?; // ...and run
    write(0x04, 0x3C)?; // DACPOWER: DAC up, LOUT/ROUT 1+2 enabled
    write(0x19, 0x00) // unmute (ramped)
}

/// The ES8388 as a mono [`AudioChannel`]: packs each sample into both slots of
/// the 16-bit stereo Philips frame and batches DMA-sized writes.
pub struct Es8388<I: AudioInterface> {
    interface: I,
    sample_rate: u32,
    buf: Vec<u8>,
    buf_pos: usize,
}

impl<I: AudioInterface> Es8388<I> {
    /// Serial format: Philips I2S, 16-bit, stereo, MSB-first.
    pub const ENCODER: AudioEncoder = AudioEncoder {
        width_16bit: true,
        stereo: true,
        bit_shift: true,
        left_channel: false,
        big_endian: false,
        least_significant_bit_first: false,
        left_align_data: false,
    };

    pub fn new(interface: I, sample_rate: u32, chunk_size: usize) -> Self {
        Self {
            interface,
            sample_rate,
            buf: vec![0u8; chunk_size * 4],
            buf_pos: 0,
        }
    }
}

impl<I: AudioInterface> Es8388<I> {
    /// Push the buffered samples to the interface without any end-of-stream
    /// handling (the mid-stream path; the trait's `flush` seals the stream).
    fn write_out(&mut self) -> core::result::Result<(), AudioError> {
        self.interface.write(&self.buf[..self.buf_pos])?;
        self.buf_pos = 0;
        Ok(())
    }
}

impl<I: AudioInterface> AudioChannel for Es8388<I> {
    fn sample_rate(&self) -> u32 {
        self.sample_rate
    }

    fn transmit(&mut self, sample: i16) -> core::result::Result<(), AudioError> {
        let sample = (sample as i32 * SAMPLE_SCALE_NUM / SAMPLE_SCALE_DEN) as i16;
        let [lo, hi] = sample.to_le_bytes();
        self.buf[self.buf_pos] = lo;
        self.buf[self.buf_pos + 1] = hi;
        self.buf[self.buf_pos + 2] = lo;
        self.buf[self.buf_pos + 3] = hi;
        self.buf_pos += 4;

        if self.buf_pos == self.buf.len() {
            self.write_out()?;
        }
        Ok(())
    }

    /// Ends a transmission: pushes the pending samples, then enough silence to
    /// overwrite every DMA descriptor. Without this the I2S hardware loops its
    /// last buffer forever and the speaker keeps whistling the final tone.
    fn flush(&mut self) -> core::result::Result<(), AudioError> {
        self.write_out()?;
        // 6 descriptors x 240 frames in the I2S channel; 2048 frames of silence
        // (~128 ms) covers them all with margin.
        let silence = vec![0u8; 2048 * 4];
        self.interface.write(&silence)
    }
}

impl<I: AudioInterface> Drop for Es8388<I> {
    fn drop(&mut self) {
        let _ = self.flush();
    }
}

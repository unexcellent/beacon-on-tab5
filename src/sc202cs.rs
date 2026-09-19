//! The SmartSens SC202CS (marketed SC2356) 2MP Bayer camera on the Tab5, in its
//! 1600x1200 / 30 fps / 1-lane / RAW10 mode: register control on top of the
//! `beacon` crate's CSI transport and shared RAW10 pixel pipeline.

use std::time::Duration;

use beacon::camera::auto_exposure::{
    AutoExposureLimits, AutoExposureState, AutoExposureStep, auto_exposure_step,
};
use beacon::camera::esp::{CsiConfig, CsiInterface, EspI2c};
use beacon::camera::raw10::{build_image, meter};
use beacon::camera::{
    BayerOrder, Camera, CameraInterface, ColorCalibration, FrameFormat, Image, PixelFormat,
};
use beacon::error::{Error, Result};
use embedded_hal::i2c::{ErrorType, I2c};
use sstv::RgbPixel;

/// SSTV output resolution the camera renders into.
const OUTPUT_WIDTH: usize = sstv::Mode::Robot36.image_width() as usize;
const OUTPUT_HEIGHT: usize = sstv::Mode::Robot36.image_height() as usize;

/// Frame length in lines (VTS; the init table keeps the sensor default 1250).
const VTS: u16 = 1250;
/// Bayer mosaic order of this sensor's RAW10 output.
const ORDER: BayerOrder = BayerOrder::Bggr;

/// Frame budget for auto-exposure convergence before capturing anyway.
const AUTO_EXPOSURE_MAX_ITERS: u32 = 6;
/// Frames to capture and discard on activation so the stream stabilises.
const WARMUP_FRAMES: u32 = 2;
/// Bounded so a stalled CSI pipeline degrades the capture instead of wedging
/// the command loop (frames normally arrive every 33 ms).
const FRAME_WAIT: Duration = Duration::from_secs(2);

/// Tab5 CSI link: the SC202CS drives one MIPI lane at 720 Mbps; the LDO channel
/// powering the ESP32-P4 MIPI PHY is chip-internal (same as MOVE-IIIa).
const TAB5_CSI: CsiConfig = CsiConfig {
    data_lane_num: 1,
    lane_bit_rate_mbps: 720,
    ldo_channel: 3,
    ldo_voltage_mv: 2500,
};

/// Bring up the SC202CS on the Tab5's CSI port. Takes over the shared system
/// I2C bus (the sensor has no reset/powerdown pins on this board).
pub fn initialize_rgb_camera(i2c: EspI2c) -> Result<Sc202cs<CsiInterface>> {
    log::info!("RGB camera: initializing SC202CS...");

    let format = Sc202cs::<CsiInterface>::FORMAT;
    let interface = CsiInterface::new(i2c, TAB5_CSI, &format).map_err(|_| Error::RgbInit)?;
    let mut camera = Sc202cs::new(
        interface,
        Sc202cs::<CsiInterface>::DEFAULT_I2C_ADDRESS,
        (OUTPUT_WIDTH, OUTPUT_HEIGHT),
    );
    camera.init().map_err(|_| Error::RgbInit)?;
    Ok(camera)
}

pub struct Sc202cs<I> {
    interface: I,
    address: u8,
    format: FrameFormat,
    order: BayerOrder,
    output: (usize, usize),
    black_level: u8,
    wb_r: f32,
    wb_b: f32,
    /// Current sensor integration time in lines (auto-exposure state).
    exposure: u32,
    /// Current gain as a linear multiplier (1.0 = unity).
    gain: f32,
}

impl<I> Sc202cs<I> {
    /// Strap-default 7-bit I2C address.
    pub const DEFAULT_I2C_ADDRESS: u8 = 0x36;

    /// The raw frame format this sensor produces on its data interface.
    pub const FORMAT: FrameFormat = FrameFormat {
        width: 1600,
        height: 1200,
        pixel: PixelFormat::Raw10(ORDER),
    };

    pub fn new(interface: I, address: u8, output: (usize, usize)) -> Self {
        let calibration = Self::color_calibration();
        Self {
            interface,
            address,
            format: Self::FORMAT,
            order: ORDER,
            output,
            black_level: calibration.black_level,
            wb_r: calibration.red_gain,
            wb_b: calibration.blue_gain,
            exposure: Self::default_exposure(),
            gain: 1.0,
        }
    }

    /// Inclusive (min, max) integration time the sensor accepts, in lines.
    fn exposure_range() -> (u32, u32) {
        (1, (VTS as u32).saturating_sub(8).max(1))
    }

    /// Integration time in lines after [`init`](Self::init)
    /// (0x3e00..02 = 0x00/0x4d/0xc0 in the init table -> 0x4dc lines).
    fn default_exposure() -> u32 {
        0x4dc
    }

    /// Largest usable gain: 32x analog times ~1.9x digital fine gain.
    fn max_gain() -> f32 {
        60.0
    }

    /// Color-processing seeds consumed by the demosaic pipeline.
    fn color_calibration() -> ColorCalibration {
        ColorCalibration {
            black_level: 16,
            red_gain: 1.5,
            blue_gain: 1.5,
        }
    }
}

impl<I: CameraInterface> Sc202cs<I> {
    /// Write an 8-bit value to a 16-bit register address, retrying on bus errors.
    fn write(
        &mut self,
        reg: u16,
        val: u8,
    ) -> core::result::Result<(), <I::Bus as ErrorType>::Error> {
        let buf = [(reg >> 8) as u8, reg as u8, val];
        let mut result = self.interface.bus().write(self.address, &buf);
        for _ in 0..2 {
            if result.is_ok() {
                break;
            }
            std::thread::sleep(Duration::from_millis(5));
            result = self.interface.bus().write(self.address, &buf);
        }
        result
    }

    /// Write the sensor's register init sequence (soft reset first, so the
    /// sensor ends up configured and not streaming).
    pub fn init(&mut self) -> core::result::Result<(), <I::Bus as ErrorType>::Error> {
        for &(reg, val) in INIT_TABLE {
            self.write(reg, val)?;
            // The soft reset and PLL config registers need time to settle.
            if reg == 0x0103 || reg == 0x36e9 {
                std::thread::sleep(Duration::from_millis(10));
            }
        }
        Ok(())
    }

    /// Start streaming frames.
    fn start(&mut self) -> core::result::Result<(), <I::Bus as ErrorType>::Error> {
        self.write(0x0100, 0x01)?;
        std::thread::sleep(Duration::from_millis(200));
        Ok(())
    }

    /// Set the integration time in lines. Same SmartSens packing as the
    /// SC850SL: {0x3e00[3:0], 0x3e01, 0x3e02[7:4]} in 1/16-line units.
    fn set_exposure(
        &mut self,
        lines: u32,
    ) -> core::result::Result<(), <I::Bus as ErrorType>::Error> {
        let (min, max) = Self::exposure_range();
        let l = lines.clamp(min, max);
        self.write(0x3e00, ((l & 0xf000) >> 12) as u8)?;
        self.write(0x3e01, ((l & 0x0ff0) >> 4) as u8)?;
        self.write(0x3e02, ((l & 0x000f) << 4) as u8)
    }

    /// The SC202CS has coarse analog gain only (0x3e09 doubles per bucket);
    /// the digital fine gain (0x3e07, base 0x80 = 1x) interpolates between
    /// buckets, as in the vendor's gain map.
    fn set_gain(&mut self, gain: f32) -> core::result::Result<(), <I::Bus as ErrorType>::Error> {
        const BUCKETS: [(f32, u8); 6] = [
            (1.0, 0x00),
            (2.0, 0x01),
            (4.0, 0x03),
            (8.0, 0x07),
            (16.0, 0x0f),
            (32.0, 0x1f),
        ];
        let g = gain.clamp(1.0, Self::max_gain());
        let (base, coarse) = BUCKETS
            .iter()
            .rev()
            .find(|&&(b, _)| g >= b)
            .copied()
            .unwrap_or(BUCKETS[0]);
        let fine = ((0x80 as f32 * g / base).round() as i32).clamp(0x80, 0xf8) as u8;
        self.write(0x3e06, 0x00)?;
        self.write(0x3e07, fine)?;
        self.write(0x3e09, coarse)
    }

    /// Push a new operating point to the sensor, then wait for it to take
    /// effect (a frame or two later) before the next metering.
    fn apply_exposure(&mut self, point: AutoExposureState) {
        self.exposure = point.exposure;
        self.gain = point.gain;
        let _ = self.set_exposure(point.exposure);
        let _ = self.set_gain(point.gain);
        let _ = self.interface.wait_frame(FRAME_WAIT);
        let _ = self.interface.wait_frame(FRAME_WAIT);
    }
}

impl<I: CameraInterface> Camera for Sc202cs<I> {
    fn power_on(&mut self) {
        let _ = self.start();
    }

    /// Deliberately keeps the sensor streaming: the ESP32-P4 CSI/ISP pipeline
    /// does not survive the MIPI stream stopping mid-frame — after a sensor
    /// stop/start it never signals another completed frame, wedging the next
    /// capture. Idle streaming costs some power, which this bench board can
    /// afford.
    fn power_off(&mut self) {}

    /// Converge auto-exposure, then discard a few warm-up frames.
    fn calibrate(&mut self) {
        let format = self.format;
        let order = self.order;
        let limits = AutoExposureLimits {
            max_exposure: Self::exposure_range().1,
            max_gain: Self::max_gain(),
        };

        for iteration in 0..AUTO_EXPOSURE_MAX_ITERS {
            let Ok(frame) = self.interface.wait_frame(FRAME_WAIT) else {
                continue;
            };
            let metered = meter(frame, &format, order);
            log::info!(
                "auto-exposure iter {iteration}: p95={:.0} clip={:.1}% exp={} gain={:.2}x",
                metered.p_high,
                metered.clip * 100.0,
                self.exposure,
                self.gain,
            );

            let state = AutoExposureState {
                exposure: self.exposure,
                gain: self.gain,
            };
            match auto_exposure_step(state, limits, &metered) {
                AutoExposureStep::Converged => break,
                AutoExposureStep::Adjust(next) => self.apply_exposure(next),
            }
        }

        for _ in 0..WARMUP_FRAMES {
            let _ = self.interface.wait_frame(FRAME_WAIT);
        }
    }

    fn receive_frame(&mut self) -> Image {
        let format = self.format;
        let order = self.order;
        let output = self.output;
        let (black_level, wb_r, wb_b) = (self.black_level, self.wb_r, self.wb_b);
        match self.interface.wait_frame(FRAME_WAIT) {
            Ok(frame) if frame.len() >= format.bytes_per_frame() => {
                build_image(frame, &format, order, output, black_level, wb_r, wb_b)
            }
            _ => {
                log::error!("RGB: no frame available — producing a blank image");
                let pixels = vec![RgbPixel::new(0, 0, 0); output.0 * output.1];
                Image::from_pixels(output.0, output.1, pixels)
            }
        }
    }
}

// Init table: MIPI 1-lane / RAW10 / 1600x1200 / 30 fps (24 MHz input clock).
// Source: esp_cam_sensor sc202cs driver, init_reglist_MIPI_1lane_raw10_1600x1200_30fps.
#[rustfmt::skip]
const INIT_TABLE: &[(u16, u8)] = &[
    (0x0103, 0x01), (0x0100, 0x00),
    (0x36e9, 0x80), (0x36e9, 0x24),
    (0x301f, 0x01), (0x3301, 0xff),
    (0x3304, 0x68), (0x3306, 0x40),
    (0x3308, 0x08), (0x3309, 0xa8),
    (0x330b, 0xb0), (0x330c, 0x18),
    (0x330d, 0xff), (0x330e, 0x20),
    (0x331e, 0x59), (0x331f, 0x99),
    (0x3333, 0x10), (0x335e, 0x06),
    (0x335f, 0x08), (0x3364, 0x1f),
    (0x337c, 0x02), (0x337d, 0x0a),
    (0x338f, 0xa0), (0x3390, 0x01),
    (0x3391, 0x03), (0x3392, 0x1f),
    (0x3393, 0xff), (0x3394, 0xff),
    (0x3395, 0xff), (0x33a2, 0x04),
    (0x33ad, 0x0c), (0x33b1, 0x20),
    (0x33b3, 0x38), (0x33f9, 0x40),
    (0x33fb, 0x48), (0x33fc, 0x0f),
    (0x33fd, 0x1f), (0x349f, 0x03),
    (0x34a6, 0x03), (0x34a7, 0x1f),
    (0x34a8, 0x38), (0x34a9, 0x30),
    (0x34ab, 0xb0), (0x34ad, 0xb0),
    (0x34f8, 0x1f), (0x34f9, 0x20),
    (0x3630, 0xa0), (0x3631, 0x92),
    (0x3632, 0x64), (0x3633, 0x43),
    (0x3637, 0x49), (0x363a, 0x85),
    (0x363c, 0x0f), (0x3650, 0x31),
    (0x3670, 0x0d), (0x3674, 0xc0),
    (0x3675, 0xa0), (0x3676, 0xa0),
    (0x3677, 0x92), (0x3678, 0x96),
    (0x3679, 0x9a), (0x367c, 0x03),
    (0x367d, 0x0f), (0x367e, 0x01),
    (0x367f, 0x0f), (0x3698, 0x83),
    (0x3699, 0x86), (0x369a, 0x8c),
    (0x369b, 0x94), (0x36a2, 0x01),
    (0x36a3, 0x03), (0x36a4, 0x07),
    (0x36ae, 0x0f), (0x36af, 0x1f),
    (0x36bd, 0x22), (0x36be, 0x22),
    (0x36bf, 0x22), (0x36d0, 0x01),
    (0x370f, 0x02), (0x3721, 0x6c),
    (0x3722, 0x8d), (0x3725, 0xc5),
    (0x3727, 0x14), (0x3728, 0x04),
    (0x37b7, 0x04), (0x37b8, 0x04),
    (0x37b9, 0x06), (0x37bd, 0x07),
    (0x37be, 0x0f), (0x3901, 0x02),
    (0x3903, 0x40), (0x3905, 0x8d),
    (0x3907, 0x00), (0x3908, 0x41),
    (0x391f, 0x41), (0x3933, 0x80),
    (0x3934, 0x02), (0x3937, 0x6f),
    (0x393a, 0x01), (0x393d, 0x01),
    (0x393e, 0xc0), (0x39dd, 0x41),
    (0x3e00, 0x00), (0x3e01, 0x4d),
    (0x3e02, 0xc0), (0x3e09, 0x00),
    (0x4509, 0x28), (0x450d, 0x61),
];

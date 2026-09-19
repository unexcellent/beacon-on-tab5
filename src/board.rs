//! Tab5 board bring-up: the shared system I2C bus and the two PI4IOE5V6408 IO
//! expanders that gate board power rails, the speaker amplifier and reset lines.
//!
//! Expander register values mirror `bsp_io_expander_pi4ioe_init` in M5Stack's
//! Tab5 BSP (M5Tab5-UserDemo, m5stack_tab5.c) — the known-good power-up state.

use std::time::Duration;

use beacon::camera::esp::{EspI2c, I2cConfig};
use beacon::error::{Error, Result};
use embedded_hal::i2c::I2c;
use esp_idf_sys::i2c_port_t_I2C_NUM_0;

const SYS_I2C_SDA_PIN: i32 = 31;
const SYS_I2C_SCL_PIN: i32 = 32;

const PI4IOE1_ADDR: u8 = 0x43;
const PI4IOE2_ADDR: u8 = 0x44;

// PI4IOE5V6408 registers.
const PI4IO_REG_CHIP_RESET: u8 = 0x01;
const PI4IO_REG_IO_DIR: u8 = 0x03;
const PI4IO_REG_OUT_SET: u8 = 0x05;
const PI4IO_REG_OUT_H_IM: u8 = 0x07;
const PI4IO_REG_IN_DEF_STA: u8 = 0x09;
const PI4IO_REG_PULL_EN: u8 = 0x0B;
const PI4IO_REG_PULL_SEL: u8 = 0x0D;
const PI4IO_REG_INT_MASK: u8 = 0x11;

/// Bring up the Tab5 system I2C bus (camera, codec, expanders, RTC, IMU).
pub fn initialize_i2c() -> Result<EspI2c> {
    EspI2c::new(I2cConfig {
        port: i2c_port_t_I2C_NUM_0 as i32,
        sda_pin: SYS_I2C_SDA_PIN,
        scl_pin: SYS_I2C_SCL_PIN,
        internal_pullups: false,
        reset_on_init: false,
        scl_speed_hz: 400_000,
        scl_wait_us: 20_000,
        timeout_ms: 50,
    })
    .map_err(|_| Error::Peripheral)
}

/// Program both IO expanders to the BSP's power-up state. Expander 1's P1
/// enables the NS4150 speaker amplifier (left high, so the speaker is live);
/// expander 2 keeps the default rails (WLAN power, USB-A 5 V) on.
pub fn initialize_io_expanders(i2c: &mut EspI2c) -> Result<()> {
    let mut write = |addr: u8, reg: u8, val: u8| {
        i2c.write(addr, &[reg, val]).map_err(|e| {
            log::error!("IO expander 0x{addr:02x} reg 0x{reg:02x} write failed: {e:?}");
            Error::Peripheral
        })
    };

    // Expander 1: display/touch resets, speaker amp enable (P1 high), P4 is the
    // LCD reset released via input-with-pullup (no 3.3 V push on a 1.8 V rail).
    write(PI4IOE1_ADDR, PI4IO_REG_CHIP_RESET, 0xFF)?;
    write(PI4IOE1_ADDR, PI4IO_REG_PULL_SEL, 0b0111_1111)?;
    write(PI4IOE1_ADDR, PI4IO_REG_PULL_EN, 0b0111_1111)?;
    write(PI4IOE1_ADDR, PI4IO_REG_OUT_SET, 0b0110_0110)?;
    write(PI4IOE1_ADDR, PI4IO_REG_OUT_H_IM, 0b0000_0000)?;
    write(PI4IOE1_ADDR, PI4IO_REG_IO_DIR, 0b0111_1111)?;
    std::thread::sleep(Duration::from_millis(10));
    write(PI4IOE1_ADDR, PI4IO_REG_IO_DIR, 0b0110_1111)?;
    std::thread::sleep(Duration::from_millis(50));

    // Expander 2: power rails (P0 WLAN, P3 USB-A 5 V on) and charger control.
    write(PI4IOE2_ADDR, PI4IO_REG_CHIP_RESET, 0xFF)?;
    write(PI4IOE2_ADDR, PI4IO_REG_IO_DIR, 0b1011_1001)?;
    write(PI4IOE2_ADDR, PI4IO_REG_OUT_H_IM, 0b0000_0110)?;
    write(PI4IOE2_ADDR, PI4IO_REG_PULL_SEL, 0b1011_1001)?;
    write(PI4IOE2_ADDR, PI4IO_REG_PULL_EN, 0b1111_1001)?;
    write(PI4IOE2_ADDR, PI4IO_REG_IN_DEF_STA, 0b0100_0000)?;
    write(PI4IOE2_ADDR, PI4IO_REG_INT_MASK, 0b1011_1111)?;
    write(PI4IOE2_ADDR, PI4IO_REG_OUT_SET, 0b0000_1001)?;

    Ok(())
}

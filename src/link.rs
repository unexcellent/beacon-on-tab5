//! Tab5 payload link: CSP over KISS over the ESP32-P4's USB-Serial-JTAG port
//! (the Tab5's USB-C connector). The same port carries the console log; both go
//! through the usb_serial_jtag driver's ring buffer, and the laptop side's KISS
//! decoder skips any log bytes between frames.

use beacon::error::{Error, Result};
use beacon::link::csp::{CspLink, CspLinkConfig, SerialRead, SerialWrite};
use beacon::link::payload::PayloadLink;
use beacon::link::{CommandLink, NODE};
use esp_idf_sys::{
    ESP_OK, esp_vfs_usb_serial_jtag_use_driver, usb_serial_jtag_driver_config_t,
    usb_serial_jtag_driver_install, usb_serial_jtag_read_bytes, usb_serial_jtag_wait_tx_done,
    usb_serial_jtag_write_bytes,
};

/// Poll timeout of the RX pump (ticks == ms at the configured 1 kHz tick rate).
const READ_TIMEOUT_TICKS: u32 = 100;
/// Bound on TX so a host that stopped draining the port cannot wedge the loop.
const WRITE_TIMEOUT_TICKS: u32 = 1_000;

/// Sized to absorb a whole OTA data burst (the RX pump drains between chunks).
const RX_BUFFER_SIZE: i32 = 16_384;
const TX_BUFFER_SIZE: i32 = 16_384;

/// Bring up the USB-Serial-JTAG driver and the CSP node, and wrap them in the
/// mission's payload link.
pub fn initialize_usb_link() -> Result<impl CommandLink> {
    unsafe {
        let mut config = usb_serial_jtag_driver_config_t {
            tx_buffer_size: TX_BUFFER_SIZE as u32,
            rx_buffer_size: RX_BUFFER_SIZE as u32,
        };
        if usb_serial_jtag_driver_install(&mut config) != ESP_OK {
            return Err(Error::UartAllocation);
        }
        // Route the console through the same driver: log bytes and KISS frames
        // then share one ring buffer and cannot interleave mid-frame.
        esp_vfs_usb_serial_jtag_use_driver();
    }

    let csp = CspLink::try_new(
        CspLinkConfig {
            address: NODE,
            hostname: "beacon",
            model: "esp32p4-tab5",
        },
        UsbSerialTx,
        UsbSerialRx,
    )
    .map_err(|_| Error::CspInit)?;

    PayloadLink::try_new(csp)
}

/// TX half of the USB-Serial-JTAG port (the driver serializes concurrent writers).
struct UsbSerialTx;

impl SerialWrite for UsbSerialTx {
    fn write_all(&mut self, data: &[u8]) {
        let mut sent = 0;
        while sent < data.len() {
            let n = unsafe {
                usb_serial_jtag_write_bytes(
                    data[sent..].as_ptr() as *const core::ffi::c_void,
                    data.len() - sent,
                    WRITE_TIMEOUT_TICKS,
                )
            };
            if n <= 0 {
                // The host stopped draining the port; drop the rest of the
                // frame rather than block the command loop forever.
                log::warn!(
                    "USB TX: host not draining, dropping {} bytes",
                    data.len() - sent
                );
                return;
            }
            sent += n as usize;
        }
        unsafe { usb_serial_jtag_wait_tx_done(WRITE_TIMEOUT_TICKS) };
    }
}

/// RX half of the USB-Serial-JTAG port.
struct UsbSerialRx;

impl SerialRead for UsbSerialRx {
    fn read(&mut self, buf: &mut [u8]) -> core::result::Result<usize, ()> {
        let n = unsafe {
            usb_serial_jtag_read_bytes(
                buf.as_mut_ptr() as *mut core::ffi::c_void,
                buf.len() as u32,
                READ_TIMEOUT_TICKS,
            )
        };
        // The driver returns the byte count (0 on timeout); it has no error path.
        Ok(n as usize)
    }
}

//! Beacon firmware for the M5Stack Tab5 (ESP32-P4): the MOVE-IIIa SSTV beacon
//! ported to bench hardware. Waits for the SSTV command on the USB-C serial
//! link, captures a frame from the Tab5's SC202CS camera, Robot36-encodes it
//! and plays it through the built-in speaker (ES8388 codec).
//!
//! All mission logic (idle loop, SSTV transmit, firmware update, CSP link
//! protocol) comes from the `beacon` crate; this binary only supplies the Tab5
//! bring-up: the USB-Serial-JTAG payload link, the shared I2C bus with its IO
//! expanders, the ES8388 audio channel and the SC202CS camera.

mod audio;
#[cfg(feature = "usb-audio-dump")]
mod audio_dump;
mod board;
mod link;
mod sc202cs;

use beacon::camera::Camera;
use beacon::error::ReportIfErr;
use beacon::idle::idle;
use beacon::link::{CommandLink, Message};

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let mut link = link::initialize_usb_link().unwrap();
    report_successful_boot(&link);

    // One physical I2C bus is shared by the IO expanders, the audio codec and
    // the camera. The one-shot configuration (expanders, codec registers) runs
    // first; afterwards the camera's CSI transport takes the bus over.
    let i2c = board::initialize_i2c()
        .and_then(|mut i2c| {
            board::initialize_io_expanders(&mut i2c)?;
            Ok(i2c)
        })
        .report_if_err(&link)
        .unwrap();

    let mut i2c = i2c;
    let audio = audio::initialize_audio_channel(&mut i2c)
        .report_if_err(&link)
        .unwrap();
    #[cfg(feature = "usb-audio-dump")]
    let audio = audio_dump::TeeDump::new(audio);
    let mut audio = audio;

    let cameras = vec![
        sc202cs::initialize_rgb_camera(i2c)
            .report_if_err(&link)
            .ok()
            .map(boxed),
    ];

    link.send(Message::Available);
    idle(&mut link, cameras, &mut audio);
}

fn report_successful_boot(link: &impl CommandLink) {
    let description = unsafe { &*esp_idf_svc::sys::esp_app_get_description() };

    let version =
        unsafe { std::ffi::CStr::from_ptr(description.version.as_ptr()) }.to_string_lossy();

    let elf_sha: String = description
        .app_elf_sha256
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect();

    let running = unsafe { &*esp_idf_svc::sys::esp_ota_get_running_partition() };

    let partition = unsafe { std::ffi::CStr::from_ptr(running.label.as_ptr()) }.to_string_lossy();

    link.send(Message::Booted(format!("{version} {elf_sha} {partition}")));
}

/// Erase a camera's concrete type so different cameras share one collection.
fn boxed<C: Camera + 'static>(camera: C) -> Box<dyn Camera> {
    Box::new(camera)
}

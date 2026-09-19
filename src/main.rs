mod audio;
#[cfg(feature = "usb-audio-dump")]
mod audio_dump;
mod board;
mod link;
mod sc202cs;

use beacon::camera::Camera;
use beacon::camera::esp::EspI2c;
use beacon::error::{ReportIfErr, Result};
use beacon::idle::idle;
use beacon::link::{CommandLink, Message};

use crate::audio::initialize_audio_channel;
use crate::link::initialize_usb_link as bring_up_payload_link;
use crate::sc202cs::initialize_rgb_camera;

fn main() {
    initialize_esp32();
    let mut link = initialize_payload_link().unwrap();

    let mut i2c = initialize_board().report_if_err(&link).unwrap();
    let mut audio = initialize_audio_channel(&mut i2c)
        .report_if_err(&link)
        .unwrap();

    let rgb_camera = initialize_rgb_camera(i2c)
        .report_if_err(&link)
        .ok()
        .map(boxed);

    link.send(Message::Available);
    idle(&mut link, vec![rgb_camera], &mut audio);
}

fn initialize_esp32() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
}

fn initialize_payload_link() -> Result<impl CommandLink> {
    let link = bring_up_payload_link()?;

    report_successful_boot(&link);

    Ok(link)
}

/// Bring up the one physical I2C bus shared by the IO expanders, the audio
/// codec and the camera. The one-shot configuration (expanders, codec
/// registers) runs first; afterwards the camera's CSI transport takes the bus
/// over.
fn initialize_board() -> Result<EspI2c> {
    let mut i2c = board::initialize_i2c()?;
    board::initialize_io_expanders(&mut i2c)?;
    Ok(i2c)
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

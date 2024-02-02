use crate::{
    hardware::hardware_mac_args,
    mac::{open_mac_rx_callback, open_mac_tx_func_callback},
};

mod c_macro_replacements;
mod hardware;
mod mac;
mod proprietary;
mod wifi_80211;

fn main() {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("Hello, world!");
    let open_hw_args = hardware_mac_args {
        _rx_callback: open_mac_rx_callback,
        _tx_func_callback: open_mac_tx_func_callback,
    };
}

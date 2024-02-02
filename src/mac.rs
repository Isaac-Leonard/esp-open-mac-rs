use std::{
    mem::{size_of, size_of_val, MaybeUninit},
    ptr::{null_mut, slice_from_raw_parts},
};

use esp_idf_svc::sys::{
    esp_timer_get_time, malloc, memcmp, memcpy, wifi_promiscuous_pkt_t, xQueueReceive,
    QueueHandle_t,
};
use log::{debug, error, info, warn};

use crate::{
    c_macro_replacements::{xPortGetCoreID, BROADCAST_MAC},
    hardware::{module_mac_addr, tx_func},
    wifi_80211::*,
    xQueueCreate, xQueueSendToBack, MAC2STR,
};

#[repr(C)]
#[derive(Clone, Copy)]
enum openmac_sta_state_t {
    INIT,
    IDLE,
    AUTHENTICATED,
    ASSOCIATED,
}

const TAG: &str = "mac.c";
static mut tx: Option<tx_func> = None;
static mut reception_queue: QueueHandle_t = null_mut();

const to_ap_auth_frame: [u8; 34] = [
    0xb0, 0x00, 0x00, 0x00, 0x4e, 0xed, 0xfb, 0x35, 0x22, 0xa8, // receiver addr
    0x00, 0x23, 0x45, 0x67, 0x89, 0xab, // transmitter addr
    0x4e, 0xed, 0xfb, 0x35, 0x22, 0xa8, // bssid
    0x00, 0x00, // sequence control
    0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0, 0, 0, 0, /*FCS*/
];

const to_ap_assoc_frame: [u8; 52] = [
    0x00, 0x00, 0x00, 0x00, 0x4e, 0xed, 0xfb, 0x35, 0x22, 0xa8, // receiver addr
    0x00, 0x23, 0x45, 0x67, 0x89, 0xab, // transmitter addr
    0x4e, 0xed, 0xfb, 0x35, 0x22, 0xa8, // bssid
    0x00, 0x00, // sequence control
    0x11, 0x00, 0x0a, 0x00, // Fixed parameters
    0x00, 0x08, 0x6d, 0x65, 0x73, 0x68, 0x74, 0x65, 0x73, 0x74, // SSID
    0x01, 0x08, 0x0c, 0x12, 0x18, 0x24, 0x30, 0x48, 0x60, 0x6c, // Supported rates
    0, 0, 0, 0, /*FCS*/
];

static mut to_ap_data_frame: [u8; 77] = [
    0x08, 0x01, 0x00, 0x00, 0x4e, 0xed, 0xfb, 0x35, 0x22, 0xa8, // receiver addr
    0x00, 0x23, 0x45, 0x67, 0x89, 0xab, // transmitter
    0x84, 0x2b, 0x2b, 0x4f, 0x89, 0x4f, // destination
    0x00, 0x00, // sequence control
    0xaa, 0xaa, 0x03, 0x00, 0x00, 0x00, 0x08, 0x00, 0x45, 0x00, 0x00, 0x29, 0x00, 0x01, 0x00, 0x00,
    0x40, 0x11, 0x34, 0xba, 0x0a, 0x00, 0x32, 0x02, 0x0a, 0x00, 0x00, 0x08, 0xea, 0x61, 0x0d, 0x05,
    0x00, 0x15, 0x46, 0x64, 0x48, 0x65, 0x6c, 0x6c, 0x6f, 0x20, 0x77, 0x6f, 0x72, 0x6c, 0x64, 0x21,
    0x0a, 0, 0, 0, 0, /*FCS*/
];

// Gets called with a packet that was received. This function does not need to free the memory of the packet,
//  but the packet will become invalid after this function returns. If you need any data from the packet,
//  better copy it before returning!
// Please avoid doing heavy processing here: it's not in an interrupt, but if this function is not fast enough,
// the RX queue that is used to pass packets to this function might overflow and drop packets.
// TODO: Change to take ownership
pub unsafe extern "C" fn open_mac_rx_callback(packet: *mut wifi_promiscuous_pkt_t) {
    let p: *mut mac80211_frame = packet.as_mut().unwrap().payload.as_mut_ptr().cast();

    // fuck beacon frames, all my homies hate beacon frames
    // Oh dear, what am I in for
    if p.as_mut().unwrap().frame_control.type_() == IEEE80211_TYPE_MGT
        && p.as_mut().unwrap().frame_control.sub_type() == IEEE80211_TYPE_MGT_SUBTYPE_BEACON
    {
        return;
    }

    // check that receiver mac address matches our mac address or is broadcast
    if memcmp(
        module_mac_addr.as_ptr().cast(),
        p.as_mut().unwrap().receiver_address.as_ptr().cast(),
        6,
    ) != 0
        && memcmp(
            BROADCAST_MAC.as_ptr().cast(),
            p.as_mut().unwrap().receiver_address.as_ptr().cast(),
            6,
        ) != 0
    {
        // We're not interested in this packet, return early to avoid having to copy it further to the networking stack
        debug!(
            "{}: Discarding packet from {} to {}",
            TAG,
            MAC2STR!(p.as_mut().unwrap().transmitter_address),
            MAC2STR!(p.as_mut().unwrap().receiver_address)
        );
        return;
    }
    error!(
        "{}: Accepted: from {} to {} type={}, subtype={} from_ds={} to_ds={}",
        TAG,
        MAC2STR!(p.as_mut().unwrap().transmitter_address),
        MAC2STR!(p.as_mut().unwrap().receiver_address),
        p.as_mut().unwrap().frame_control.type_(),
        p.as_mut().unwrap().frame_control.sub_type(),
        p.as_mut().unwrap().frame_control.from_ds(),
        p.as_mut().unwrap().frame_control.to_ds()
    );

    if reception_queue.is_null() {
        error!("{}: Received, but queue does not exist yet", TAG);
        return;
    }
    // 28 is size of rx_ctrl, 4 is size of FCS (which we don't need)
    let mut packet_queue_copy: *mut wifi_promiscuous_pkt_t =
        malloc(packet.as_mut().unwrap().rx_ctrl.sig_len() + 28 - 4).cast();
    memcpy(
        packet_queue_copy.cast(),
        packet.cast(),
        packet.as_mut().unwrap().rx_ctrl.sig_len() + 28 - 4,
    );

    if xQueueSendToBack!(reception_queue, packet_queue_copy.cast(), 0) == 0 {
        warn!("{}: MAC RX queue full!", TAG);
    }
}

// This function will get called exactly once, with as argument a function (`bool tx_func(uint8_t* packet, uint32_t len)`).
// The function that is passed will TX packets. If it returned `true`, that means that the packet was sent. If false,
//  you'll need to call the function again.
pub unsafe extern "C" fn open_mac_tx_func_callback(t: tx_func) {
    tx = Some(t);
}

pub unsafe extern "C" fn mac_task(pvParameters: *mut core::ffi::c_void) {
    error!(
        "{}: Starting mac_task, running on {}",
        TAG,
        xPortGetCoreID()
    );

    reception_queue = xQueueCreate!(10, size_of::<*mut wifi_promiscuous_pkt_t>() as u32);
    assert!(!reception_queue.is_null());
    let mut sta_state: openmac_sta_state_t = openmac_sta_state_t::IDLE;
    let mut last_transmission_us = esp_timer_get_time();
    loop {
        let mut packet: MaybeUninit<wifi_promiscuous_pkt_t> = MaybeUninit::zeroed();
        if xQueueReceive(reception_queue, packet.as_mut_ptr().cast(), 10) != 0 {
            let packet = packet.assume_init_mut();
            let mut p: *mut mac80211_frame = packet.payload.as_mut_ptr().cast();

            if !(p.as_mut().unwrap().frame_control.type_() == IEEE80211_TYPE_MGT
                && p.as_mut().unwrap().frame_control.sub_type()
                    == IEEE80211_TYPE_MGT_SUBTYPE_BEACON)
            {
                // Print all non-beacon packets
                // Would be nice to print as hex like the original
                info!(
                    "{}: packet-content {:?}, {}",
                    TAG,
                    // Is the - 4 needed here?
                    slice_from_raw_parts(
                        packet.payload.as_mut_ptr().cast::<u8>(),
                        (packet.rx_ctrl.sig_len() - 4) as usize
                    ),
                    packet.rx_ctrl.sig_len() - 4
                );
            }

            match sta_state {
                openmac_sta_state_t::IDLE =>
                // idle, wait for authenticate packet
                {
                    if p.as_mut().unwrap().frame_control.type_() == IEEE80211_TYPE_MGT
                        && p.as_mut().unwrap().frame_control.sub_type()
                            == IEEE80211_TYPE_MGT_SUBTYPE_AUTHENTICATION
                    {
                        // TODO check that authentication succeeded
                        // For now, assume it's fine
                        warn!(
                            "{}: Authentication received from={} to= {}",
                            TAG,
                            MAC2STR!(p.as_mut().unwrap().transmitter_address),
                            MAC2STR!(p.as_mut().unwrap().receiver_address)
                        );
                        sta_state = openmac_sta_state_t::AUTHENTICATED;
                        last_transmission_us = 0;
                    }
                }
                openmac_sta_state_t::AUTHENTICATED =>
                // authenticated, wait for association response packet
                {
                    if p.as_mut().unwrap().frame_control.type_() == IEEE80211_TYPE_MGT
                        && p.as_mut().unwrap().frame_control.sub_type()
                            == IEEE80211_TYPE_MGT_SUBTYPE_ASSOCIATION_RESP
                    {
                        // TODO check that association succeeded
                        // For now, assume it's fine
                        warn!(
                            "{}: Association response received from={} to= {}",
                            TAG,
                            MAC2STR!(p.as_mut().unwrap().transmitter_address),
                            MAC2STR!(p.as_mut().unwrap().receiver_address)
                        );
                        sta_state = openmac_sta_state_t::ASSOCIATED;
                        last_transmission_us = 0;
                    }
                }
                openmac_sta_state_t::ASSOCIATED =>
                    // associated
                    {}
                _ => {}
            }
            // TODO: Really not sure if this is correct
            esp_idf_svc::sys::free((packet as *mut wifi_promiscuous_pkt_t).cast());
        }

        // don't transmit too fast
        if esp_timer_get_time() - last_transmission_us < 1000 * 1000 {
            continue;
        }

        // don't transmit if we don't know how to
        if tx.is_none() {
            warn!("{}: no transmit function yet", TAG);
            continue;
        };

        match sta_state {
            openmac_sta_state_t::IDLE => {
                error!("{}: Sending authentication frame!", TAG);
                tx.as_ref().unwrap()(
                    to_ap_auth_frame.as_mut_ptr().cast(),
                    size_of_val(&to_ap_auth_frame) as u32,
                );
            }
            openmac_sta_state_t::AUTHENTICATED => {
                error!("{}: Sending association request frame!", TAG);
                tx.as_ref().unwrap()(
                    to_ap_assoc_frame.as_mut_ptr().cast(),
                    size_of_val(&to_ap_assoc_frame) as u32,
                );
            }
            openmac_sta_state_t::ASSOCIATED => {
                error!("{}: Sending data frame", TAG);
                tx.as_ref().unwrap()(
                    to_ap_data_frame.as_mut_ptr().cast(),
                    size_of_val(&to_ap_data_frame) as u32,
                );
            }
            _ => {}
        }
        last_transmission_us = esp_timer_get_time();
    }
}

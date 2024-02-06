use core::slice;
use std::{
    mem::{size_of, size_of_val, MaybeUninit},
    ptr::{null_mut, slice_from_raw_parts},
};

use esp_idf_svc::sys::{
    abort, calloc, esp_err_t, esp_netif_action_connected, esp_netif_action_start, esp_netif_attach,
    esp_netif_config_t, esp_netif_driver_base_t, esp_netif_driver_ifconfig_t,
    esp_netif_flags_ESP_NETIF_DHCP_CLIENT, esp_netif_flags_ESP_NETIF_FLAG_EVENT_IP_MODIFIED,
    esp_netif_flags_ESP_NETIF_FLAG_GARP, esp_netif_flags_ESP_NETIF_FLAG_MLDV6_REPORT,
    esp_netif_inherent_config_t, esp_netif_new, esp_netif_receive, esp_netif_set_driver_config,
    esp_netif_set_hostname, esp_netif_set_mac, esp_netif_t, esp_timer_get_time,
    ip_event_t_IP_EVENT_STA_GOT_IP, ip_event_t_IP_EVENT_STA_LOST_IP, malloc, memcmp, memcpy,
    memset, wifi_promiscuous_pkt_t, xQueueReceive, QueueHandle_t, CONFIG_LWIP_ESP_GRATUITOUS_ARP,
    CONFIG_LWIP_ESP_MLDV6_REPORT, CONFIG_LWIP_IPV4, ESP_FAIL, ESP_OK,
};
use log::{debug, error, info, warn};

use crate::{
    c_macro_replacements::{xPortGetCoreID, BROADCAST_MAC, ESP_NETIF_NETSTACK_DEFAULT_WIFI_STA},
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
// TODO: This and the proceeding type need to be checked.
struct openmac_netif_driver {
    base: esp_netif_driver_base_t,
}

type openmac_netif_driver_t = *mut openmac_netif_driver;

static mut receive_task_is_running: bool = true;
static mut netif_openmac: *mut esp_netif_t = null_mut();

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

const data_frame_template: [u8; 32] = [
    0x08, 0x01, // frame control
    0x00, 0x00, // duration/ID
    0x4e, 0xed, 0xfb, 0x35, 0x22, 0xa8, // receiver addr
    0x00, 0x23, 0x45, 0x67, 0x89, 0xab, // transmitter
    0x84, 0x2b, 0x2b, 0x4f, 0x89, 0x4f, // destination
    0x00, 0x00, // sequence control
    0xaa, 0xaa, // SNAP
    0x03, 0x00, 0x00, 0x00, // other LLC headers
    0xAA, 0xBB, // type (AA BB because this needs to be overwritten)
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
                    "netif - tx 802.11 {:?}, {}",
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
                        // TODO: Odd cast here
                        esp_netif_action_connected(netif_openmac.cast(), null_mut(), 0, null_mut());
                    }
                }
                openmac_sta_state_t::ASSOCIATED =>
                // associated
                {
                    if p.as_ref().unwrap().frame_control.type_() == IEEE80211_TYPE_DATA
                        && p.as_ref().unwrap().frame_control.sub_type()
                            == IEEE80211_TYPE_DATA_SUBTYPE_DATA
                    {
                        warn!("assoc-data: Received data frame, will handle");
                        if p.as_ref().unwrap().frame_control.to_ds()
                            == p.as_ref().unwrap().frame_control.from_ds()
                        {
                            warn!(
                                "unhandled data frame case from={} to={}",
                                p.as_ref().unwrap().frame_control.from_ds(),
                                (&*p).frame_control.to_ds()
                            );
                            break;
                        }
                        // construct frame for netif
                        let mac_header_size: usize =
                            size_of::<mac80211_frame>() - size_of_val(&(&*p).data_and_fcs);
                        // TODO check that this data actually includes the FCS
                        // TODO header size can be variable, handle that case
                        let mac_data_size:usize =(&* packet). rx_ctrl.sig_len()as usize - 4 /*FCS*/ - mac_header_size - 8 /*LLC header size*/;
                        let netif_frame_size: usize =
                            6 /*destination*/ + 6 /*source*/ + 2 /*type*/ + mac_data_size;
                        let mut netif_frame: *mut u8 = malloc(netif_frame_size as u32).cast();

                        warn!(
                            "handled case from={} to={}",
                            (&*p).frame_control.from_ds(),
                            (&*p).frame_control.to_ds()
                        );
                        // format of netif frame: destination MAC, source MAC, type (2 bytes), data

                        // Note Changed from original
                        // It appears that both branches had the exact same code
                        // So We just add an or to the check and remove the else if
                        if ((&*p).frame_control.to_ds() == 0 && (&*p).frame_control.from_ds() != 0)
                            || ((&*p).frame_control.to_ds() != 0
                                && (&*p).frame_control.from_ds() == 0)
                        {
                            // to_ds=0 from_ds=1
                            memcpy(
                                netif_frame.cast(),
                                (&*p).receiver_address.as_ptr().cast(),
                                6,
                            );
                            memcpy(
                                netif_frame.add(6).cast(),
                                (&*p).address_3.as_ptr().cast(),
                                6,
                            );
                        // address 3 is source address if to_ds=0 from_ds=1
                        } else {
                            error!("Something wrong, aborting");
                            abort();
                            break;
                        }
                        // copy LLC type
                        memcpy(
                            netif_frame.add(12).cast(),
                            (&*p).data_and_fcs.as_ptr().add(6).cast(),
                            2,
                        );
                        // copy data
                        memcpy(
                            netif_frame.add(14).cast(),
                            (&*p).data_and_fcs.as_ptr().add(8).cast(),
                            mac_data_size as u32,
                        );
                        info!("passing data frame to netif");
                        // TODO we can log this buffer if useful
                        // TODO: is it useful to populate eb?
                        info!(
                            "netif-rx: Received packet dest= {} from= {}.",
                            MAC2STR!(slice::from_raw_parts(netif_frame.cast_const(), 6)),
                            MAC2STR!(slice::from_raw_parts(netif_frame.add(6), 6))
                        );

                        esp_netif_receive(
                            netif_openmac,
                            netif_frame.cast(),
                            netif_frame_size,
                            null_mut(),
                        );
                    }
                }
                _ => {}
            }
            // TODO: Really not sure if this is correct
            esp_idf_svc::sys::free((packet as *mut wifi_promiscuous_pkt_t).cast());
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
                info!("{}: Sending association request frame!", TAG);
                tx.as_ref().unwrap()(
                    to_ap_assoc_frame.as_mut_ptr().cast(),
                    size_of_val(&to_ap_assoc_frame) as u32,
                );
            }
            openmac_sta_state_t::ASSOCIATED => {
                //info!("Sending No need to send anything anymore");
            }
            _ => {}
        }
        last_transmission_us = esp_timer_get_time();
    }
}

unsafe extern "C" fn openmac_netif_transmit(
    h: *mut core::ffi::c_void,
    buffer: *mut core::ffi::c_void,
    len: usize,
) -> esp_err_t {
    let mut eth_data: *mut u8 = buffer.cast();
    info!(
        "netif-tx: Going to transmit a packet: to {} from {} type={:02x}{:02x}",
        MAC2STR!(slice::from_raw_parts(eth_data, 6)),
        MAC2STR!(slice::from_raw_parts(eth_data.add(6), 6)),
        eth_data.add(12).read(),
        eth_data.add(13).read()
    );
    info!("netif-tx: {:?}", slice::from_raw_parts(eth_data, len));
    // We need to transform this Ethernet packet to a packet that can be sent via 802.11 data frame
    // Luckily for us; that's pretty do-able
    let wifi_packet_size:usize = size_of_val(&data_frame_template) - (6+6+2/*ethernet header*/) + len + 4 /*FCS*/;
    let mut wifi_data_frame: *mut u8 = malloc(wifi_packet_size as u32).cast();

    // Copy over wifi data frame template
    memcpy(
        wifi_data_frame.cast(),
        data_frame_template.as_ptr().cast(),
        size_of_val(&data_frame_template) as u32,
    );
    // Set destination MAC address
    memcpy(wifi_data_frame.add(4 + 2 * 6).cast(), eth_data.cast(), 6);
    // Set transmitter MAC address
    memcpy(wifi_data_frame.add(4 + 6).cast(), eth_data.add(6).cast(), 6);
    // Set type
    memcpy(
        wifi_data_frame
            .add(size_of_val(&data_frame_template) - 2)
            .cast(),
        eth_data.add(2 * 6).cast(),
        2,
    );
    // Set data
    memcpy(
        wifi_data_frame
            .add(size_of_val(&data_frame_template))
            .cast(),
        eth_data.add(2 * 6 + 2).cast(),
        len as u32 - (2 * 6 + 2),
    );
    // Set FCS to 0
    memset(wifi_data_frame.add(wifi_packet_size - 4).cast(), 0, 4);

    let Some(ref tx_local) = tx else {
        return ESP_FAIL;
    };
    // TODO check that we have TX slots before transmitting
    // ESP_LOGI("netif-tx", "transformed packet");
    // ESP_LOG_BUFFER_HEXDUMP("netif-tx", wifi_data_frame, wifi_packet_size, ESP_LOG_INFO);

    tx_local(wifi_data_frame, wifi_packet_size as u32);
    esp_idf_svc::sys::free(wifi_data_frame.cast());

    return ESP_OK;
}

unsafe extern "C" fn openmac_netif_transmit_wrap(
    h: *mut core::ffi::c_void,
    buffer: *mut core::ffi::c_void,
    len: usize,
    netstack_buf: *mut core::ffi::c_void,
) -> esp_err_t {
    return openmac_netif_transmit(h, buffer.cast(), len);
}

// Free RX buffer (not used as the buffer is static)
// TODO ^ is this true?
pub unsafe extern "C" fn openmac_free(h: *mut core::ffi::c_void, buffer: *mut core::ffi::c_void) {
    info!("Free-ing RX'd packet {}", buffer as usize);
    esp_idf_svc::sys::free(buffer);
}

unsafe extern "C" fn openmac_driver_start(
    esp_netif: *mut esp_netif_t,
    args: *mut core::ffi::c_void,
) -> esp_err_t {
    let mut driver: openmac_netif_driver_t = args.cast();
    (&mut *driver).base.netif = esp_netif;
    let driver_ifconfig = esp_netif_driver_ifconfig_t {
        handle: driver.cast(),
        transmit: Some(openmac_netif_transmit),
        transmit_wrap: Some(openmac_netif_transmit_wrap),
        driver_free_rx_buffer: Some(openmac_free),
    };

    return esp_netif_set_driver_config(esp_netif, &driver_ifconfig as *const _);
}

unsafe extern "C" fn openmac_create_if_driver() -> openmac_netif_driver_t {
    let driver: openmac_netif_driver_t = calloc(1, size_of::<openmac_netif_driver>() as u32).cast();
    if driver.is_null() {
        error!("No memory to create a wifi interface handle");
        return null_mut();
    }
    (&mut *driver).base.post_attach = Some(openmac_driver_start);

    // TODO fix this
    if !receive_task_is_running {
        receive_task_is_running = true;
    }
    return driver;
}

pub unsafe extern "C" fn openmac_netif_start() -> esp_err_t {
    let mut base_cfg: esp_netif_inherent_config_t = ESP_NETIF_INHERENT_DEFAULT_WIFI_STA();
    base_cfg.if_desc = "openmac".as_ptr().cast();
    // base_cfg.get_ip_event = NULL;
    // base_cfg.lost_ip_event = NULL;

    let cfg = esp_netif_config_t {
        base: &base_cfg,
        driver: null_mut(),
        stack: ESP_NETIF_NETSTACK_DEFAULT_WIFI_STA(),
    };
    netif_openmac = esp_netif_new(&cfg);
    assert!(!netif_openmac.is_null());

    let driver: openmac_netif_driver_t = openmac_create_if_driver();
    if driver == null_mut() {
        error!("Failed to create wifi interface handle");
        return ESP_FAIL;
    }
    esp_netif_attach(netif_openmac, driver.cast());
    esp_netif_set_hostname(netif_openmac, "esp32-open-mac".as_ptr().cast());
    esp_netif_set_mac(netif_openmac, module_mac_addr.as_mut_ptr());
    esp_netif_action_start(netif_openmac.cast(), null_mut(), 0, null_mut());
    return ESP_OK;
}

fn ESP_NETIF_INHERENT_DEFAULT_WIFI_STA() -> esp_netif_inherent_config_t {
    esp_netif_inherent_config_t {
        flags: (esp_netif_flags_ESP_NETIF_DHCP_CLIENT
            | esp_netif_flags_ESP_NETIF_FLAG_GARP
            | esp_netif_flags_ESP_NETIF_FLAG_MLDV6_REPORT
            | esp_netif_flags_ESP_NETIF_FLAG_EVENT_IP_MODIFIED),
        mac: [0, 0, 0, 0, 0, 0],
        ip_info: null_mut(),
        get_ip_event: ip_event_t_IP_EVENT_STA_GOT_IP,
        lost_ip_event: ip_event_t_IP_EVENT_STA_LOST_IP,
        if_key: "WIFI_STA_DEF".as_ptr().cast(),
        if_desc: "sta".as_ptr().cast(),
        route_prio: 100,
        bridge_info: null_mut(),
    }
}

#![allow(non_camel_case_types)]
#![allow(non_snake_case)]

use core::mem::size_of;
use core::ptr;
use std::mem::MaybeUninit;
use std::sync::atomic::AtomicI32;

use esp_idf_svc::sys::calloc;
use esp_idf_svc::sys::esp_timer_get_time;
use esp_idf_svc::sys::esp_wifi_80211_tx;
use esp_idf_svc::sys::esp_wifi_get_mac;
use esp_idf_svc::sys::esp_wifi_init;
use esp_idf_svc::sys::esp_wifi_set_mode;
use esp_idf_svc::sys::esp_wifi_start;
use esp_idf_svc::sys::malloc;
use esp_idf_svc::sys::memcpy;
use esp_idf_svc::sys::portNUM_PROCESSORS;
use esp_idf_svc::sys::vTaskDelay;
use esp_idf_svc::sys::wifi_init_config_t;
use esp_idf_svc::sys::wifi_interface_t_WIFI_IF_STA;
use esp_idf_svc::sys::wifi_mode_t_WIFI_MODE_STA;
use esp_idf_svc::sys::wifi_promiscuous_pkt_t;
use esp_idf_svc::sys::xQueueReceive;
use esp_idf_svc::sys::xt_ints_on;
use esp_idf_svc::sys::xt_set_interrupt_handler;
use esp_idf_svc::sys::EspError;
use esp_idf_svc::sys::QueueHandle_t;
use esp_idf_svc::sys::SemaphoreHandle_t;
use esp_idf_svc::sys::ESP_OK;
use esp_idf_svc::sys::ETS_WIFI_MAC_INTR_SOURCE;
use esp_idf_svc::sys::ETS_WMAC_INUM;
use log::{debug, error, info, warn};

use crate::c_macro_replacements::intr_matrix_set;
use crate::c_macro_replacements::portTICK_PERIOD_MS;
use crate::c_macro_replacements::xPortGetCoreID;
use crate::c_macro_replacements::WIFI_INIT_CONFIG_DEFAULT;
use crate::proprietary::_xt_interrupt_table;
use crate::proprietary::pp_post;
use crate::proprietary::xt_unhandled_interrupt;
use crate::xQueueCreate;
use crate::xQueueSendFromISR;
use crate::xQueueSendToFront;
use crate::xSemaphoreCreateCounting;
use crate::xSemaphoreGive;
use crate::xSemaphoreTake;
use crate::xSemaphoreTakeFromISR;

const RX_BUFFER_AMOUNT: usize = 10;

const TAG: &str = "hardware.c";
pub const module_mac_addr: [u8; 6] = [0x00, 0x23, 0x45, 0x67, 0x89, 0xab];

#[inline]
unsafe fn write_register(address: *mut u32, value: u32) {
    address.write_volatile(value);
}

#[inline]
unsafe fn read_register(address: *const u32) -> u32 {
    address.read_volatile()
}

const WIFI_DMA_OUTLINK: *mut u32 = 0x3ff73d20 as _;
const WIFI_TX_CONFIG_0: *mut u32 = 0x3ff73d1c as _;

const MAC_TX_PLCP1: *mut u32 = 0x3ff74258 as _;
const MAC_TX_PLCP2: *mut u32 = 0x3ff7425c as _;
const MAC_TX_DURATION: *mut u32 = 0x3ff74268 as _;

const WIFI_DMA_INT_STATUS: *const u32 = 0x3ff73c48 as _;
const WIFI_DMA_INT_CLR: *mut u32 = 0x3ff73c4c as _;

const WIFI_MAC_BITMASK_084: *mut u32 = 0x3ff73084 as _;
const WIFI_NEXT_RX_DSCR: *const u32 = 0x3ff7308c as _;
const WIFI_LAST_RX_DSCR: *mut u32 = 0x3ff73090 as _;
const WIFI_BASE_RX_DSCR: *mut u32 = 0x3ff73088 as _;

const WIFI_MAC_ADDR_SLOT_0: *mut u32 = 0x3ff73040 as _;
const WIFI_MAC_ADDR_ACK_ENABLE_SLOT_0: u32 = 0x3ff73064;
#[repr(C, packed)]
#[derive(Debug, Clone)]
pub struct dma_list_item {
    size: u16,
    length: u16,
    _unknown: u8,
    has_data: u8,
    owner: u8, // What does this mean?
    packet: *mut core::ffi::c_void,
    next: *mut dma_list_item,
}

impl dma_list_item {
    pub unsafe fn new(packet: *mut core::ffi::c_void, next: *mut Self) -> Self {
        Self {
            size: 12,
            length: 12,
            _unknown: 6,
            has_data: 1,
            owner: 1, // What does this mean?
            packet,
            next,
        }
    }
}

#[repr(C)]
#[derive(PartialEq)]
pub enum hardware_queue_entry_type_t {
    RX_ENTRY,
    TX_ENTRY,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct rx_queue_entry_t {
    interrupt_received: u32,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct tx_queue_entry_t {
    packet: *mut u8,
    len: u32,
}

#[repr(C)]
pub union content_t {
    rx: rx_queue_entry_t,
    tx: tx_queue_entry_t,
}

#[repr(C)]
pub struct hardware_queue_entry_t {
    typ: hardware_queue_entry_type_t,
    content: content_t,
}

pub static mut tx_queue_resources: SemaphoreHandle_t = ptr::null_mut();
pub static mut rx_queue_resources: SemaphoreHandle_t = ptr::null_mut();

pub static mut hardware_event_queue: QueueHandle_t = ptr::null_mut();
pub static mut rx_chain_begin: *mut dma_list_item = ptr::null_mut();

pub static mut rx_chain_last: *mut dma_list_item = ptr::null_mut();

pub static interrupt_count: AtomicI32 = AtomicI32::new(0);

// TODO: have more than 1 TX slot
pub static mut tx_item: *mut dma_list_item = ptr::null_mut();

// Must be at least 24 bits long
pub static mut tx_buffer: *mut u8 = ptr::null_mut();

pub static mut last_transmit_timestamp: u64 = 0;
pub static mut seqnum: u32 = 0;

pub unsafe extern "C" fn setup_tx_buffers() {
    tx_item = calloc(1, size_of::<dma_list_item>() as _).cast();
    tx_buffer = calloc(1, 1600 as _) as _;
}

pub unsafe extern "C" fn log_dma_item(item: &dma_list_item) {
    let item = item.clone();
    let length = item.length;
    let size = item.size;
    info!(
        "dma_item, cur={:?} owner={} has_data={} length={} size={} packet={} next={}",
        item, item.owner, item.has_data, length, size, item.packet as usize, item.next as usize
    );
}

pub unsafe extern "C" fn transmit_packet(packet: *mut u8, buffer_len: u32) -> bool {
    // 50 ms for safety, it's likely much shorter
    // TODO: figure out how we can know we can recycle the packet
    if esp_timer_get_time() as u64 - last_transmit_timestamp < 50000 {
        info!("{}, Not transmitting packet, last transmit too recent", TAG);
        return false;
    }

    memcpy(tx_buffer.cast(), packet.cast(), buffer_len);
    let size_len: u32 = buffer_len + 32;

    // Update sequence number
    // This was a simple array access, had to change it to this for rust
    // Return it to an array access once converting to safer code
    tx_buffer.add(22).write_volatile((seqnum as u8 & 0x0f) << 4);
    tx_buffer
        .add(23)
        .write_volatile(((seqnum & 0xff0) >> 4) as u8);
    seqnum += 1;
    if seqnum > 0xfff {
        seqnum = 0
    };

    info!("{}: len={}", TAG, buffer_len);
    info!("to-transmit: {:x?}, {}", tx_buffer, buffer_len);

    tx_item.as_mut().unwrap().owner = 1;
    tx_item.as_mut().unwrap().has_data = 1;
    tx_item.as_mut().unwrap().length = buffer_len as _;
    tx_item.as_mut().unwrap().size = size_len as u16;
    tx_item.as_mut().unwrap().packet = tx_buffer.cast();
    tx_item.as_mut().unwrap().next = ptr::null_mut();

    write_register(WIFI_TX_CONFIG_0, read_register(WIFI_TX_CONFIG_0) | 0xa);

    write_register(WIFI_DMA_OUTLINK, (tx_item as u32 & 0xfffff) | (0x00600000));

    write_register(MAC_TX_PLCP1, 0x10000000 | buffer_len);
    write_register(MAC_TX_PLCP2, 0x00000020);
    write_register(MAC_TX_DURATION, 0);

    write_register(
        WIFI_TX_CONFIG_0,
        read_register(WIFI_TX_CONFIG_0) | 0x02000000,
    );

    write_register(
        WIFI_TX_CONFIG_0,
        read_register(WIFI_TX_CONFIG_0) | 0x00003000,
    );

    // Transmit: setting the 0xc0000000 bit in WIFI_DMA_OUTLINK enables transmission
    write_register(
        WIFI_DMA_OUTLINK,
        read_register(WIFI_DMA_OUTLINK) | 0xc0000000,
    );
    // TODO: instead of sleeping, figure out how to know that our packet was sent
    last_transmit_timestamp = esp_timer_get_time() as u64;
    true
}

// Should use IRAM_ATTR
// Can't find good documentation on each section but this looks like how its done in the HAL code
#[link_section = ".iram1.interrupt_active"]
pub unsafe extern "C" fn wifi_interrupt_handler(args: *mut core::ffi::c_void) {
    interrupt_count.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
    let cause = read_register(WIFI_DMA_INT_STATUS);
    if cause == 0 {
        return;
    }
    write_register(WIFI_DMA_INT_CLR, cause);

    if cause & 0x800 != 0 {
        // TODO handle this with open-source code
        // wdev_process_panic_watchdog() is the closed-source way to recover from this
    }
    let mut tmp: i32 = 0;
    if xSemaphoreTakeFromISR!(rx_queue_resources, &mut tmp as *mut i32) != 0 {
        let queue_entry: hardware_queue_entry_t = hardware_queue_entry_t {
            typ: hardware_queue_entry_type_t::RX_ENTRY,
            content: content_t {
                rx: rx_queue_entry_t {
                    interrupt_received: cause,
                },
            },
        };
        xQueueSendFromISR!(
            hardware_event_queue,
            (&queue_entry as *const hardware_queue_entry_t).cast(),
            ptr::null_mut()
        );
    }
}

pub unsafe fn setup_interrupt() {
    // See the documentation of intr_matrix_set in esp-idf/components/esp_rom/include/esp32s3/rom/ets_sys.h
    intr_matrix_set(0, ETS_WIFI_MAC_INTR_SOURCE, ETS_WMAC_INUM);

    // Wait for interrupt to be set, so we can replace it
    while _xt_interrupt_table
        [(ETS_WMAC_INUM * portNUM_PROCESSORS + xPortGetCoreID() as u32) as usize]
        .handler
        == (xt_unhandled_interrupt as *const fn(*const core::ffi::c_void) -> core::ffi::c_void)
            .cast()
    {
        vTaskDelay(100 / portTICK_PERIOD_MS);
        warn!("{}, Waiting for interrupt to become set", TAG);
    }

    // Replace the existing wDev_ProcessFiq interrupt
    xt_set_interrupt_handler(
        ETS_WMAC_INUM as i32,
        Some(wifi_interrupt_handler),
        ptr::null_mut(),
    );
    xt_ints_on(1 << ETS_WMAC_INUM);
}

pub unsafe extern "C" fn print_rx_chain(mut item: *mut dma_list_item) {
    // Debug print to display RX linked list
    let mut index: core::ffi::c_int = 0;
    debug!(
        "rx-chain, base={} next={} last={}",
        read_register(WIFI_BASE_RX_DSCR),
        read_register(WIFI_NEXT_RX_DSCR),
        read_register(WIFI_LAST_RX_DSCR)
    );
    while !item.is_null() {
        // We unpack it for debugging
        // Accessing non-aligned fields is UB so we must assign to variables first
        let dma_list_item {
            size,
            length,
            _unknown,
            owner,
            has_data,
            packet,
            next,
        } = item.as_mut().unwrap().clone();
        debug!(
            "rx-chain, idx={} cur={:?} owner={} has_data={} length={} size={} packet={} next={}",
            index, item, owner, has_data, length, size, packet as usize, next as usize
        );
        item = item.as_mut().unwrap().next;
        index += 1;
    }
    debug!(
        "rx-chain, base={} next={} last={}",
        read_register(WIFI_BASE_RX_DSCR),
        read_register(WIFI_NEXT_RX_DSCR),
        read_register(WIFI_LAST_RX_DSCR)
    );
}

pub unsafe extern "C" fn set_rx_base_address(item: *mut dma_list_item) {
    write_register(WIFI_BASE_RX_DSCR, item as u32);
}

pub unsafe extern "C" fn setup_rx_chain() {
    // This function sets up the linked list needed for the Wi-Fi MAC RX functionality
    let mut prev: *mut dma_list_item = ptr::null_mut();
    for i in 0..RX_BUFFER_AMOUNT {
        let item: *mut dma_list_item = malloc(size_of::<dma_list_item> as _).cast();
        item.as_mut().unwrap().has_data = 0;
        item.as_mut().unwrap().owner = 1;
        item.as_mut().unwrap().size = 1600;
        item.as_mut().unwrap().length = item.as_mut().unwrap().size;

        let mut packet: *mut u8 = malloc(1600) as _; // TODO verify that this does not need to be bigger
        item.as_mut().unwrap().packet = packet.cast();
        item.as_mut().unwrap().next = prev;
        prev = item;
        if rx_chain_last.is_null() {
            rx_chain_last = item;
        }
    }
    set_rx_base_address(prev);
    rx_chain_begin = prev;
}

pub unsafe extern "C" fn update_rx_chain() {
    write_register(
        WIFI_MAC_BITMASK_084,
        read_register(WIFI_MAC_BITMASK_084) | 0x1,
    );
    // Wait for confirmation from hardware
    while read_register(WIFI_MAC_BITMASK_084) & 0x1 != 0 {}
}

pub unsafe extern "C" fn handle_rx_messages(rxcb: rx_callback) {
    let mut current: *mut dma_list_item = rx_chain_begin;

    // This is a workaround for when we receive a lot of packets; otherwise we get stuck in this function,
    // handling packets for all eternity
    // This is much less of a problem now that we implement hardware filtering
    let mut received: core::ffi::c_int = 0;
    while !current.is_null() {
        let mut next: *mut dma_list_item = current.as_mut().unwrap().next;
        if current.as_mut().unwrap().has_data != 0 {
            //TODO enable interrupt

            received += 1;
            // Has data, but actual 802.11 MAC frame only starts at 28 bytes into the packet
            // The data before contains packet metadata
            let mut packet: *mut wifi_promiscuous_pkt_t = current.as_mut().unwrap().packet as _;
            // packet->rx_ctrl.sig_len includes the FCS (4 bytes), but we don't need this

            // call callback of upper layer
            rxcb(packet);
            // Recycle DMA item and buffer
            rx_chain_begin = current.as_mut().unwrap().next;
            current.as_mut().unwrap().next = ptr::null_mut();
            current.as_mut().unwrap().length = current.as_mut().unwrap().size;
            current.as_mut().unwrap().has_data = 0;

            // This puts the DMA buffer back in the linked list
            // TODO: this code looks pretty ugly and might not be optimal
            if !rx_chain_begin.is_null() {
                rx_chain_last.as_mut().unwrap().next = current;
                update_rx_chain();
                if read_register(WIFI_NEXT_RX_DSCR) == 0x3ff00000 {
                    let mut last_dscr: *mut dma_list_item =
                        read_register(WIFI_LAST_RX_DSCR) as *mut dma_list_item;
                    if current == last_dscr {
                        rx_chain_last = current;
                    } else {
                        set_rx_base_address(last_dscr.as_mut().unwrap().next);
                        rx_chain_last = current;
                    }
                } else {
                    rx_chain_last = current;
                }
            } else {
                rx_chain_begin = current;
                set_rx_base_address(current);
                rx_chain_last = current;
            }
            //TODO disable interrupt
        }
        current = next;
        if received > 10 {
            // Break should do the exact same thing here but leaving it commented out just in case
            // goto out;
            break;
        }
    }
    // TODO enable interrupt
}

pub unsafe extern "C" fn wifi_hardware_tx_func(packet: *mut u8, len: u32) -> bool {
    if xSemaphoreTake!(tx_queue_resources, 1) == 0 {
        error!("{}, TX semaphore full!", TAG);
        return false;
    }
    let mut queue_copy: *mut u8 = malloc(len) as _;
    memcpy(queue_copy.cast(), packet.cast(), len);
    let mut queue_entry = hardware_queue_entry_t {
        typ: hardware_queue_entry_type_t::TX_ENTRY,
        content: content_t {
            tx: tx_queue_entry_t {
                len,
                packet: queue_copy,
            },
        },
    };
    xQueueSendToFront!(
        hardware_event_queue,
        (&mut queue_entry as *mut hardware_queue_entry_t).cast(),
        0
    );
    info!("{}, TX entry queued", TAG);
    return true;
}

pub unsafe extern "C" fn set_enable_mac_addr_filter(slot: u8, enable: bool) {
    // This will allow packets that match the filter to be queued in our reception queue
    // will also ack them once they arrive
    assert!(slot <= 1);
    let addr: u32 = WIFI_MAC_ADDR_ACK_ENABLE_SLOT_0 + 8 * slot as u32;
    let addr = addr as *mut u32;
    if enable {
        write_register(addr, read_register(addr) | 0x10000);
    } else {
        // TODO: ensure switching ~ for ! is correct
        write_register(addr, read_register(addr) & !0x10000);
    }
}

pub unsafe extern "C" fn set_mac_addr_filter(slot: u8, addr: *mut u8) {
    assert!(slot <= 1);
    write_register(
        // This feels like I'm doing something wrong
        WIFI_MAC_ADDR_SLOT_0.add(slot as usize * 8),
        addr.read_volatile() as u32
            | (addr.add(1).read_volatile() as u32) << 8
            | (addr.add(2).read_volatile() as u32) << 16
            | (addr.add(3).read_volatile() as u32) << 24,
    );
    write_register(
        WIFI_MAC_ADDR_SLOT_0.add(slot as usize * 8 + 4),
        addr.add(4) as u32 | (addr.add(5).read_volatile() as u32) << 8,
    );
    write_register(
        WIFI_MAC_ADDR_SLOT_0.add(slot as usize * 8 + 8 * 4) as *mut u32,
        !0,
    ); // ?
}

pub unsafe extern "C" fn wifi_hardware_task(pvParameter: *mut hardware_mac_args) {
    let mut cfg: wifi_init_config_t = WIFI_INIT_CONFIG_DEFAULT();
    cfg.static_rx_buf_num = 2; // we won't use these buffers, so reduce the amount from default 10, so we don't waste as much memory
                               // Disable AMPDU and AMSDU for now, we don't support this (yet)
    cfg.ampdu_rx_enable = 0;
    cfg.ampdu_tx_enable = 0;
    cfg.amsdu_tx_enable = 0;
    cfg.nvs_enable = 0;

    // Print MAC addresses
    for i in 0..2 {
        let mut mac: [u8; 6] = [0; 6];
        if esp_wifi_get_mac(i, mac.as_mut_ptr()) == ESP_OK {
            warn!(
                "{}, MAC {} = {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
                TAG, i, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]
            );
        }
    }

    hardware_event_queue = xQueueCreate!(
        RX_BUFFER_AMOUNT as u32 + 10,
        size_of::<hardware_queue_entry_t>() as u32
    );
    assert!(!hardware_event_queue.is_null());
    rx_queue_resources = xSemaphoreCreateCounting!(RX_BUFFER_AMOUNT as _, RX_BUFFER_AMOUNT as _);
    assert!(!rx_queue_resources.is_null());
    tx_queue_resources = xSemaphoreCreateCounting!(10, 10);
    assert!(!tx_queue_resources.is_null());

    warn!("{}, calling esp_wifi_init", TAG);
    // Panick if error
    EspError::from(esp_wifi_init(&cfg)).map(|x| x.panic());
    warn!("{}, done esp_wifi_init", TAG);

    warn!(
        "{}, Starting wifi_hardware task, running on {}",
        TAG,
        xPortGetCoreID()
    );
    warn!("{}, calling esp_wifi_set_mode", TAG);
    EspError::from(esp_wifi_set_mode(wifi_mode_t_WIFI_MODE_STA)).map(|x| x.panic());
    warn!("{}, done esp_wifi_set_mode", TAG);

    warn!("{}, calling esp_wifi_start", TAG);
    EspError::from(esp_wifi_start()).map(|x| x.panic());
    warn!("{}, done esp_wifi_start", TAG);

    static mut initframe: [u8; 24] = [
        0x08, 0x01, 0x00, 0x00, // data frame
        0x4e, 0xed, 0xfb, 0x35, 0x22, 0xa8, // receiver addr
        0x00, 0x23, 0x45, 0x67, 0x89, 0xab, // transmitter
        0x84, 0x2b, 0x2b, 0x4f, 0x89, 0x4f, // destination
        0x00, 0x00, // sequence control
    ];

    // Send a packet, to make sure the proprietary stack has fully initialized all hardware
    // Original code had no abort but will just do this for now
    EspError::from(esp_wifi_80211_tx(
        wifi_interface_t_WIFI_IF_STA,
        initframe.as_mut_ptr().cast(),
        initframe.len() as i32,
        true,
    ))
    .map(|x| x.panic());

    // From here, we start taking over the hardware; no more proprietary code is executed from now on
    setup_interrupt();

    // ppTask is a FreeRTOS task included in the esp32-wifi-lib blob
    // It reads from a queue that the proprietary WMAC interrupt handler writes to
    // We kill it to make sure that no proprietary code is running anymore
    warn!("{}: Killing proprietary wifi task (ppTask)", TAG);
    pp_post(0xf, 0);

    setup_rx_chain();
    setup_tx_buffers();

    unsafe {
        (pvParameter.as_mut().unwrap()._tx_func_callback)(wifi_hardware_tx_func);
    }
    warn!("{}: Starting to receive messages", TAG);

    set_mac_addr_filter(0, module_mac_addr.as_mut_ptr());
    set_enable_mac_addr_filter(0, true);
    // acking will only happen if the hardware puts the packet in an RX buffer

    let first_part: u32 = read_register(WIFI_MAC_ADDR_SLOT_0.add(4));
    warn!(
        "{}: addr_p = {:x} {:x}",
        TAG,
        first_part & 0xff,
        (first_part >> 8) & 0xff
    );

    loop {
        let mut queue_entry: MaybeUninit<hardware_queue_entry_t> = MaybeUninit::zeroed();
        if xQueueReceive(hardware_event_queue, queue_entry.as_mut_ptr().cast(), 10) != 0 {
            let mut queue_entry = queue_entry.assume_init();
            if queue_entry.typ == hardware_queue_entry_type_t::RX_ENTRY {
                let cause: u32 = queue_entry.content.rx.interrupt_received;
                // ESP_LOGW(TAG, "interrupt = 0x%08lx", cause);
                if cause & 0x800 != 0 {
                    // Watchdog panic
                    // TODO process this
                    // TODO what pets this watchdog?
                    // ESP_LOGW(TAG, "watchdog panic, how do we pet it?");
                }
                if cause & 0x600000 != 0 {
                    // TODO this is bad, we should reboot
                    error!("{}: something bad, we should reboot", TAG);
                }
                if cause & 0x1000024 != 0 {
                    // ESP_LOGW(TAG, "received message");
                    handle_rx_messages(pvParameter.as_mut().unwrap()._rx_callback);
                }
                if cause & 0x80 != 0 {
                    // ESP_LOGW(TAG, "lmacPostTxComplete");
                }
                if cause & 0x80000 != 0 {
                    // ESP_LOGW(TAG, "lmacProcessAllTxTimeout");
                }
                if cause & 0x100 != 0 {
                    // ESP_LOGW(TAG, "lmacProcessCollisions");
                }
                xSemaphoreGive!(rx_queue_resources);
            } else if queue_entry.typ == hardware_queue_entry_type_t::TX_ENTRY {
                error!("{}: TX from queue", TAG);
                // TODO: implement retry
                // (we might not actually need it, but how do we know a packet has been transmitted and we can recycle its content)
                transmit_packet(queue_entry.content.tx.packet, queue_entry.content.tx.len);
                esp_idf_svc::sys::free(queue_entry.content.tx.packet as _);
                xSemaphoreGive!(tx_queue_resources);
            } else {
                error!("{}: unknown queue type", TAG);
            }
        }
        // ESP_LOGW(TAG, "interrupt count=%d", interrupt_count);
    }
}

pub type rx_callback = unsafe extern "C" fn(packet: *mut wifi_promiscuous_pkt_t);
pub type tx_func = unsafe extern "C" fn(packet: *mut u8, len: u32) -> bool;

pub type tx_func_callback = unsafe extern "C" fn(t: tx_func);

#[repr(C)]
pub struct hardware_mac_args {
    pub _rx_callback: rx_callback,
    pub _tx_func_callback: tx_func_callback,
}

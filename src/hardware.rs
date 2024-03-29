#![allow(non_camel_case_types)]
#![allow(non_snake_case)]

use core::mem::size_of;
use core::ptr;
use std::mem::MaybeUninit;
use std::ptr::null_mut;
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
use vcell::VolatileCell;

use crate::c_macro_replacements::intr_matrix_set;
use crate::c_macro_replacements::portTICK_PERIOD_MS;
use crate::c_macro_replacements::xPortGetCoreID;
use crate::c_macro_replacements::WIFI_INIT_CONFIG_DEFAULT;
use crate::proprietary::_xt_interrupt_table;
use crate::proprietary::pp_post;
use crate::proprietary::xt_unhandled_interrupt;
use crate::utils::Register;
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

// TODO: Remove theses
// -#define WIFI_DMA_OUTLINK 0x3ff73d20
//-#define WIFI_TX_CONFIG_0 0x3ff73d1c

// TODO: Remove these constants
// -#define MAC_TX_PLCP1 0x3ff74258
// -#define MAC_TX_PLCP2 0x3ff7425c
// -#define MAC_TX_DURATION 0x3ff74268
// there are 5 TX slots
// format: _BASE addresses are the base addresses
//         _OS amounts is the amount of 4-byte words in the offset between slots
// So for example, if the MAC_TX_PLCP0 for slot 0 is at 0x3ff73d20
// then the MAC_TX_PLCP0 for slot 1 will be at 0x3ff73d20 - 2 * 4 = 0x3ff73d18

const MAC_TX_PLCP0_BASE: Register<u32> = Register::at(0x3ff73d20);
const MAC_TX_PLCP0_OS: i32 = -2;

const WIFI_TX_CONFIG_BASE: Register<u32> = Register::at(0x3ff73d1c);
const WIFI_TX_CONFIG_OS: i32 = -2;

const MAC_TX_PLCP1_BASE: Register<u32> = Register::at(0x3ff74258);
const MAC_TX_PLCP1_OS: i32 = -0xf;

const MAC_TX_PLCP2_BASE: Register<u32> = Register::at(0x3ff7425c);
const MAC_TX_PLCP2_OS: i32 = -0xf;

const MAC_TX_DURATION_BASE: Register<u32> = Register::at(0x3ff74268);
const MAC_TX_DURATION_OS: i32 = -0xf;

const WIFI_DMA_OUTLINK: Register<u32> = Register::at(0x3ff73d20);
const WIFI_TX_CONFIG_0: Register<u32> = Register::at(0x3ff73d1c);

const MAC_TX_PLCP1: Register<u32> = Register::at(0x3ff74258);
const MAC_TX_PLCP2: Register<u32> = Register::at(0x3ff7425c);
const MAC_TX_DURATION: Register<u32> = Register::at(0x3ff74268);

const WIFI_DMA_INT_STATUS: Register<u32> = Register::at(0x3ff73c48);
const WIFI_DMA_INT_CLR: Register<u32> = Register::at(0x3ff73c4c);

const WIFI_MAC_BITMASK_084: Register<u32> = Register::at(0x3ff73084 as _);
const WIFI_NEXT_RX_DSCR: Register<u32> = Register::at(0x3ff7308c);
const WIFI_LAST_RX_DSCR: Register<u32> = Register::at(0x3ff73090);
const WIFI_BASE_RX_DSCR: Register<u32> = Register::at(0x3ff73088);
const WIFI_TXQ_GET_STATE_COMPLETE: Register<u32> = Register::at(0x3ff73cc8);
const WIFI_TXQ_CLR_STATE_COMPLETE: Register<u32> = Register::at(0x3ff73cc4);

// Collision or timeout
// These were in the original code but both aren't referenced anywhere and are larger than a u32, u32 is the size of a pointer on this platform and so these are likely invalid memory addresses
// const WIFI_TXQ_GET_STATE_ERROR: Register<u32> = Register::at(0x3ff73ccc0);
// const WIFI_TXQ_CLR_STATE_ERROR: Register<u32> = Register::at(0x3ff73ccbc);

const WIFI_MAC_ADDR_SLOT_0: Register<u32> = Register::at(0x3ff73040);
const WIFI_MAC_ADDR_ACK_ENABLE_SLOT_0: Register<u32> = Register::at(0x3ff73064);

#[repr(C, packed)]
#[derive(Debug, Clone)]
pub struct dma_list_item {
    size: u16,
    length: u16,
    _unknown: u8,
    has_data: u8,
    owner: u8, // What does this mean?
    // TODO: Feel like this property should just be typed as *mut u8
    packet: *mut core::ffi::c_void,
    next: *mut dma_list_item,
}
impl dma_list_item {
    const fn zeroed() -> Self {
        Self {
            size: 0,
            length: 0,
            _unknown: 0,
            has_data: 0,
            owner: 0,
            packet: null_mut(),
            next: null_mut(),
        }
    }
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
impl tx_queue_entry_t {
    const fn zeroed() -> Self {
        Self {
            packet: null_mut(),
            len: 0,
        }
    }
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

const TX_SLOT_CNT: usize = 5;
#[repr(C, align(4))]
struct tx_hardware_slot_t {
    // dma_list_item must be 4-byte aligned (it's passed to hardware that only takes those addresses)
    _alignment: [u8; 0],
    // This is meant to have alignment of 4 but rust-analyzer tells me its got alignment 1
    // TODO: investigate further
    dma: dma_list_item,

    packet: tx_queue_entry_t,
    in_use: bool,
}
impl tx_hardware_slot_t {
    const fn zeroed() -> Self {
        tx_hardware_slot_t {
            // dma_list_item must be 4-byte aligned (it's passed to hardware that only takes those addresses)
            _alignment: [],
            // This is meant to have alignment of 4 but rust-analyzer tells me its got alignment 1
            // TODO: investigate further
            dma: dma_list_item::zeroed(),

            packet: tx_queue_entry_t::zeroed(),
            in_use: false,
        }
    }
}
static mut tx_slots: [tx_hardware_slot_t; TX_SLOT_CNT as usize] = [
    tx_hardware_slot_t::zeroed(),
    tx_hardware_slot_t::zeroed(),
    tx_hardware_slot_t::zeroed(),
    tx_hardware_slot_t::zeroed(),
    tx_hardware_slot_t::zeroed(),
];

pub static mut last_transmit_timestamp: u64 = 0;
pub static mut seqnum: u32 = 0;

pub unsafe extern "C" fn log_dma_item(item: &dma_list_item) {
    let item = item.clone();
    let length = item.length;
    let size = item.size;
    info!(
        "dma_item, cur={:?} owner={} has_data={} length={} size={} packet={} next={}",
        item, item.owner, item.has_data, length, size, item.packet as usize, item.next as usize
    );
}

// Should we ensure this function can only be called once at a time?
// It appears as though there might be room for the buffers to over write each other if the function is called twice at the same time
pub unsafe extern "C" fn transmit_packet(tx_buffer: *mut u8, buffer_len: u32) -> bool {
    let mut slot: u32 = 0;

    // Find the first free TX slot
    let slot = tx_slots.iter_mut().position(|x| x.in_use);
    let Some(slot) = slot else {
        error!("all tx slots full");
        return false;
    };
    info!("using tx slot {}", slot);

    let mut tx_item = &mut tx_slots[slot].dma;
    // dma_list_item must be 4-byte aligned (it's passed to hardware that only takes those addresses)
    assert!((tx_item as *mut dma_list_item) as usize & 0b11 == 0);

    tx_slots[slot].in_use = true;
    tx_slots[slot].packet.packet = tx_buffer;
    tx_slots[slot].packet.len = buffer_len;
    let size_len: u32 = buffer_len + 32;

    // Set & update sequence number
    tx_slots[slot]
        .packet
        .packet
        .add(22)
        .write(((seqnum & 0x0f) << 4) as u8);
    tx_slots[slot]
        .packet
        .packet
        .add(23)
        .write(((seqnum & 0xff0) >> 4) as u8);
    seqnum += 1;
    if seqnum > 0xfff {
        seqnum = 0
    }

    info!("len={}", buffer_len);
    info!("Replace this with hex dump of the packet");

    tx_item.owner = 1;
    tx_item.has_data = 1;
    tx_item.length = buffer_len as u16;
    tx_item.size = size_len as u16;
    // TODO: Feel like this property should just be typed as *mut u8
    tx_item.packet = tx_buffer.cast();
    tx_item.next = null_mut();

    WIFI_TX_CONFIG_BASE
        .offset(WIFI_TX_CONFIG_OS as isize * slot as isize)
        .set(
            WIFI_TX_CONFIG_BASE
                .offset(WIFI_TX_CONFIG_OS as isize * slot as isize)
                .read()
                | 0xa,
        );

    MAC_TX_PLCP0_BASE
        .offset(MAC_TX_PLCP0_OS as isize * slot as isize)
        .set(((tx_item as *const dma_list_item) as u32 & 0xfffff) | (0x00600000));
    MAC_TX_PLCP1_BASE
        .offset(MAC_TX_PLCP1_OS as isize * slot as isize)
        .set(0x10000000 | buffer_len);
    MAC_TX_PLCP2_BASE
        .offset(MAC_TX_PLCP2_OS as isize * slot as isize)
        .set(0x00000020);
    MAC_TX_DURATION_BASE
        .offset(MAC_TX_DURATION_OS as isize * slot as isize)
        .set(0);

    WIFI_TX_CONFIG_BASE
        .offset(WIFI_TX_CONFIG_OS as isize * slot as isize)
        .set(
            WIFI_TX_CONFIG_BASE
                .offset(WIFI_TX_CONFIG_OS as isize * slot as isize)
                .read()
                | 0x02000000,
        );
    WIFI_TX_CONFIG_BASE
        .offset(WIFI_TX_CONFIG_OS as isize * slot as isize)
        .set(
            WIFI_TX_CONFIG_BASE
                .offset(WIFI_TX_CONFIG_OS as isize * slot as isize)
                .read()
                | 0x00003000,
        );

    // Transmit: setting the 0xc0000000 bit in MAC_TX_PLCP0 enables transmission
    MAC_TX_PLCP0_BASE
        .offset(MAC_TX_PLCP0_OS as isize * slot as isize)
        .set(
            MAC_TX_PLCP0_BASE
                .offset(MAC_TX_PLCP0_OS as isize * slot as isize)
                .read()
                | 0xc0000000,
        );
    true
}

unsafe fn processTxComplete() {
    let txq_state_complete = WIFI_TXQ_GET_STATE_COMPLETE.read();
    warn!("tx complete = {}", txq_state_complete);
    if txq_state_complete == 0 {
        return;
    }
    let slot: usize = 31 - txq_state_complete.leading_zeros() as usize;
    warn!("slot {} is now free again", slot);
    let clear_mask: u32 = 1 << slot;
    WIFI_TXQ_CLR_STATE_COMPLETE.set(WIFI_TXQ_CLR_STATE_COMPLETE.read() | clear_mask);
    if slot < TX_SLOT_CNT {
        tx_slots[slot].in_use = false;
        esp_idf_svc::sys::free(tx_slots[slot].packet.packet.cast());
        tx_slots[slot].packet.packet = null_mut();
    }
}

// Copies packet content to internal buffer, so you can free `packet` immediately after calling this function
// Should use IRAM_ATTR
// Can't find good documentation on each section but this looks like how its done in the HAL code
#[link_section = ".iram1.interrupt_active"]
pub unsafe extern "C" fn wifi_interrupt_handler(args: *mut core::ffi::c_void) {
    interrupt_count.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
    let cause = WIFI_DMA_INT_STATUS.read();
    if cause == 0 {
        return;
    }
    WIFI_DMA_INT_CLR.set(cause);

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
        WIFI_BASE_RX_DSCR.read(),
        WIFI_NEXT_RX_DSCR.read(),
        WIFI_LAST_RX_DSCR.read()
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
        WIFI_BASE_RX_DSCR.read(),
        WIFI_NEXT_RX_DSCR.read(),
        WIFI_LAST_RX_DSCR.read()
    );
}

pub unsafe extern "C" fn set_rx_base_address(item: *mut dma_list_item) {
    // TODO: Should I change the type of this register?
    WIFI_BASE_RX_DSCR.set(item as u32);
}

pub unsafe extern "C" fn setup_rx_chain() {
    // This function sets up the linked list needed for the Wi-Fi MAC RX functionality
    let mut prev: *mut dma_list_item = ptr::null_mut();
    for i in 0..RX_BUFFER_AMOUNT {
        // Malloc fails for this for some reason but works perfectly fine later
        let mut item = Box::new(dma_list_item::zeroed());
        info!("after box");
        item.has_data = 0;
        item.owner = 1;
        item.size = 1600;
        item.length = item.size;

        let mut packet: *mut u8 = malloc(1600) as _; // TODO verify that this does not need to be bigger
        item.packet = packet.cast();
        item.next = prev;
        prev = item.as_mut() as *mut dma_list_item;
        if rx_chain_last.is_null() {
            rx_chain_last = item.as_mut() as _;
        }
    }
    info!["After for loop"];
    set_rx_base_address(prev);
    rx_chain_begin = prev;
}

pub unsafe extern "C" fn update_rx_chain() {
    WIFI_MAC_BITMASK_084.xor_with(0x1);
    // Wait for confirmation from hardware
    while WIFI_MAC_BITMASK_084.read() & 0x1 != 0 {}
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
                if WIFI_NEXT_RX_DSCR.read() == 0x3ff00000 {
                    let mut last_dscr: *mut dma_list_item =
                        WIFI_LAST_RX_DSCR.read() as *mut dma_list_item;
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
    let addr = WIFI_MAC_ADDR_ACK_ENABLE_SLOT_0.offset(8 * slot as isize);
    if enable {
        addr.xor_with(0x10000);
    } else {
        // TODO: ensure switching ~ for ! is correct
        addr.and_with(!0x10000);
    }
}

pub unsafe extern "C" fn set_mac_addr_filter(slot: u8, addr: *const u8) {
    assert!(slot <= 1);
    // This feels like I'm doing something wrong
    WIFI_MAC_ADDR_SLOT_0.offset(slot as isize * 8).set(
        addr.read_volatile() as u32
            | (addr.add(1).read_volatile() as u32) << 8
            | (addr.add(2).read_volatile() as u32) << 16
            | (addr.add(3).read_volatile() as u32) << 24,
    );
    WIFI_MAC_ADDR_SLOT_0
        .offset(slot as isize * 8 + 4)
        .set(addr.add(4).read() as u32 | (addr.add(5).read_volatile() as u32) << 8);
    WIFI_MAC_ADDR_SLOT_0
        .offset(slot as isize * 8 + 8 * 4)
        .set(!0); // ?
}

pub unsafe extern "C" fn wifi_hardware_task(pvParameter: *mut core::ffi::c_void) {
    let pvParameter: *mut hardware_mac_args = pvParameter.cast();
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

    static mut initframe: [u8; 37] = [
        0x08, 0x01, 0x00, 0x00, // data frame
        0x4e, 0xed, 0xfb, 0x35, 0x22, 0xa8, // receiver addr
        0x00, 0x23, 0x45, 0x67, 0x89, 0xab, // transmitter
        0x84, 0x2b, 0x2b, 0x4f, 0x89, 0x4f, // destination
        0x00, 0x00, // sequence control
        0xff, 0x00, 0x00, 0x00, // IEEE 802.2
        b'i', b'n', b'i', b't', b'f', b'r', b'a', b'm', b'e',
    ];

    // Send a packet, to make sure the proprietary stack has fully initialized all hardware
    // Original code had no abort but will just do this for now
    info!("Sending test packet on proprietary stack");
    EspError::from(esp_wifi_80211_tx(
        wifi_interface_t_WIFI_IF_STA,
        initframe.as_mut_ptr().cast(),
        initframe.len() as i32,
        true,
    ))
    .map(|x| x.panic());
    info!("Done sending test packet on proprietary stack");

    // From here, we start taking over the hardware; no more proprietary code is executed from now on
    setup_interrupt();

    // ppTask is a FreeRTOS task included in the esp32-wifi-lib blob
    // It reads from a queue that the proprietary WMAC interrupt handler writes to
    // We kill it to make sure that no proprietary code is running anymore
    warn!("{}: Killing proprietary wifi task (ppTask)", TAG);
    pp_post(0xf, 0);

    setup_rx_chain();
    info!["RX chain is set up"];

    unsafe {
        (pvParameter.read()._tx_func_callback)(wifi_hardware_tx_func);
    }
    warn!("Starting to receive messages");

    set_mac_addr_filter(0, module_mac_addr.as_ptr());
    set_enable_mac_addr_filter(0, true);
    // acking will only happen if the hardware puts the packet in an RX buffer

    let first_part: u32 = WIFI_MAC_ADDR_SLOT_0.offset(4).read();
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
                    processTxComplete();
                }
                if cause & 0x80000 != 0 {
                    warn!("lmacProcessAllTxTimeout");
                }
                if cause & 0x100 != 0 {
                    warn!("lmacProcessCollisions");
                }
                xSemaphoreGive!(rx_queue_resources);
            } else if queue_entry.typ == hardware_queue_entry_type_t::TX_ENTRY {
                info!("{}: TX from queue", TAG);
                // TODO: implement retry
                transmit_packet(queue_entry.content.tx.packet, queue_entry.content.tx.len);
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

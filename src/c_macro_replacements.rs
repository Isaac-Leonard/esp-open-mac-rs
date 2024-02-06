use std::arch::asm;

use esp_idf_svc::sys::{
    _g_esp_netif_netstack_default_wifi_sta, configTICK_RATE_HZ, esp_netif_netstack_config_t,
    g_wifi_default_wpa_crypto_funcs, g_wifi_osi_funcs, wifi_init_config_t, BaseType_t, TickType_t,
    CONFIG_ESP_WIFI_DYNAMIC_RX_BUFFER_NUM, CONFIG_ESP_WIFI_DYNAMIC_RX_MGMT_BUF,
    CONFIG_ESP_WIFI_ENABLE_WPA3_SAE, CONFIG_ESP_WIFI_ESPNOW_MAX_ENCRYPT_NUM,
    CONFIG_ESP_WIFI_STATIC_RX_BUFFER_NUM, CONFIG_ESP_WIFI_TX_BUFFER_TYPE, WIFI_AMPDU_RX_ENABLED,
    WIFI_AMPDU_TX_ENABLED, WIFI_AMSDU_TX_ENABLED, WIFI_CACHE_TX_BUFFER_NUM, WIFI_CSI_ENABLED,
    WIFI_DEFAULT_RX_BA_WIN, WIFI_DYNAMIC_TX_BUFFER_NUM, WIFI_INIT_CONFIG_MAGIC, WIFI_MGMT_SBUF_NUM,
    WIFI_NANO_FORMAT_ENABLED, WIFI_NVS_ENABLED, WIFI_RX_MGMT_BUF_NUM_DEF,
    WIFI_SOFTAP_BEACON_MAX_LEN, WIFI_STATIC_TX_BUFFER_NUM, WIFI_STA_DISCONNECTED_PM_ENABLED,
    WIFI_TASK_CORE_ID,
};

pub const queueSEND_TO_BACK: BaseType_t = 0;
pub const queueSEND_TO_FRONT: BaseType_t = 1;
pub const portTICK_PERIOD_MS: TickType_t = 1000 / configTICK_RATE_HZ;
pub const queueQUEUE_TYPE_BASE: u8 = 0;
pub const semGIVE_BLOCK_TIME: TickType_t = 0;

#[macro_export]
macro_rules! xQueueSendFromISR {
    ($xQueue:expr, $pvItemToQueue:expr, $pxHigherPriorityTaskWoken:expr) => {
        esp_idf_svc::sys::xQueueGenericSendFromISR(
            $xQueue,
            $pvItemToQueue,
            $pxHigherPriorityTaskWoken,
            $crate::c_macro_replacements::queueSEND_TO_BACK,
        )
    };
}

// Not really sure why this exists
#[macro_export]
macro_rules! xSemaphoreTake {
    ($xSemaphore:expr, $xBlockTime:expr) => {
        esp_idf_svc::sys::xQueueSemaphoreTake($xSemaphore, $xBlockTime)
    };
}

#[macro_export]
macro_rules! xSemaphoreTakeFromISR {
    ($xSemaphore:expr, $pxHigherPriorityTaskWoken:expr) => {
        esp_idf_svc::sys::xQueueReceiveFromISR(
            $xSemaphore,
            ptr::null_mut(),
            $pxHigherPriorityTaskWoken,
        )
    };
}

#[macro_export]
macro_rules! xQueueSendToFront {
    ($xQueue:expr, $pvItemToQueue:expr, $xTicksToWait:expr) => {
        esp_idf_svc::sys::xQueueGenericSend(
            $xQueue,
            $pvItemToQueue,
            $xTicksToWait,
            $crate::c_macro_replacements::queueSEND_TO_FRONT,
        )
    };
}

#[macro_export]
macro_rules! xQueueCreate {
    ($uxQueueLength:expr, $uxItemSize:expr) => {
        esp_idf_svc::sys::xQueueGenericCreate(
            $uxQueueLength,
            $uxItemSize,
            $crate::c_macro_replacements::queueQUEUE_TYPE_BASE,
        )
    };
}

#[macro_export]
macro_rules! xSemaphoreCreateCounting {
    ($uxMaxCount:expr, $uxInitialCount:expr) => {
        esp_idf_svc::sys::xQueueCreateCountingSemaphore($uxMaxCount, $uxInitialCount)
    };
}

#[macro_export]
macro_rules! xSemaphoreGive {
    ($xSemaphore:expr) => {
        esp_idf_svc::sys::xQueueGenericSend(
            $xSemaphore,
            core::ptr::null_mut(),
            $crate::c_macro_replacements::semGIVE_BLOCK_TIME,
            $crate::c_macro_replacements::queueSEND_TO_BACK,
        )
    };
}

#[macro_export]
macro_rules! MAC2STR {
    ($mac_addr:expr) => {
        format!(
            "{:#02x}:{:#02x}:{:#02x}:{:#02x}:{:#02x}:{:#02x}",
            $mac_addr[0], $mac_addr[1], $mac_addr[2], $mac_addr[3], $mac_addr[4], $mac_addr[5],
        )
    };
}

#[macro_export]
macro_rules! xQueueSendToBack {
    ($xQueue:expr, $pvItemToQueue:expr, $xTicksToWait:expr) => {
        esp_idf_svc::sys::xQueueGenericSend(
            $xQueue,
            $pvItemToQueue,
            $xTicksToWait,
            $crate::c_macro_replacements::queueSEND_TO_BACK,
        )
    };
}

// Looks like the last 3 are not set so going to leave them out for now but leaving the full list documented here
// CONFIG_ESP_WIFI_ENABLE_WPA3_SAE | WIFI_ENABLE_SPIRAM | WIFI_FTM_INITIATOR | WIFI_FTM_RESPONDER;
const WIFI_FEATURE_CAPS: u64 = CONFIG_ESP_WIFI_ENABLE_WPA3_SAE as u64;

extern "C" {
    // TODO: Check this is actually the correct signature
    pub fn intr_matrix_set(
        cpu_no: core::ffi::c_int,
        model_num: u32,
        intr_num: u32,
    ) -> core::ffi::c_void;

    // TODO: Check this is correct
    // I could only find one macro that set this to 0 and lots of different header definitions of this but no implementation
    // Note that some of the definitions had extra modifiers on them, I'm hoping this will just work
    //    pub fn xPortGetCoreID() -> BaseType_t;
}

// Need to check that this assembly actually works
#[inline(always)]
pub unsafe extern "C" fn xPortGetCoreID() -> u32 {
    let mut id: u32 = 0;
    asm! (
        "rsr.prid {id}",
        "extui {id},{id},13,1",
			id=out(reg) id);
    return id;
}

// Can't implement default trait so we just use the original c macros name
pub unsafe fn WIFI_INIT_CONFIG_DEFAULT() -> wifi_init_config_t {
    wifi_init_config_t {
        osi_funcs: unsafe { &mut g_wifi_osi_funcs as *mut _ },
        wpa_crypto_funcs: g_wifi_default_wpa_crypto_funcs,
        static_rx_buf_num: CONFIG_ESP_WIFI_STATIC_RX_BUFFER_NUM as _,
        dynamic_rx_buf_num: CONFIG_ESP_WIFI_DYNAMIC_RX_BUFFER_NUM as _,
        tx_buf_type: CONFIG_ESP_WIFI_TX_BUFFER_TYPE as _,
        static_tx_buf_num: WIFI_STATIC_TX_BUFFER_NUM as _,
        dynamic_tx_buf_num: WIFI_DYNAMIC_TX_BUFFER_NUM as _,
        rx_mgmt_buf_type: CONFIG_ESP_WIFI_DYNAMIC_RX_MGMT_BUF as _,
        rx_mgmt_buf_num: WIFI_RX_MGMT_BUF_NUM_DEF as _,
        cache_tx_buf_num: WIFI_CACHE_TX_BUFFER_NUM as _,
        csi_enable: WIFI_CSI_ENABLED as _,
        ampdu_rx_enable: WIFI_AMPDU_RX_ENABLED as _,
        ampdu_tx_enable: WIFI_AMPDU_TX_ENABLED as _,
        amsdu_tx_enable: WIFI_AMSDU_TX_ENABLED as _,
        nvs_enable: WIFI_NVS_ENABLED as _,
        nano_enable: WIFI_NANO_FORMAT_ENABLED as _,
        rx_ba_win: WIFI_DEFAULT_RX_BA_WIN as _,
        wifi_task_core_id: WIFI_TASK_CORE_ID as _,
        beacon_max_len: WIFI_SOFTAP_BEACON_MAX_LEN as _,
        mgmt_sbuf_num: WIFI_MGMT_SBUF_NUM as _,
        feature_caps: WIFI_FEATURE_CAPS,
        sta_disconnected_pm: WIFI_STA_DISCONNECTED_PM_ENABLED == 1,
        espnow_max_encrypt_num: CONFIG_ESP_WIFI_ESPNOW_MAX_ENCRYPT_NUM as _,
        magic: WIFI_INIT_CONFIG_MAGIC as _,
    }
}

// For some reason bindgen doesn't convert this
pub const BROADCAST_MAC: [u8; 6] = [0xff, 0xff, 0xff, 0xff, 0xff, 0xff];

pub unsafe fn ESP_NETIF_NETSTACK_DEFAULT_WIFI_STA() -> *const esp_netif_netstack_config_t {
    _g_esp_netif_netstack_default_wifi_sta
}

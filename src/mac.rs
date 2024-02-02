#[repr(C)]
enum openmac_sta_state_t{
    INIT,
    IDLE,
    AUTHENTICATED,
    ASSOCIATED,
} ;

unsafe extern "C" fn open_mac_rx_callback(packet:*mut wifi_promiscuous_pkt_t);
unsafe extern "C" fn open_mac_tx_func_callback(t:*mut tx_func);

unsafe extern "C" fn mac_task(pvParameters:*mut core::ffi::c_void);

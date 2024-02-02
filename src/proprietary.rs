use esp_idf_svc::sys::portNUM_PROCESSORS;

extern "C" {
    pub fn pp_post(requestnum: u32, argument: u32) -> bool;

    // Interrupt-related functions
    pub fn xt_unhandled_interrupt(arg: *const core::ffi::c_void) -> core::ffi::c_void;
    pub fn config_get_wifi_task_core_id() -> u32;

    // extern void wdev_process_panic_watchdog();
    pub static _xt_interrupt_table: [xt_handler_table_entry; 32 * portNUM_PROCESSORS as usize];
}

#[repr(C)]
pub struct xt_handler_table_entry {
    pub handler: *const core::ffi::c_void,
    pub arg: *const core::ffi::c_void,
}

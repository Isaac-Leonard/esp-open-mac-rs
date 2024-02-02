This is an open source demonstration of using the esp32 wifi hardware peripherals.
This is pretty much a one to one translation of https://github.com/esp32-open-mac/esp32-open-mac written in rust

I made some minor modifications to implementation code but not in funcctionality.
I also had to reimplement some macros and inline functions from esp_idf, these are located in in src/c_macro_replacements.rs
I intend to extend this and work on building up a custom mac and networking layer on top of it.

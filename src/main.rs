#![deny(unsafe_code)]
#![no_main]
#![no_std]

use cortex_m_semihosting::hprintln;
use panic_semihosting as _;

#[rtfm::app(device = stm32f4xx_hal::stm32, peripherals = true)]
const APP: () = {
    #[init]
    fn init(_: init::Context) {
        hprintln!("Hello, World!").ok();
    }
};

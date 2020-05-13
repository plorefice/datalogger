#![deny(unsafe_code)]
#![no_main]
#![no_std]

use panic_semihosting as _;
use rtfm::cyccnt::U32Ext;
use stm32f4xx_hal::{
    gpio::{gpiod::PD12, GpioExt, Output, PushPull},
    hal::digital::v2::ToggleableOutputPin,
};

#[rtfm::app(device = stm32f4xx_hal::stm32, peripherals = true, monotonic = rtfm::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        heartbeat_led: PD12<Output<PushPull>>,
    }

    #[init(schedule = [heartbeat])]
    fn init(mut cx: init::Context) -> init::LateResources {
        // Initialize (enable) the monotonic timer (CYCCNT)
        cx.core.DWT.enable_cycle_counter();

        let gpiod = cx.device.GPIOD.split();

        let heartbeat_led = gpiod.pd12.into_push_pull_output();

        cx.schedule.heartbeat(cx.start).ok();

        init::LateResources { heartbeat_led }
    }

    #[task(schedule = [heartbeat], resources = [heartbeat_led])]
    fn heartbeat(cx: heartbeat::Context) {
        static mut DELAY_PATTERN: [u32; 4] = [1, 3, 1, 20];
        static mut I: usize = 0;

        cx.schedule
            .heartbeat(cx.scheduled + (DELAY_PATTERN[*I] * 1_000_000).cycles())
            .ok();

        cx.resources.heartbeat_led.toggle().ok();

        *I = (*I + 1) % DELAY_PATTERN.len();
    }

    extern "C" {
        fn UART4();
    }
};

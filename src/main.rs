//! `datalogger` is a toy application that reads temperature and humidity data from a DHT11 sensor
//! and log it via WiFi.
//!
//! The main goal of this project is to explore the production readyness of the Rust ecosystem for
//! real-world embedded applications.

#![no_main]
#![no_std]

mod time;

use atsamd_hal::{
    clock::GenericClockController,
    common::gpio::Pa20,
    gpio::{OpenDrain, Output},
    prelude::*,
};
use panic_halt as _;
use time::{Duration, Ticker};

#[rtfm::app(device = atsamd_hal::target_device, peripherals = true, monotonic = crate::time::Ticker)]
const APP: () = {
    struct Resources {
        /// System clock.
        clk: Ticker,
        /// DHT11 sensor instance.
        // dht11: Dht11<PA8<Output<OpenDrain>>>,
        /// LED indicating CPU activity
        heartbeat_led: Pa20<Output<OpenDrain>>,
    }

    #[init(schedule = [heartbeat])]
    fn init(mut cx: init::Context) -> init::LateResources {
        // Clock the MCU using the external crystal and run the CPU at full speed
        let mut clocks = GenericClockController::with_external_32kosc(
            cx.device.GCLK,
            &mut cx.device.PM,
            &mut cx.device.SYSCTRL,
            &mut cx.device.NVMCTRL,
        );
        let gclk0 = clocks.gclk0();

        // Use TC3 as monotonic clock provider
        let clk = Ticker::init(
            cx.device.TC3,
            &clocks.tcc2_tc3(&gclk0).unwrap(),
            &mut cx.device.PM,
        );

        let mut pins = cx.device.PORT.split();

        // Create DHT11 instance on PA8
        // let dht11 = Dht11::new(pins.pa8.into_open_drain_output(&mut pins.port));

        // Create led instances
        let mut heartbeat_led = pins.pa20.into_open_drain_output(&mut pins.port);
        heartbeat_led.set_low().unwrap();

        // Schedule periodic tasks
        // cx.schedule.sensor_reading(cx.start).unwrap();
        cx.schedule.heartbeat(cx.start).unwrap();

        init::LateResources {
            clk,
            // dht11,
            heartbeat_led,
        }
    }

    /// Periodic software task which reads temperature and humidity data from the DHT11
    /// and schedules the data to be backed up to the persistent storage.
    // #[task(
    //     priority = 3,
    //     schedule = [sensor_reading],
    //     resources = [dwt, dht11]
    // )]
    // fn sensor_reading(cx: sensor_reading::Context) {
    //     static READING_PERIOD: u32 = 10 * 60; // 10 minutes
    //     static RETRY_PERIOD: u32 = 10; // 10 seconds

    //     let sensor_reading::Resources { mut dwt, dht11 } = cx.resources;

    //     let mut dly = dwt.lock(|dwt| dwt.delay());

    //     // Perform reading or schedule a retry
    //     let next_reading = if let Ok(meas) = dht11.perform_measurement(&mut dly) {
    //         Duration::from_secs(READING_PERIOD)
    //     } else {
    //         Duration::from_secs(RETRY_PERIOD)
    //     };

    //     cx.schedule
    //         .sensor_reading(cx.scheduled + next_reading)
    //         .unwrap();
    // }

    /// Low-priority background task that blinks the heartbeat led.
    #[task(
        priority = 1,
        schedule = [heartbeat],
        resources = [heartbeat_led]
    )]
    fn heartbeat(cx: heartbeat::Context) {
        static mut DELAY_PATTERN: [u32; 4] = [50, 150, 50, 1_000];
        static mut I: usize = 0;

        cx.schedule
            .heartbeat(cx.scheduled + Duration::from_millis(DELAY_PATTERN[*I]))
            .unwrap();

        // Blink heartbeat LED
        cx.resources.heartbeat_led.toggle();
        *I = (*I + 1) % DELAY_PATTERN.len();
    }

    /// Interrupt from the system timer. Runs at the highest priority.
    #[task(
        binds = TC3,
        priority = 15,
        resources = [clk]
    )]
    fn system_timer(cx: system_timer::Context) {
        cx.resources.clk.tick();
    }

    // Unused interrupts to dispatch software tasks
    extern "C" {
        fn I2S();
    }
};

#[cfg(not(debug_assertions))]
#[inline(never)]
#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    use core::sync::atomic::{self, Ordering};

    loop {
        atomic::compiler_fence(Ordering::SeqCst);
    }
}

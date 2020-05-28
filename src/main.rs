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
    gpio::{IntoFunction, OpenDrain, Output},
    prelude::*,
    usb::UsbBus,
};
use time::{Duration, Ticker};
use usb_device::{bus::UsbBusAllocator, prelude::*};
use usbd_serial::{SerialPort, USB_CLASS_CDC};

#[rtfm::app(device = atsamd_hal::target_device, peripherals = true, monotonic = crate::time::Ticker)]
const APP: () = {
    struct Resources {
        /// System clock.
        clk: Ticker,
        /// DHT11 sensor instance.
        // dht11: Dht11<PA8<Output<OpenDrain>>>,
        /// LED indicating CPU activity
        heartbeat_led: Pa20<Output<OpenDrain>>,
        // USB bus configured as USB device.
        usb_dev: UsbDevice<'static, UsbBus>,
        // CDC class USB device.
        serial: SerialPort<'static, UsbBus>,
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

        // Allocate USB bus instance (statically so that the borrow-checker is happy)
        let bus_allocator = unsafe {
            static mut USB_ALLOCATOR: Option<UsbBusAllocator<UsbBus>> = None;

            USB_ALLOCATOR = Some(UsbBusAllocator::new(UsbBus::new(
                &clocks.usb(&gclk0).unwrap(),
                &mut cx.device.PM,
                pins.pa24.into_function(&mut pins.port),
                pins.pa25.into_function(&mut pins.port),
                cx.device.USB,
            )));

            USB_ALLOCATOR.as_ref().unwrap()
        };

        // Create USB device and a serial port on top of it
        let serial = SerialPort::new(&bus_allocator);
        let usb_dev = UsbDeviceBuilder::new(&bus_allocator, UsbVidPid(0x16c0, 0x27dd))
            .product("Arduino Serial Port")
            .device_class(USB_CLASS_CDC)
            .build();

        // Schedule periodic tasks
        // cx.schedule.sensor_reading(cx.start).unwrap();
        cx.schedule.heartbeat(cx.start).unwrap();

        init::LateResources {
            clk,
            // dht11,
            heartbeat_led,
            usb_dev,
            serial,
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

    /// Interrupt from the USB port.
    #[task(
        binds = USB,
        priority = 14,
        resources = [usb_dev, serial]
    )]
    fn usb(cx: usb::Context) {
        let usb::Resources { usb_dev, serial } = cx.resources;

        usb_dev.poll(&mut [serial]);
        let mut buf = [0u8; 64];

        if let Ok(count) = serial.read(&mut buf) {
            for (i, c) in buf.iter().enumerate() {
                if i >= count {
                    break;
                }
                serial.write(&[c.clone()]).unwrap();
            }
        };
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

#[inline(never)]
#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    use core::sync::atomic::{self, Ordering};

    loop {
        atomic::compiler_fence(Ordering::SeqCst);
    }
}

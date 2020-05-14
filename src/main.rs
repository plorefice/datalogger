#![deny(unsafe_code)]
#![no_main]
#![no_std]

mod storage;

use dht11::{Dht11, Measurement};
use panic_semihosting as _;
use rtfm::cyccnt::U32Ext;
use stm32f4xx_hal::{
    dwt::{self, Dwt, DwtExt},
    gpio::{
        gpioa::PA8,
        gpiod::{PD12, PD13},
        GpioExt, OpenDrain, Output, PushPull,
        Speed::VeryHigh,
    },
    hal::{
        blocking::delay::DelayMs,
        digital::v2::{OutputPin, ToggleableOutputPin},
    },
    rcc::RccExt,
    sdio::{ClockFreq, Sdio},
};
use storage::{SdCard, Storage};

#[rtfm::app(device = stm32f4xx_hal::stm32, peripherals = true, monotonic = rtfm::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        dwt: Dwt,
        dht11: Dht11<PA8<Output<OpenDrain>>>,
        storage: Storage<dwt::Delay>,
        busy_led: PD13<Output<PushPull>>,
        heartbeat_led: PD12<Output<PushPull>>,
    }

    #[init(schedule = [sensor_reading, heartbeat])]
    fn init(cx: init::Context) -> init::LateResources {
        use stm32f4xx_hal::time::U32Ext;

        // We are using the HSE oscillator here for accurate communication with the DHT11.
        let rcc = cx.device.RCC.constrain();
        let clocks = rcc.cfgr.use_hse(8.mhz()).sysclk(168.mhz()).freeze();

        // Use the DWT as an accurate delay provider
        let dwt = cx.core.DWT.constrain(cx.core.DCB, clocks);

        let gpioa = cx.device.GPIOA.split();
        let gpioc = cx.device.GPIOC.split();
        let gpiod = cx.device.GPIOD.split();

        // Create DHT11 instance on PA8
        let dht11 = Dht11::new(gpioa.pa8.into_open_drain_output().set_speed(VeryHigh));

        // Create led instances
        let mut heartbeat_led = gpiod.pd12.into_push_pull_output();
        let mut busy_led = gpiod.pd13.into_push_pull_output();

        heartbeat_led.set_low().unwrap();
        busy_led.set_low().unwrap();

        // Create SDIO bus instance
        let mut sdio = Sdio::new(
            cx.device.SDIO,
            (
                gpioc.pc12.into_alternate_af12().internal_pull_up(false),
                gpiod.pd2.into_alternate_af12().internal_pull_up(true),
                gpioc.pc8.into_alternate_af12().internal_pull_up(true),
                gpioc.pc9.into_alternate_af12().internal_pull_up(true),
                gpioc.pc10.into_alternate_af12().internal_pull_up(true),
                gpioc.pc11.into_alternate_af12().internal_pull_up(true),
            ),
        );

        // Wait for an SD card to be inserted
        while sdio.init_card(ClockFreq::F400Khz).is_err() {
            dwt.delay().delay_ms(100_u32);
        }

        // Create and initialize an SD-backed storage
        let storage = Storage::new(SdCard::new(sdio, dwt.delay())).unwrap();

        // Schedule periodic tasks
        cx.schedule.sensor_reading(cx.start).unwrap();
        cx.schedule.heartbeat(cx.start).unwrap();

        init::LateResources {
            dwt,
            dht11,
            storage,
            busy_led,
            heartbeat_led,
        }
    }

    #[task(schedule = [sensor_reading], spawn = [save_data], resources = [dwt, dht11])]
    fn sensor_reading(cx: sensor_reading::Context) {
        static READING_PERIOD: u32 = 500_000_000;

        let mut delay = cx.resources.dwt.delay();

        // Perform sensor reading and save the measurement
        if let Ok(meas) = cx.resources.dht11.perform_measurement(&mut delay) {
            cx.spawn.save_data(meas).unwrap();
        }

        cx.schedule
            .sensor_reading(cx.scheduled + READING_PERIOD.cycles())
            .unwrap();
    }

    #[task(resources = [dwt, storage, busy_led])]
    fn save_data(cx: save_data::Context, meas: Measurement) {
        let save_data::Resources {
            storage,
            busy_led,
            dwt,
        } = cx.resources;

        busy_led.set_high().unwrap();

        // Try writing data to storage, and retry in case it fails
        while storage.save_measurement(meas).is_err() {
            dwt.delay().delay_ms(10_u32);
        }

        busy_led.set_low().unwrap();
    }

    #[task(schedule = [heartbeat], resources = [heartbeat_led])]
    fn heartbeat(cx: heartbeat::Context) {
        static mut DELAY_PATTERN: [u32; 4] = [1, 3, 1, 20];
        static mut I: usize = 0;

        cx.schedule
            .heartbeat(cx.scheduled + (DELAY_PATTERN[*I] * 8_000_000).cycles())
            .unwrap();

        cx.resources.heartbeat_led.toggle().unwrap();
        *I = (*I + 1) % DELAY_PATTERN.len();
    }

    extern "C" {
        fn UART4();
    }
};

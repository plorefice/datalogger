#![deny(unsafe_code)]
#![no_main]
#![no_std]

use cortex_m_semihosting::hprintln;
use dht11::Dht11;
use panic_semihosting as _;
use rtfm::cyccnt::U32Ext;
use stm32f4xx_hal::{
    dwt::{Dwt, DwtExt},
    gpio::{gpioa::PA8, gpiod::PD12, GpioExt, OpenDrain, Output, PushPull, Speed::VeryHigh},
    hal::{blocking::delay::DelayMs, digital::v2::ToggleableOutputPin},
    rcc::RccExt,
    sdio::{ClockFreq, Sdio},
};

#[rtfm::app(device = stm32f4xx_hal::stm32, peripherals = true, monotonic = rtfm::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        dwt: Dwt,
        dht11: Dht11<PA8<Output<OpenDrain>>>,
        sdcard: SdCard,
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

        // Create heartbeat led instance
        let heartbeat_led = gpiod.pd12.into_push_pull_output();

        // Create sd-card instance
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
            dwt.delay().delay_ms(1_000_u32);
        }

        cx.schedule.sensor_reading(cx.start).ok();
        cx.schedule.heartbeat(cx.start).ok();

        init::LateResources {
            dwt,
            dht11,
            sdcard: SdCard(sdio),
            heartbeat_led,
        }
    }

    #[task(schedule = [sensor_reading], spawn = [save_data], resources = [dwt, dht11])]
    fn sensor_reading(cx: sensor_reading::Context) {
        static READING_PERIOD: u32 = 500_000_000;

        let mut delay = cx.resources.dwt.delay();

        if let Ok(meas) = cx.resources.dht11.perform_measurement(&mut delay) {
            cx.spawn.save_data(meas).unwrap();
        }

        cx.schedule
            .sensor_reading(cx.scheduled + READING_PERIOD.cycles())
            .ok();
    }

    #[task(resources = [sdcard])]
    fn save_data(_: save_data::Context, meas: dht11::Measurement) {
        hprintln!("{:?}", meas).ok();
    }

    #[task(schedule = [heartbeat], resources = [heartbeat_led])]
    fn heartbeat(cx: heartbeat::Context) {
        static mut DELAY_PATTERN: [u32; 4] = [1, 3, 1, 20];
        static mut I: usize = 0;

        cx.schedule
            .heartbeat(cx.scheduled + (DELAY_PATTERN[*I] * 8_000_000).cycles())
            .ok();

        cx.resources.heartbeat_led.toggle().ok();

        *I = (*I + 1) % DELAY_PATTERN.len();
    }

    extern "C" {
        fn UART4();
    }
};

pub struct SdCard(Sdio);

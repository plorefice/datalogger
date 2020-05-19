//! `datalogger` is a toy application that reads temperature and humidity data from a DHT11 sensor
//! and stores it into a FAT-formatted SD card.
//!
//! The main goal of this project is to explore the production readyness of the Rust ecosystem for
//! real-world embedded applications.

#![no_main]
#![no_std]

mod network;
mod storage;
mod time;

#[cfg(debug_assertions)]
extern crate panic_semihosting;

use cast::u64;
use cortex_m::peripheral::NVIC;
use dht11::{Dht11, Measurement};
use network::Netlink;
use sntp::net;
use stm32f4xx_hal::{
    dwt::{self, Dwt, DwtExt},
    gpio::{
        gpioa::{PA0, PA8},
        gpiod::{PD12, PD13},
        Edge, ExtiPin, Floating, GpioExt, Input, OpenDrain, Output, PushPull,
        Speed::VeryHigh,
    },
    hal::{
        blocking::delay::DelayMs,
        digital::v2::{InputPin, OutputPin, ToggleableOutputPin},
    },
    rcc::RccExt,
    sdio::{ClockFreq, Sdio},
    stm32::{self, Interrupt},
};
use storage::{SdCard, Storage};
use time::{Duration, Instant, SystemTime, Ticker};

#[rtfm::app(device = stm32f4xx_hal::stm32, peripherals = true, monotonic = crate::time::Ticker)]
const APP: () = {
    struct Resources {
        /// A `Copy`able delay provider based on DWT.
        dwt: Dwt,
        /// System clock.
        clk: Ticker,
        /// DHT11 sensor instance.
        dht11: Dht11<PA8<Output<OpenDrain>>>,
        /// SD-backed persistent storage.
        storage: Storage<dwt::Delay>,
        /// Networking-related data.
        netlink: Netlink,
        /// LED indicating busy activity
        busy_led: PD13<Output<PushPull>>,
        /// LED indicating CPU activity
        heartbeat_led: PD12<Output<PushPull>>,
        /// Push-button used to sync data to disk
        sync_btn: PA0<Input<Floating>>,
    }

    #[init(schedule = [sensor_reading, heartbeat, netlink_loop])]
    fn init(mut cx: init::Context) -> init::LateResources {
        use stm32f4xx_hal::time::U32Ext;

        // Clock the MCU using the external crystal and run the CPU at full speed
        let rcc = cx.device.RCC.constrain();
        let clocks = rcc.cfgr.use_hse(8.mhz()).sysclk(168.mhz()).freeze();

        // Use the DWT as an accurate delay provider
        let dwt = cx.core.DWT.constrain(cx.core.DCB, clocks);

        // Use TIM2 as monotonic clock provider
        let clk = Ticker::init(cx.device.TIM2, clocks);

        let gpioa = cx.device.GPIOA.split();
        let gpiob = cx.device.GPIOB.split();
        let gpioc = cx.device.GPIOC.split();
        let gpiod = cx.device.GPIOD.split();

        // Create DHT11 instance on PA8
        let dht11 = Dht11::new(gpioa.pa8.into_open_drain_output().set_speed(VeryHigh));

        // Create led instances
        let mut heartbeat_led = gpiod.pd12.into_push_pull_output();
        let mut busy_led = gpiod.pd13.into_push_pull_output();

        heartbeat_led.set_low().unwrap();
        busy_led.set_low().unwrap();

        // Create user button
        let mut sync_btn = gpioa.pa0.into_floating_input();
        sync_btn.make_interrupt_source(&mut cx.device.SYSCFG);
        sync_btn.trigger_on_edge(&mut cx.device.EXTI, Edge::RISING);
        sync_btn.enable_interrupt(&mut cx.device.EXTI);
        // NOTE(unsafe) we are not in an interrupt context
        unsafe { NVIC::unmask(Interrupt::EXTI0) };

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

        // Create ethernet device and SNTP client
        let netlink = network::setup(
            cx.device.SYSCFG,
            (
                gpioa.pa1.into_alternate_af11(),
                gpioa.pa2.into_alternate_af11(),
                gpioa.pa7.into_alternate_af11(),
                gpiob.pb11.into_alternate_af11(),
                gpiob.pb12.into_alternate_af11(),
                gpiob.pb13.into_alternate_af11(),
                gpioc.pc1.into_alternate_af11(),
                gpioc.pc4.into_alternate_af11(),
                gpioc.pc5.into_alternate_af11(),
            ),
            cx.device.ETHERNET_MAC,
            cx.device.ETHERNET_DMA,
        );

        // Schedule periodic tasks
        cx.schedule.sensor_reading(cx.start).unwrap();
        cx.schedule.heartbeat(cx.start).unwrap();
        cx.schedule.netlink_loop(cx.start).unwrap();

        init::LateResources {
            dwt,
            clk,
            dht11,
            storage,
            netlink,
            busy_led,
            heartbeat_led,
            sync_btn,
        }
    }

    /// Periodic software task which reads temperature and humidity data from the DHT11
    /// and schedules the data to be backed up to the persistent storage.
    #[task(schedule = [sensor_reading], spawn = [save_data], resources = [dwt, dht11], priority = 2)]
    fn sensor_reading(cx: sensor_reading::Context) {
        static READING_PERIOD: u32 = 60_000; // 1 minute

        let sensor_reading::Resources { mut dwt, dht11 } = cx.resources;

        let mut dly = dwt.lock(|dwt| dwt.delay());

        if let Ok(meas) = dht11.perform_measurement(&mut dly) {
            cx.spawn.save_data(meas).unwrap();
        }

        cx.schedule
            .sensor_reading(cx.scheduled + Duration::from_millis(READING_PERIOD))
            .unwrap();
    }

    /// Software task spawned whenever new data needs to be saved.
    #[task(resources = [dwt, storage, busy_led], priority = 2)]
    fn save_data(cx: save_data::Context, meas: Measurement) {
        let save_data::Resources {
            mut storage,
            busy_led,
            mut dwt,
        } = cx.resources;

        let mut dly = dwt.lock(|dwt| dwt.delay());
        let acquisition_time = Instant::now();

        busy_led.set_high().unwrap();

        // Try writing data to storage, and retry forever in case it fails
        while storage
            .lock(|storage| storage.save_measurement(meas, acquisition_time))
            .is_err()
        {
            dly.delay_ms(10_u32);
        }

        busy_led.set_low().unwrap();
    }

    /// Flush buffered data to disk.
    #[task(binds = EXTI0, resources = [sync_btn, storage, dwt], priority = 3)]
    fn sync_data(cx: sync_data::Context) {
        static mut LAST_SYNC_TIME: Instant = Instant::zero();

        let sync_data::Resources {
            sync_btn,
            storage,
            dwt,
        } = cx.resources;

        let now = Instant::now();
        let elapsed_ms = now.duration_since(*LAST_SYNC_TIME).as_millis();

        // Sync at most once a second
        if sync_btn.is_high().unwrap() && elapsed_ms > 1_000 {
            while storage.flush_buffer().is_err() {
                dwt.delay().delay_ms(10_u32);
            }
            *LAST_SYNC_TIME = now;
        }

        // Clear interrupt
        sync_btn.clear_interrupt_pending_bit();
    }

    #[task(schedule = [netlink_loop], resources = [netlink])]
    fn netlink_loop(mut cx: netlink_loop::Context) {
        let Netlink {
            iface,
            sockets,
            sntp,
        } = &mut cx.resources.netlink;

        // Current instant in smolctp time
        let timestamp = net::time::Instant::from_millis(
            Instant::now().duration_since(Instant::zero()).as_millis() as i64,
        );

        // Poll socket interface
        iface.poll(sockets, timestamp).map(|_| ()).ok();

        // Process SNTP requests
        let network_time = sntp.poll(sockets, timestamp).unwrap_or_else(|_| None);
        if let Some(time) = network_time {
            // `time` is in seconds, to convert it to millis
            SystemTime::adjust(u64(time) * 1_000);
        }

        // Compute how long we can sleep
        let mut timeout = sntp.next_poll(timestamp);
        iface
            .poll_delay(&sockets, timestamp)
            .map(|sockets_timeout| timeout = sockets_timeout);

        // Sleep until next scheduled activation or until the ETH interrupt wakes us
        cx.schedule
            .netlink_loop(cx.scheduled + Duration::from_millis(timeout.millis() as u32))
            .ok();
    }

    /// Low-priority background task that blinks the heartbeat led.
    #[task(schedule = [heartbeat], resources = [heartbeat_led], priority = 1)]
    fn heartbeat(cx: heartbeat::Context) {
        static mut DELAY_PATTERN: [u32; 4] = [50, 150, 50, 1_000];
        static mut I: usize = 0;

        cx.schedule
            .heartbeat(cx.scheduled + Duration::from_millis(DELAY_PATTERN[*I]))
            .unwrap();

        cx.resources.heartbeat_led.toggle().unwrap();
        *I = (*I + 1) % DELAY_PATTERN.len();
    }

    /// Interrupt from the network interface. Runs at the second highest priority.
    #[task(binds = ETH, spawn = [netlink_loop], priority = 14)]
    fn ethernet_rx(cx: ethernet_rx::Context) {
        // Clear interrupt flags
        // TODO: use the safe API to access this peripheral
        let p = unsafe { stm32::Peripherals::steal() };
        stm32_eth::eth_interrupt_handler(&p.ETHERNET_DMA);

        // Spawn netlink loop to handle the packet
        cx.spawn.netlink_loop().ok();
    }

    /// Interrupt from the system timer. Runs at the highest priority.
    #[task(binds = TIM2, resources = [clk], priority = 15)]
    fn system_timer(cx: system_timer::Context) {
        cx.resources.clk.tick();
        SystemTime::tick();
    }

    // Unused interrupts to dispatch software tasks
    extern "C" {
        fn EXTI1();
        fn EXTI2();
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

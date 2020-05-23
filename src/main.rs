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
use core::fmt::Write;
use cortex_m::peripheral::NVIC;
use dht11::{Dht11, Measurement};
use heapless::{consts::U32, String};
use managed::ManagedSlice;
use network::Netlink;
use smolapps::{
    net::{
        socket::{SocketHandle, UdpSocket},
        wire::{IpCidr, IpEndpoint, Ipv4Address},
    },
    tftp::Transfer,
};
use stm32f4xx_hal::{
    dwt::{self, Dwt, DwtExt},
    gpio::{
        gpioa::{PA0, PA8},
        gpiod::{PD12, PD13, PD15},
        Edge, ExtiPin, Floating, GpioExt, Input, OpenDrain, Output, PushPull,
        Speed::VeryHigh,
    },
    hal::{
        blocking::delay::DelayMs,
        digital::v2::{InputPin, OutputPin, ToggleableOutputPin},
    },
    rcc::RccExt,
    sdio::{ClockFreq, Sdio},
    stm32::Interrupt,
};
use storage::{FileHandle, SdCard, Storage};
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
        netlink: Netlink<'static>,
        /// Socket used to send heartbeat messages.
        hb_socket: SocketHandle,
        /// LED indicating busy activity
        busy_led: PD13<Output<PushPull>>,
        /// LED indicating CPU activity
        heartbeat_led: PD12<Output<PushPull>>,
        /// LED indicating that the wall clock is NTP-synced.
        ntp_sync_led: PD15<Output<PushPull>>,
        /// Push-button used to sync data to disk
        sync_btn: PA0<Input<Floating>>,
        /// Whether or not the wall clock is network synced.
        #[init(false)]
        ntp_synced: bool,
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
        let mut ntp_sync_led = gpiod.pd15.into_push_pull_output();
        let mut busy_led = gpiod.pd13.into_push_pull_output();

        heartbeat_led.set_low().unwrap();
        ntp_sync_led.set_low().unwrap();
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
        let mut netlink = network::setup(
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

        // Create heartbeat socket
        let hb_socket = network::create_heartbeat_socket(&mut netlink.sockets);

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
            hb_socket,
            busy_led,
            heartbeat_led,
            ntp_sync_led,
            sync_btn,
        }
    }

    /// Periodic software task which reads temperature and humidity data from the DHT11
    /// and schedules the data to be backed up to the persistent storage.
    #[task(
        priority = 3,
        spawn = [save_data],
        schedule = [sensor_reading],
        resources = [dwt, dht11, ntp_synced]
    )]
    fn sensor_reading(cx: sensor_reading::Context) {
        static READING_PERIOD: u32 = 10 * 60; // 10 minutes
        static RETRY_PERIOD: u32 = 10; // 10 seconds

        let sensor_reading::Resources {
            mut dwt,
            dht11,
            ntp_synced,
        } = cx.resources;

        let mut dly = dwt.lock(|dwt| dwt.delay());

        // Perform reading or schedule a retry
        let next_reading = if *ntp_synced {
            if let Ok(meas) = dht11.perform_measurement(&mut dly) {
                cx.spawn.save_data(meas).unwrap();
                Duration::from_secs(READING_PERIOD)
            } else {
                Duration::from_secs(RETRY_PERIOD)
            }
        } else {
            Duration::from_secs(RETRY_PERIOD)
        };

        cx.schedule
            .sensor_reading(cx.scheduled + next_reading)
            .unwrap();
    }

    /// Software task spawned whenever new data needs to be saved.
    #[task(
        priority = 3,
        resources = [dwt, storage, busy_led]
    )]
    fn save_data(cx: save_data::Context, meas: Measurement) {
        let save_data::Resources {
            mut storage,
            busy_led,
            mut dwt,
        } = cx.resources;

        let mut dly = dwt.lock(|dwt| dwt.delay());
        let acquisition_time = SystemTime::now();

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
    #[task(
        binds = EXTI0,
        priority = 4,
        resources = [sync_btn, storage, dwt]
    )]
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

    // Run the network loop.
    #[task(
        capacity = 4, // ETH IRQ handler + immediate timeout + scheduled timeout + one spare
        priority = 2,
        schedule = [netlink_loop],
        resources = [netlink, storage, ntp_synced, ntp_sync_led]
    )]
    fn netlink_loop(cx: netlink_loop::Context) {
        static mut TRANSFERS: [Option<Transfer<FileHandle<dwt::Delay>>>; 1] = [None; 1];
        static mut SCHEDULED: Option<Instant> = None;

        let netlink_loop::Resources {
            mut netlink,
            mut storage,
            mut ntp_synced,
            ntp_sync_led,
        } = cx.resources;

        // Current instant in smolctp time
        let timestamp = Instant::now().into();

        // This runs in a critical section shared with the ethernet interrupt,
        // so we need to verify that this does not cause problems like packet loss.
        let timeout = netlink.lock(
            |Netlink {
                 iface,
                 sockets,
                 dhcp,
                 sntp,
                 tftp,
             }| {
                // Poll socket interface
                iface.poll(sockets, timestamp).ok();

                // Process DHCP requests
                if let Some(cfg) = dhcp.poll(iface, sockets, timestamp).unwrap_or(None) {
                    // If an IP address is received, assign it to the interface
                    match cfg.address {
                        Some(cidr) if !iface.has_ip_addr(cidr.address()) => {
                            iface.update_ip_addrs(|addrs| {
                                addrs.iter_mut().nth(0).map(|addr| {
                                    *addr = IpCidr::Ipv4(cidr);
                                });
                            });
                        }
                        _ => (),
                    }

                    // Also set the default gateway, even though we don't use it
                    if let Some(route) = cfg.router {
                        iface
                            .routes_mut()
                            .add_default_ipv4_route(route.into())
                            .unwrap();
                    }
                }

                // Process SNTP requests
                let network_time = sntp.poll(sockets, timestamp).unwrap_or_else(|_| None);
                if let Some(time) = network_time {
                    // `time` is in seconds, to convert it to millis
                    SystemTime::adjust(u64(time) * 1_000);
                    ntp_synced.lock(|sync| *sync = true);
                    ntp_sync_led.set_high().unwrap();
                }

                // Process TFTP transfers
                storage.lock(|storage| {
                    tftp.serve(
                        sockets,
                        &mut *storage,
                        &mut ManagedSlice::Borrowed(&mut *TRANSFERS),
                        timestamp,
                    )
                    .ok();
                });

                // Compute how long we can sleep
                let mut timeout = tftp
                    .next_poll(timestamp)
                    .min(sntp.next_poll(timestamp))
                    .min(dhcp.next_poll(timestamp));

                if let Some(t) = iface.poll_delay(&sockets, timestamp) {
                    timeout = t.min(timeout);
                }

                timeout
            },
        );

        if timeout.millis() == 0 {
            // An immediate polling was requested (probably due to outgoing transmissions).
            // Immediately reschedule the task.
            cx.schedule.netlink_loop(cx.scheduled).ok();
        } else if SCHEDULED.is_none()
            || (SCHEDULED.is_some() && SCHEDULED.unwrap() < timestamp.into())
        {
            // No immediate action is required.
            // In this case, only schedule an activation if the previously scheduled one has expired.
            let scheduled = cx.scheduled + timeout.into();
            cx.schedule.netlink_loop(scheduled).ok();
            *SCHEDULED = Some(scheduled);
        }
    }

    /// Low-priority background task that blinks the heartbeat led.
    #[task(
        priority = 1,
        schedule = [heartbeat],
        resources = [heartbeat_led, netlink, hb_socket]
    )]
    fn heartbeat(cx: heartbeat::Context) {
        static mut HEARTBEAT_TIMER: Option<Instant> = None;
        static mut DELAY_PATTERN: [u32; 4] = [50, 150, 50, 1_000];
        static mut I: usize = 0;

        const HEARTBEAT_PERIOD: Duration = Duration::from_secs(5);

        let heartbeat::Resources {
            mut netlink,
            hb_socket,
            heartbeat_led,
        } = cx.resources;

        // Send heartbeat packet
        if HEARTBEAT_TIMER.is_none()
            || (HEARTBEAT_TIMER.is_some() && cx.scheduled > HEARTBEAT_TIMER.unwrap())
        {
            netlink.lock(|netlink| {
                // Do not send out heartbeats if we don't have an IP yet
                match netlink.iface.ipv4_address() {
                    Some(ip) if !ip.is_unspecified() => {
                        let mut heartbeat = String::<U32>::new();
                        write!(&mut heartbeat, "datalogger {}", ip).unwrap();

                        let mut socket = netlink.sockets.get::<UdpSocket>(*hb_socket);

                        if !socket.is_open() {
                            socket
                                .bind(IpEndpoint {
                                    addr: ip.into(),
                                    port: 20_000,
                                })
                                .unwrap();
                        }

                        socket
                            .send_slice(
                                heartbeat.as_str().as_bytes(),
                                IpEndpoint {
                                    addr: Ipv4Address::BROADCAST.into(),
                                    port: 20_000,
                                },
                            )
                            .ok();
                    }
                    _ => {}
                }
            });

            *HEARTBEAT_TIMER = Some(cx.scheduled + HEARTBEAT_PERIOD);
        }

        cx.schedule
            .heartbeat(cx.scheduled + Duration::from_millis(DELAY_PATTERN[*I]))
            .unwrap();

        // Blink heartbeat LED
        heartbeat_led.toggle().unwrap();
        *I = (*I + 1) % DELAY_PATTERN.len();
    }

    /// Interrupt from the network interface. Runs at the second highest priority.
    #[task(
        binds = ETH,
        priority = 14,
        schedule = [netlink_loop],
        resources = [netlink]
    )]
    fn ethernet_rx(cx: ethernet_rx::Context) {
        // Clear interrupt flags
        cx.resources.netlink.iface.device().interrupt_handler();

        // Spawn netlink loop to handle the packet
        cx.schedule.netlink_loop(cx.start).ok();
    }

    /// Interrupt from the system timer. Runs at the highest priority.
    #[task(
        binds = TIM2,
        priority = 15,
        resources = [clk]
    )]
    fn system_timer(cx: system_timer::Context) {
        cx.resources.clk.tick();
        SystemTime::tick();
    }

    // Unused interrupts to dispatch software tasks
    extern "C" {
        fn EXTI1();
        fn EXTI2();
        fn EXTI3();
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

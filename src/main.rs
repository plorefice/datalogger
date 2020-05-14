#![deny(unsafe_code)]
#![no_main]
#![no_std]

use core::{
    cell::RefCell,
    fmt::{self, Write},
};
use dht11::{Dht11, Measurement};
use embedded_sdmmc::{
    Block, BlockCount, BlockDevice, BlockIdx, Controller, File, Mode, TimeSource, Timestamp,
    Volume, VolumeIdx,
};
use heapless::{consts::U32, ArrayLength, Vec};
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
    sdio::{self, ClockFreq, Sdio},
};

#[rtfm::app(device = stm32f4xx_hal::stm32, peripherals = true, monotonic = rtfm::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        dwt: Dwt,
        dht11: Dht11<PA8<Output<OpenDrain>>>,
        storage: Storage,
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
        let storage = Storage::new(SdCard(RefCell::new((sdio, dwt.delay())))).unwrap();

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

/// An SD card attached to a SDIO bus.
pub struct SdCard(RefCell<(Sdio, dwt::Delay)>);

impl BlockDevice for SdCard {
    type Error = sdio::Error;

    fn read(
        &self,
        blocks: &mut [Block],
        start_block_idx: BlockIdx,
        _reason: &str,
    ) -> Result<(), Self::Error> {
        for (i, block) in blocks.iter_mut().enumerate() {
            let mut inner = self.0.borrow_mut();

            while inner
                .0
                .read_block(start_block_idx.0 + i as u32, &mut block.contents)
                .is_err()
            {
                // FIXME: writing too often seems to trigger a timeout error.
                inner.1.delay_ms(10_u8);
            }
        }
        Ok(())
    }

    fn write(&self, blocks: &[Block], start_block_idx: BlockIdx) -> Result<(), Self::Error> {
        for (i, block) in blocks.iter().enumerate() {
            let mut inner = self.0.borrow_mut();

            while inner
                .0
                .write_block(start_block_idx.0 + i as u32, &block.contents)
                .is_err()
            {
                // FIXME: reading too often seems to trigger a timeout error.
                inner.1.delay_ms(10_u8);
            }
        }
        Ok(())
    }

    fn num_blocks(&self) -> Result<BlockCount, Self::Error> {
        Ok(BlockCount(self.0.borrow().0.card()?.block_count()))
    }
}

/// A `TimeSource` always returning the Unix epoch time.
struct Epoch;

impl TimeSource for Epoch {
    fn get_timestamp(&self) -> Timestamp {
        Timestamp {
            year_since_1970: 0,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}

/// A persistent storage backed by an `SdCard` with a FAT partition.
pub struct Storage {
    controller: Controller<SdCard, Epoch>,
    volume: Volume,
    data: File,
}

impl Storage {
    /// Creates a new storage using the provided SD card instance.
    pub fn new(sd: SdCard) -> Result<Self, embedded_sdmmc::Error<sdio::Error>> {
        let mut controller = Controller::new(sd, Epoch);
        let mut volume = controller.get_volume(VolumeIdx(0))?;

        let root_dir = controller.open_root_dir(&volume)?;

        let data = controller.open_file_in_dir(
            &mut volume,
            &root_dir,
            "data.csv",
            Mode::ReadWriteCreateOrAppend,
        )?;

        Ok(Storage {
            controller,
            volume,
            data,
        })
    }

    /// Saves the specified measurement to the underlying storage.
    pub fn save_measurement(
        &mut self,
        meas: Measurement,
    ) -> Result<(), embedded_sdmmc::Error<sdio::Error>> {
        let mut s: String<U32> = String::new();

        writeln!(&mut s, "{:.02},{:.02}", meas.temperature, meas.humidity).unwrap();

        self.controller.write(
            &mut self.volume,
            &mut self.data,
            &s.into_bytes_exact().unwrap(),
        )?;

        Ok(())
    }
}

/// Wrapper around a `heapless::String` which keeps track
/// of the amount of bytes written into it.
struct String<N: ArrayLength<u8>> {
    inner: heapless::String<N>,
    len: usize,
}

impl<N: ArrayLength<u8>> Write for String<N> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        write!(&mut self.inner, "{}", s)?;
        self.len += s.len();
        Ok(())
    }
}

impl<N: ArrayLength<u8>> String<N> {
    /// Creates a new fixed-size string.
    fn new() -> Self {
        Self {
            inner: heapless::String::new(),
            len: 0,
        }
    }

    /// Converts the string into a `Vec<u8>` containing the portion of the string
    /// that has been written to.
    fn into_bytes_exact(self) -> Result<Vec<u8, N>, ()> {
        let mut v = self.inner.into_bytes();
        v.resize(self.len, 0)?;
        Ok(v)
    }
}

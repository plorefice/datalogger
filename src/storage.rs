use core::{
    cell::RefCell,
    fmt::{self, Write},
};
use dht11::Measurement;
use embedded_sdmmc::{
    Block, BlockCount, BlockDevice, BlockIdx, Controller, File, Mode, TimeSource, Timestamp,
    Volume, VolumeIdx,
};
use heapless::{consts::U32, ArrayLength, Vec};
use panic_semihosting as _;
use stm32f4xx_hal::{
    hal::blocking::delay::DelayMs,
    sdio::{self, Sdio},
};

/// An SD card attached to a SDIO bus.
pub struct SdCard<D>(RefCell<(Sdio, D)>);

impl<D> SdCard<D> {
    pub fn new(sdio: Sdio, delay: D) -> Self {
        Self(RefCell::new((sdio, delay)))
    }
}

impl<D: DelayMs<u8>> BlockDevice for SdCard<D> {
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
pub struct Storage<D: DelayMs<u8>> {
    controller: Controller<SdCard<D>, Epoch>,
    volume: Volume,
    data: File,
}

impl<D: DelayMs<u8>> Storage<D> {
    /// Creates a new storage using the provided SD card instance.
    pub fn new(sd: SdCard<D>) -> Result<Self, embedded_sdmmc::Error<sdio::Error>> {
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

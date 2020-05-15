use core::{cell::RefCell, fmt::Write};
use dht11::Measurement;
use embedded_sdmmc::{
    Block, BlockCount, BlockDevice, BlockIdx, Controller, Mode, TimeSource, Timestamp, Volume,
    VolumeIdx,
};
use heapless::{consts::U4096, String};
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
    buffer: String<U4096>,
}

impl<D: DelayMs<u8>> Storage<D> {
    /// Creates a new storage using the provided SD card instance.
    pub fn new(sd: SdCard<D>) -> Result<Self, embedded_sdmmc::Error<sdio::Error>> {
        let mut controller = Controller::new(sd, Epoch);
        let volume = controller.get_volume(VolumeIdx(0))?;
        let buffer = String::new();

        Ok(Storage {
            controller,
            volume,
            buffer,
        })
    }

    /// Saves the specified measurement to the underlying storage.
    pub fn save_measurement(
        &mut self,
        meas: Measurement,
    ) -> Result<(), embedded_sdmmc::Error<sdio::Error>> {
        // Try storing the measurement in memory first.
        // If the memory is full, flush it to disk, then try again.
        if writeln!(
            &mut self.buffer,
            "{:.02},{:.02}",
            meas.temperature, meas.humidity
        )
        .is_err()
        {
            self.flush_buffer()?;
            self.save_measurement(meas)?;
        }

        Ok(())
    }

    /// Flushes the internal buffer to the underlying storage. The buffer is then cleared.
    fn flush_buffer(&mut self) -> Result<(), embedded_sdmmc::Error<sdio::Error>> {
        let root_dir = self.controller.open_root_dir(&self.volume)?;

        let mut data = self.controller.open_file_in_dir(
            &mut self.volume,
            &root_dir,
            "data.csv",
            Mode::ReadWriteCreateOrAppend,
        )?;

        self.controller.write(
            &mut self.volume,
            &mut data,
            &self.buffer.as_str().as_bytes(),
        )?;

        self.buffer.clear();

        // Files and dirs and not closed automatically when dropped!
        self.controller.close_file(&mut self.volume, data)?;
        self.controller.close_dir(&mut self.volume, root_dir);

        Ok(())
    }
}

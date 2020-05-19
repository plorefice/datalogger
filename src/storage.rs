use crate::time::SystemTime;
use core::{cell::RefCell, fmt::Write};
use dht11::Measurement;
use embedded_sdmmc::{
    Block, BlockCount, BlockDevice, BlockIdx, Controller, File, Mode, TimeSource, Timestamp,
    Volume, VolumeIdx,
};
use heapless::{
    consts::{U32, U4096},
    String,
};
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
        acquisition_time: SystemTime,
    ) -> Result<(), embedded_sdmmc::Error<sdio::Error>> {
        let mut tmp = String::<U32>::new();

        // Temperature to integer and decimal part
        let t_i = meas.temperature / 10;
        let t_d = meas.temperature.abs() - (t_i * 10);

        // Humidity to integer and decimal part
        let h_i = meas.humidity / 10;
        let h_d = meas.humidity - (h_i * 10);

        // NOTE(unwrap) the formatted string must always fit the temporary buffer
        writeln!(
            &mut tmp,
            "{},{}.{:01},{}.{:01}",
            acquisition_time.as_secs(),
            t_i,
            t_d,
            h_i,
            h_d
        )
        .unwrap();

        // Try storing the measurement in memory first.
        // If the memory is full, flush it to disk, then try again.
        if self.buffer.push_str(&tmp).is_err() {
            self.flush_buffer()?;
            // NOTE(unwrap) this must not fail after the buffer was flushed
            self.buffer.push_str(&tmp).unwrap();
        }

        Ok(())
    }

    /// Flushes the internal buffer to the underlying storage. The buffer is then cleared.
    pub fn flush_buffer(&mut self) -> Result<(), embedded_sdmmc::Error<sdio::Error>> {
        let mut file = self
            .controller
            .open_root_dir(&self.volume)
            .and_then(|dir| {
                let file = self.controller.open_file_in_dir(
                    &mut self.volume,
                    &dir,
                    "data.csv",
                    Mode::ReadWriteCreateOrAppend,
                );

                // Not needed anymore
                self.controller.close_dir(&self.volume, dir);

                file
            })?;

        let res = self.flush_to_file(&mut file);

        self.controller.close_file(&self.volume, file)?;

        self.buffer.clear();

        res
    }

    /// Writes the internal buffer to the specified file on disk.
    fn flush_to_file(&mut self, file: &mut File) -> Result<(), embedded_sdmmc::Error<sdio::Error>> {
        if file.length() == 0 {
            // New file, insert CSV header
            self.controller
                .write(&mut self.volume, file, b"Timestamp,Temperature,Humidity\n")?;
        }

        self.controller
            .write(&mut self.volume, file, &self.buffer.as_str().as_bytes())?;

        Ok(())
    }
}

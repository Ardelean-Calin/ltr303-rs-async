#![no_std]

use core::prelude::rust_2021::*;

mod types;
pub use types::*;

#[cfg(feature = "defmt")]
use defmt::Format;
#[cfg(feature = "defmt")]
use defmt::{info, write};
use embedded_hal_async::delay::DelayUs;
use embedded_hal_async::i2c::{self};

#[repr(u8)]
enum Register {
    ALS_CONTR = 0x80,
    ALS_MEAS_RATE = 0x85,
    PART_ID = 0x86,
    MANUFAC_ID = 0x87,
    ALS_DATA_CH1_LOW = 0x88,
    ALS_DATA_CH1_HIGH = 0x89,
    ALS_DATA_CH0_LOW = 0x8A,
    ALS_DATA_CH0_HIGH = 0x8B,
    ALS_STATUS = 0x8C,
    INTERRUPT = 0x8F,
    ALS_THRES_UP_0 = 0x97,
    ALS_THRES_UP_1 = 0x98,
    ALS_THRES_LOW_0 = 0x99,
    ALS_THRES_LOW_1 = 0x9A,
    INTERRUPT_PERSIST = 0x9E,
    NONE,
}

pub struct Identifier {
    mfc_id: u8,
    part_id: u8,
}

#[cfg(feature = "defmt")]
impl Format for Identifier {
    fn format(&self, fmt: defmt::Formatter) {
        write!(
            fmt,
            "MFC_ID: 0x{:02X}  PART_ID: 0x{:02X}",
            self.mfc_id, self.part_id
        );
    }
}

/// All possible errors in this crate
#[derive(Debug, PartialEq, Clone)]
pub enum LTR303Error<E> {
    /// I²C bus error
    I2c(E),
    /// Some kind of timing related error.
    TimingError,
}

#[cfg(feature = "defmt")]
impl<E> Format for LTR303Error<E> {
    fn format(&self, fmt: defmt::Formatter) {
        write!(fmt, "{:?}", self);
    }
}

/// The I2C address of the LTR303-ALS sensor.
const ADDRESS: u8 = 0x29;

#[derive(Debug, Default)]
pub struct Ltr303<I2C>
where
    I2C: i2c::I2c,
{
    i2c: I2C,
}

impl<I2C, E> Ltr303<I2C>
where
    I2C: i2c::I2c<Error = E>,
{
    pub fn new(i2c: I2C) -> Self {
        Self { i2c }
    }

    /// Reads data from a register.
    async fn read_register(&mut self, reg_address: Register) -> Result<u8, LTR303Error<E>> {
        let mut buf: [u8; 1] = [0; 1];
        self.i2c
            .write_read(ADDRESS, &[reg_address as u8], &mut buf)
            .await
            .map_err(LTR303Error::I2c)
            .and(Ok(buf[0]))
    }

    /// Writes the given byte in the given register.
    async fn write_register(
        &mut self,
        reg_address: Register,
        reg_value: u8,
    ) -> Result<(), LTR303Error<E>> {
        self.i2c
            .write(ADDRESS, &[reg_address as u8, reg_value])
            .await
            .map_err(LTR303Error::I2c)
    }

    /// Gets the manufacturer ID and part ID. These should be 0x05 and 0xA0.
    pub async fn get_identifier(&mut self) -> Result<Identifier, LTR303Error<E>> {
        let mfc_id = self.read_register(Register::MANUFAC_ID).await?;
        let part_id = self.read_register(Register::PART_ID).await?;

        Ok(Identifier { mfc_id, part_id })
    }
}

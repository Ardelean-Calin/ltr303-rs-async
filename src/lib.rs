#![no_std]

use core::prelude::rust_2021::*;

mod types;
pub use types::*;

#[cfg(feature = "defmt")]
use defmt::Format;
#[cfg(feature = "defmt")]
use defmt::{info, write};
use embedded_hal::blocking::i2c::{Read, Write, WriteRead};
use embedded_hal_async::delay::DelayUs;

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

pub struct Measurement {
    pub lux: u16,
}

pub struct Identifier {
    pub mfc_id: u8,
    pub part_id: u8,
}

#[cfg(feature = "defmt")]
impl Format for Measurement {
    fn format(&self, fmt: defmt::Formatter) {
        write!(fmt, "Lux: {}", self.lux);
    }
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
pub struct Ltr303<I2C> {
    i2c: I2C,
}

impl<I2C, E> Ltr303<I2C>
where
    I2C: Write<Error = E> + Read<Error = E> + WriteRead<Error = E>,
{
    pub fn new(i2c: I2C) -> Self {
        Self { i2c }
    }

    /// Reads data from a register.
    fn read_register(&mut self, reg_address: Register) -> Result<u8, LTR303Error<E>> {
        let mut buf: [u8; 1] = [0; 1];
        self.i2c
            .write_read(ADDRESS, &[reg_address as u8], &mut buf)
            .map_err(LTR303Error::I2c)
            .and(Ok(buf[0]))
    }

    /// Writes the given byte in the given register.
    fn write_register(
        &mut self,
        reg_address: Register,
        reg_value: u8,
    ) -> Result<(), LTR303Error<E>> {
        self.i2c
            .write(ADDRESS, &[reg_address as u8, reg_value])
            .map_err(LTR303Error::I2c)
    }

    pub async fn sample(
        &mut self,
        delay: &mut impl DelayUs,
    ) -> Result<Measurement, LTR303Error<E>> {
        // Start measurement. Default values.
        let command: u8 = 0b000000001;
        self.write_register(Register::ALS_CONTR, command)?;

        delay
            .delay_ms(100)
            .await
            .map_err(|_| LTR303Error::TimingError)?;

        // Wait for data to be ready.
        loop {
            let status = self.read_register(Register::ALS_STATUS)?;
            // info!("Read status register: {}", status);
            if status & 0x04 != 0 {
                break;
            }
            delay
                .delay_ms(10)
                .await
                .map_err(|_| LTR303Error::TimingError)?;
        }

        let ch1_0 = self.read_register(Register::ALS_DATA_CH1_LOW)? as u16;
        let ch1_1 = self.read_register(Register::ALS_DATA_CH1_HIGH)? as u16;
        let ch0_0 = self.read_register(Register::ALS_DATA_CH0_LOW)? as u16;
        let ch0_1 = self.read_register(Register::ALS_DATA_CH0_HIGH)? as u16;
        // Go to sleep.
        let command: u8 = 0b000000000;
        self.write_register(Register::ALS_CONTR, command)?;

        let ch0 = (ch0_1 << 8) + ch0_0;
        let ch1 = (ch1_1 << 8) + ch1_0;

        // Go to sleep
        let command: u8 = 0b000000000;
        self.write_register(Register::ALS_CONTR, command)?;

        Ok(Measurement {
            lux: raw_to_lux(ch0, ch1),
        })
    }

    pub async fn sleep(&mut self) {}
    pub async fn wakeup(&mut self, delay: &mut impl DelayUs) {}

    /// Gets the manufacturer ID and part ID. These should be 0x05 and 0xA0.
    pub async fn get_identifier(&mut self) -> Result<Identifier, LTR303Error<E>> {
        let mfc_id = self.read_register(Register::MANUFAC_ID)?;
        let part_id = self.read_register(Register::PART_ID)?;

        Ok(Identifier { mfc_id, part_id })
    }
}

#[inline]
fn raw_to_lux(lux_raw_ch0: u16, lux_raw_ch1: u16) -> u16 {
    let ratio = lux_raw_ch1 as f32 / (lux_raw_ch0 as f32 + lux_raw_ch1 as f32);
    let als_gain: f32 = 1.0;
    let int_time: f32 = 1.0;

    let result = if ratio < 0.45 {
        ((1.7743 * f32::from(lux_raw_ch0)) + (1.1059 * f32::from(lux_raw_ch1)))
            / als_gain
            / int_time
    } else if (0.45..0.64).contains(&ratio) {
        ((4.2785 * f32::from(lux_raw_ch0)) - (1.9548 * f32::from(lux_raw_ch1)))
            / als_gain
            / int_time
    } else if (0.64..0.85).contains(&ratio) {
        ((0.5926 * f32::from(lux_raw_ch0)) - (0.1185 * f32::from(lux_raw_ch1)))
            / als_gain
            / int_time
    } else {
        0.0
    };

    result as u16
}

//! A platform agnostic driver to interface with the QMC5883L magnetometer.
//!
//! This driver was built using [`embedded-hal`] traits.
//!
//! [`embedded-hal`]: https://docs.rs/embedded-hal/~0.2

#![no_std]

extern crate embedded_hal as hal;

use core::slice::from_mut;

use hal::blocking::i2c::{Write, WriteRead};

const SLAVE_ADDRESS: u8 = 0x0d;

#[allow(dead_code)]
#[allow(non_camel_case_types)]
#[derive(Copy, Clone)]
#[repr(u8)]
pub enum Register {
    DATA_OUT_X_L = 0,
    DATA_OUT_X_H = 1,
    DATA_OUT_Y_L = 2,
    DATA_OUT_Y_H = 3,
    DATA_OUT_Z_L = 4,
    DATA_OUT_Z_H = 5,
    STATUS = 6,
    TOUT_L = 7,
    TOUT_H = 8,
    CONTROL1 = 9,
    CONTROL2 = 10,
    PERIOD = 11,
    CHIP_ID = 13,
}

pub const STATUS_OVL: u8 = 0b010;
pub const STATUS_DRDY: u8 = 0b001;

const MODE_CONTINUOUS: u8 = 0b01;

const CTRL2_SOFT_RST: u8 = 1 << 7;
const CTRL2_ROL_PNT: u8 = 1 << 6;
const CTRL2_INT_ENB: u8 = 1 << 0;

/// Update frequency; 10Hz recommended for low power consumption.
#[repr(u8)]
pub enum OutputDataRate {
    /// 10Hz update rate.
    Rate10Hz = 0,
    /// 50Hz update rate.
    Rate50Hz = 0b0100,
    /// 100Hz update rate.
    Rate100Hz = 0b1000,
    /// 200Hz update rate.
    Rate200Hz = 0b1100,
}

/// Oversampling rate; controls bandwidth of internal digital filter.
/// Larger oversampling gets less in-band noise but higher power consumption.
#[repr(u8)]
pub enum OversampleRate {
    /// Oversample by 64.
    Rate64 = 3 << 6,
    /// Oversample by 128.
    Rate128 = 1 << 7,
    /// Oversample by 256.
    Rate256 = 1 << 6,
    /// Oversample by 512.
    Rate512 = 0,
}

/// Field range of magnetic sensor.
#[repr(u8)]
pub enum FieldRange {
    /// ± 2 gauss
    Range2Gauss = 0,
    /// ± 8 gauss
    Range8Gauss = 1 << 3,
}

impl FieldRange {
    pub fn sensitive(self) -> i16 {
        match self {
            Self::Range2Gauss => 12000,
            Self::Range8Gauss => 3000,
        }
    }
}

/// QMC5883L Error
#[derive(Debug, Copy, Clone)]
pub enum Error<E> {
    /// CHIP_ID returned invalid value (returned value is argument).
    InvalidDevice(u8),
    /// Read taken from magnetometer before ready.
    NotReady,
    /// Reading overflowed.
    Overflow,
    /// Underlying I2C bus error.
    BusError(E),
}

impl<E> core::convert::From<E> for Error<E> {
    fn from(e: E) -> Self {
        Error::BusError(e)
    }
}

/// QMC5883L driver
pub struct QMC5883L<I2C>(I2C);

impl QMC5883L<()> {
    pub fn probe<E>(i2c: &mut dyn WriteRead<Error = E>) -> Result<bool, E> {
        let mut value = 0u8;
        i2c.write_read(SLAVE_ADDRESS, &[Register::CHIP_ID as u8], from_mut(&mut value))?;
        Ok(value == 0xff)
    }
}

impl<I2C> QMC5883L<I2C> {
    /// Creates a new QMC5883L device from an I2C peripheral; begins with a soft reset.
    pub fn new(i2c: I2C) -> Self {
        QMC5883L(i2c)
    }
}

impl<I2C: WriteRead<Error = E> + Write<Error = E>, E> QMC5883L<I2C> {
    pub fn into_inner(self) -> I2C {
        self.0
    }

    /// Soft reset the device.
    pub fn reset(&mut self) -> Result<(), E> {
        self.write_u8(Register::CONTROL2, CTRL2_SOFT_RST)?;
        self.write_u8(Register::CONTROL2, CTRL2_ROL_PNT | CTRL2_INT_ENB)?;
        self.write_u8(Register::PERIOD, 1)
    }

    /// Set the device field range.
    pub fn set_field_range(&mut self, field_range: FieldRange) -> Result<(), E> {
        let ctrl1 = self.read_u8(Register::CONTROL1)?;
        let v = (ctrl1 & !(FieldRange::Range8Gauss as u8)) | (field_range as u8);
        self.write_u8(Register::CONTROL1, v)
    }

    /// Set the device oversampling rate.
    pub fn set_oversample(&mut self, rate: OversampleRate) -> Result<(), E> {
        let ctrl1 = self.read_u8(Register::CONTROL1)?;
        let v = (ctrl1 & !(OversampleRate::Rate64 as u8)) | (rate as u8);
        self.write_u8(Register::CONTROL1, v)
    }

    /// Set the device output data rate.
    pub fn set_output_data_rate(&mut self, rate: OutputDataRate) -> Result<(), E> {
        let ctrl1 = self.read_u8(Register::CONTROL1)?;
        let v = (ctrl1 & !(OutputDataRate::Rate200Hz as u8)) | (rate as u8);
        self.write_u8(Register::CONTROL1, v)
    }

    /// Put device in continous mode.
    pub fn continuous(&mut self) -> Result<(), E> {
        let ctrl1 = self.read_u8(Register::CONTROL1)?;
        self.write_u8(Register::CONTROL1, ctrl1 | MODE_CONTINUOUS)
    }

    /// Put device in standby mode.
    pub fn standby(&mut self) -> Result<(), E> {
        let ctrl1 = self.read_u8(Register::CONTROL1)?;
        self.write_u8(Register::CONTROL1, ctrl1 & !MODE_CONTINUOUS)
    }

    /// Enable interrupt pin.
    pub fn enable_interrupt(&mut self) -> Result<(), E> {
        self.write_u8(Register::CONTROL2, CTRL2_ROL_PNT)
    }

    /// Disable interrupt pin.
    pub fn disable_interrupt(&mut self) -> Result<(), E> {
        self.write_u8(Register::CONTROL2, CTRL2_ROL_PNT | CTRL2_INT_ENB)
    }

    /// Read temperature sensor; temperature coefficient is about 100 LSB/°C.
    pub fn read_temperature(&mut self) -> Result<i16, E> {
        let temp_l = self.read_u8(Register::TOUT_L)? as i16;
        let temp_h = self.read_u8(Register::TOUT_H)? as i16;
        Ok((temp_h << 8) | temp_l)
    }

    /// Read raw (x,y,z) from magnetometer.
    pub fn read_magnetism(&mut self) -> Result<(i16, i16, i16), Error<E>> {
        let mut buf = [0u8; 8];
        self.0.write_read(SLAVE_ADDRESS, &[Register::STATUS as u8], &mut buf[1..])?;
        let status = buf[1];
        if (status & STATUS_DRDY) == 0 {
            return Err(Error::NotReady);
        } else if (status & STATUS_OVL) != 0 {
            return Err(Error::Overflow);
        }
        let result: [i16; 4] = unsafe { core::mem::transmute(buf) };
        Ok((i16::from_le(result[1]), i16::from_le(result[2]), i16::from_le(result[3])))
    }

    fn read_u8(&mut self, reg: Register) -> Result<u8, E> {
        let mut value = 0u8;
        self.0.write_read(SLAVE_ADDRESS, &[reg as u8], from_mut(&mut value))?;
        Ok(value)
    }

    fn write_u8(&mut self, reg: Register, v: u8) -> Result<(), E> {
        self.0.write(SLAVE_ADDRESS, &[reg as u8, v])
    }
}

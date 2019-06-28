//! A platform agnostic driver to interface with the QMC5883L magnetometer.
//!
//! This driver was built using [`embedded-hal`] traits.
//!
//! [`embedded-hal`]: https://docs.rs/embedded-hal/~0.2

#![deny(missing_docs)]
#![no_std]

extern crate embedded_hal as hal;

use hal::blocking::i2c::{Write, WriteRead};

const I2C_ADDRESS: u8 = 0x0d;

#[allow(dead_code)]
#[allow(non_camel_case_types)]
#[derive(Copy, Clone)]
#[repr(u8)]
enum Register {
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

const STATUS_OVL: u8 = 0b010;
const STATUS_DRDY: u8 = 0b001;

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
pub struct QMC5883L<I2C> {
    i2c: I2C,
}

impl<I2C, E> QMC5883L<I2C>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
{
    /// Creates a new QMC5883L device from an I2C peripheral; begins with a soft reset.
    pub fn new(i2c: I2C) -> Result<Self, Error<E>> {
        let mut dev = QMC5883L { i2c: i2c };
        let id = dev.read_u8(Register::CHIP_ID)?;
        if id != 0xff {
            return Err(Error::InvalidDevice(id));
        }
        dev.reset()?;
        Ok(dev)
    }

    /// Soft reset the device.
    pub fn reset(&mut self) -> Result<(), E> {
        self.write_u8(Register::CONTROL2, CTRL2_SOFT_RST)?;
        self.write_u8(Register::CONTROL2, CTRL2_ROL_PNT | CTRL2_INT_ENB)?;
        self.write_u8(Register::PERIOD, 1)
    }

    /// Set the device field range.
    pub fn set_field_range(&mut self, rng: FieldRange) -> Result<(), E> {
        let ctrl1 = self.read_u8(Register::CONTROL1)?;
        let v = (ctrl1 & !(FieldRange::Range8Gauss as u8)) | (rng as u8);
        self.write_u8(Register::CONTROL1, v)
    }

    /// Set the device oversampling rate.
    pub fn set_oversample(&mut self, osr: OversampleRate) -> Result<(), E> {
        let ctrl1 = self.read_u8(Register::CONTROL1)?;
        let v = (ctrl1 & !(OversampleRate::Rate64 as u8)) | (osr as u8);
        self.write_u8(Register::CONTROL1, v)
    }

    /// Set the device output data rate.
    pub fn set_output_data_rate(&mut self, odr: OutputDataRate) -> Result<(), E> {
        let ctrl1 = self.read_u8(Register::CONTROL1)?;
        let v = (ctrl1 & !(OutputDataRate::Rate200Hz as u8)) | (odr as u8);
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
    pub fn temp(&mut self) -> Result<i16, E> {
        let temp_l = self.read_u8(Register::TOUT_L)? as i16;
        let temp_h = self.read_u8(Register::TOUT_H)? as i16;
        Ok((temp_h << 8) | temp_l)
    }

    /// Read raw (x,y,z) from magnetometer.
    pub fn mag(&mut self) -> Result<(i16, i16, i16), Error<E>> {
        let buf: &mut [u8; 7] = &mut [0; 7];
        self.i2c
            .write_read(I2C_ADDRESS, &[Register::STATUS as u8], buf)?;
        let status = buf[0];
        if (status & STATUS_DRDY) == 0 {
            return Err(Error::NotReady);
        } else if (status & STATUS_OVL) != 0 {
            return Err(Error::Overflow);
        }
        let x = ((buf[2] as i16) << 8) | (buf[1] as i16);
        let y = ((buf[4] as i16) << 8) | (buf[3] as i16);
        let z = ((buf[6] as i16) << 8) | (buf[5] as i16);
        Ok((x, y, z))
    }

    fn read_u8(&mut self, reg: Register) -> Result<u8, E> {
        let buf: &mut [u8; 1] = &mut [0];
        self.i2c.write_read(I2C_ADDRESS, &[reg as u8], buf)?;
        Ok(buf[0])
    }

    fn write_u8(&mut self, reg: Register, v: u8) -> Result<(), E> {
        self.i2c.write(I2C_ADDRESS, &[reg as u8, v])
    }
}

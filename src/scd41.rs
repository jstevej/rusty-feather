use embedded_hal::blocking::i2c::{Read, Write};
use feather_rp2040::hal::i2c;
use heapless::String;
use ufmt::uwrite;

use crate::i2c::FeatherI2C;
use crate::usb_writer::UsbWriter;

// Note: maximum speed is 100 kHz.

const ADDRESS: u8 = 0x62;

#[non_exhaustive]
pub enum Error {
    I2CError(i2c::Error),
    InvalidChecksum,
}

impl From<i2c::Error> for Error {
    fn from(error: i2c::Error) -> Self {
        Error::I2CError(error)
    }
}

fn calc_crc(data: &[u8], len: usize) -> u8 {
    let mut crc: u8 = 0xff;

    for byte_num in 0..len {
        crc ^= data[byte_num];

        for _bit_num in 0..8 {
            if crc & 0x80 == 0x80 {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc = crc << 1;
            }
        }
    }

    return crc;
}

const MAX_READ_BYTES: usize = 16;

#[allow(dead_code)]
fn read_data_dbg(i2c: &mut FeatherI2C, cmd: u16, buf: &mut [u16], u: &mut UsbWriter) -> Result<(), Error> {
    let mut s: String<64> = String::new();
    let cmd_bytes: [u8; 2] = [((cmd & 0xff00) >> 8) as u8, (cmd & 0x00ff) as u8];
    let _ = i2c.write(ADDRESS, &cmd_bytes)?;

    let mut data_bytes: [u8; MAX_READ_BYTES] = [0; MAX_READ_BYTES];
    let _ = i2c.read(ADDRESS, &mut data_bytes)?;

    for n in 0..buf.len() {
        let word: &[u8] = &data_bytes[3 * n..];
        let crc = calc_crc(&word, 2);
        s.clear();
        let _ = uwrite!(s, "[{}, {}, {}] -> {}", word[0], word[1], word[2], crc);
        u.dbg(s.as_str());
        if crc != word[2] {
            return Err(Error::InvalidChecksum);
        }
        buf[n] = ((word[0] as u16) << 8) | (word[1] as u16);
    }

    Ok(())
}

fn read_data(i2c: &mut FeatherI2C, cmd: u16, buf: &mut [u16]) -> Result<(), Error> {
    let cmd_bytes: [u8; 2] = [((cmd & 0xff00) >> 8) as u8, (cmd & 0x00ff) as u8];
    let _ = i2c.write(ADDRESS, &cmd_bytes)?;

    let mut data_bytes: [u8; 3] = [0; 3];

    for n in 0..buf.len() {
        let _ = i2c.read(ADDRESS, &mut data_bytes)?;
        let crc = calc_crc(&data_bytes, 2);
        if crc != data_bytes[2] {
            return Err(Error::InvalidChecksum);
        }
        buf[n] = ((data_bytes[0] as u16) << 8) | (data_bytes[1] as u16);
    }

    Ok(())
}

fn write_data(i2c: &mut FeatherI2C, cmd: u16, buf: &[u16]) -> Result<(), Error> {
    let cmd_bytes: [u8; 2] = [((cmd & 0xff00) >> 8) as u8, (cmd & 0x00ff) as u8];
    let _ = i2c.write(ADDRESS, &cmd_bytes)?;

    let mut data_bytes: [u8; 3] = [0; 3];

    for w in buf {
        data_bytes[0] = (w >> 8) as u8;
        data_bytes[1] = (w & 0x00ff) as u8;
        let crc = calc_crc(&data_bytes, 2);
        data_bytes[2] = crc;
        let _ = i2c.write(ADDRESS, &data_bytes)?;
    }

    Ok(())
}

#[allow(dead_code)]
pub fn start_periodic_measurement(i2c: &mut FeatherI2C) -> Result<(), Error> {
    let _ = write_data(i2c, 0x21b1, &[])?;
    Ok(())
}

#[allow(dead_code)]
pub struct Measurement {
    pub co2: u16, // QU16.0, units ppm
    pub temp: i16, // Q7.8, units C, range [-10, 60]
    pub rh: u16, // QU0.16
}

#[allow(dead_code)]
pub fn read_measurement(i2c: &mut FeatherI2C, u: &mut UsbWriter) -> Result<Measurement, Error> {
    u.dbg("reading measurements");
    let mut buf: [u16; 3] = [0; 3];
    let _ = read_data_dbg(i2c, 0xec05, &mut buf, u)?;

    let co2 = buf[0]; // QU0.16

    // temp = 175 * temp_raw - 45
    let temp_raw: i16 = buf[1] as i16; // Q0.15
    let temp32: i32 = (175 << 7) * (temp_raw as i32) + (-45 << 23);
    let temp: i16 = (temp32 >> 16) as i16;

    let rh: u16 = buf[2]; // QU0.16

    Ok(Measurement { co2, temp, rh })
}

#[allow(dead_code)]
pub fn stop_periodic_measurement(i2c: &mut FeatherI2C) -> Result<(), Error> {
    let _ = write_data(i2c, 0x3f86, &[])?;
    Ok(())
}

// offset = Toffset (C) * 2^16 / 175
#[allow(dead_code)]
pub fn set_temperature_offset(i2c: &mut FeatherI2C, offset: i16) -> Result<(), Error> {
    let _ = write_data(i2c, 0x241d, &[offset as u16])?;
    Ok(())
}

// Toffset (C) = 175 * offset / 2^16
#[allow(dead_code)]
pub fn get_temperature_offset(i2c: &mut FeatherI2C) -> Result<i16, Error> {
    let mut buf: [u16; 1] = [0; 1];
    let _ = read_data(i2c, 0x2318, &mut buf)?;
    Ok(buf[0] as i16)
}

// altitude in meters
#[allow(dead_code)]
pub fn set_sensor_altitude(i2c: &mut FeatherI2C, altitude: u16) -> Result<(), Error> {
    let _ = write_data(i2c, 0x2427, &[altitude])?;
    Ok(())
}

// altitude in meters
#[allow(dead_code)]
pub fn get_sensor_altitude(i2c: &mut FeatherI2C) -> Result<u16, Error> {
    let mut buf: [u16; 1] = [0; 1];
    let _ = read_data(i2c, 0x2322, &mut buf)?;
    Ok(buf[0])
}

// pressure in Pa / 100
#[allow(dead_code)]
pub fn set_ambient_pressure(i2c: &mut FeatherI2C, pressure: u16) -> Result<(), Error> {
    let _ = write_data(i2c, 0xe000, &[pressure])?;
    Ok(())
}

// target = target CO2 concentration in ppm
#[allow(dead_code)]
pub fn perform_forced_recalibration(i2c: &mut FeatherI2C, target: u16) -> Result<(), Error> {
    let _ = write_data(i2c, 0x362f, &[target])?;
    Ok(())
}

#[allow(dead_code)]
pub fn set_automatic_self_calibration_enabled(i2c: &mut FeatherI2C, enabled: bool) -> Result<(), Error> {
    let word: u16 = match enabled {
        true => 1,
        false => 0
    };
    let _ = write_data(i2c, 0x2416, &[word])?;
    Ok(())
}

#[allow(dead_code)]
pub fn get_automatic_self_calibration_enabled(i2c: &mut FeatherI2C) -> Result<bool, Error> {
    let mut buf: [u16; 1] = [0; 1];
    let _ = read_data(i2c, 0x2313, &mut buf)?;
    Ok(buf[0] != 0)
}

#[allow(dead_code)]
pub fn start_low_power_periodic_measurement(i2c: &mut FeatherI2C) -> Result<(), Error> {
    let _ = write_data(i2c, 0x21ac, &[])?;
    Ok(())
}

#[allow(dead_code)]
pub fn get_data_ready_status(i2c: &mut FeatherI2C, u: &mut UsbWriter) -> Result<bool, Error> {
    u.dbg("reading data ready");
    let mut buf: [u16; 1] = [0; 1];
    let _ = read_data_dbg(i2c, 0xe4b8, &mut buf, u)?;
    Ok(buf[0] & 0x07ff != 0)
}

#[allow(dead_code)]
pub fn persist_settings(i2c: &mut FeatherI2C) -> Result<(), Error> {
    let _ = write_data(i2c, 0x3615, &[])?;
    Ok(())
}

#[allow(dead_code)]
pub fn get_serial_number(i2c: &mut FeatherI2C, u: &mut UsbWriter) -> Result<u64, Error> {
    u.dbg("getting serial number");
    let mut buf: [u16; 3] = [0; 3];
    let _ = read_data_dbg(i2c, 0x3682, &mut buf, u)?;
    Ok(((buf[0] as u64) << 32) | ((buf[1] as u64) << 16) | (buf[2] as u64))
}

// result is true if successful, false if malfunction
#[allow(dead_code)]
pub fn perform_self_test(i2c: &mut FeatherI2C) -> Result<bool, Error> {
    let mut buf: [u16; 1] = [0; 1];
    let _ = read_data(i2c, 0x3639, &mut buf)?;
    Ok(buf[0] == 0)
}

#[allow(dead_code)]
pub fn perform_factory_reset(i2c: &mut FeatherI2C) -> Result<(), Error> {
    let _ = write_data(i2c, 0x3632, &[])?;
    Ok(())
}

#[allow(dead_code)]
pub fn reinit(i2c: &mut FeatherI2C) -> Result<(), Error> {
    let _ = write_data(i2c, 0x3646, &[])?;
    Ok(())
}

#[allow(dead_code)]
pub fn measure_single_shot(i2c: &mut FeatherI2C) -> Result<(), Error> {
    let _ = write_data(i2c, 0x219d, &[])?;
    Ok(())
}

#[allow(dead_code)]
pub fn measure_single_shot_rht_only(i2c: &mut FeatherI2C) -> Result<(), Error> {
    let _ = write_data(i2c, 0x2196, &[])?;
    Ok(())
}


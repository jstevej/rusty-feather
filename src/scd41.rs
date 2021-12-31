use cortex_m::delay::Delay;
use embedded_hal::blocking::i2c::{Read, Write};
use feather_rp2040::hal::i2c;
use heapless::{String, Vec};
use ufmt::uwrite;

use crate::console::status;
use crate::i2c::FeatherI2C;
use crate::parser::{CommandResult, MAX_TOKENS, MSG_SIZE};

// Note: maximum speed is 100 kHz.

const ADDRESS: u8 = 0x62;

#[non_exhaustive]
pub enum Error {
    I2CError(i2c::Error),
    InvalidBufferSize,
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

#[allow(dead_code)]
pub struct Measurement {
    pub co2: u16, // QU16.0, units ppm
    pub temp: i16, // Q7.8, units C, range [-10, 60]
    pub rh: u16, // QU0.16
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

fn read_data_with_delay(delay: &mut Delay, i2c: &mut FeatherI2C, cmd: u16, delay_ms: u32, buf: &mut [u16]) -> Result<(), Error> {
    if buf.len() > 3 {
        return Err(Error::InvalidBufferSize);
    }

    let cmd_bytes: [u8; 2] = [((cmd & 0xff00) >> 8) as u8, (cmd & 0x00ff) as u8];
    let _ = i2c.write(ADDRESS, &cmd_bytes)?;

    delay.delay_ms(delay_ms);

    let mut data_bytes: [u8; 9] = [0; 9];
    let _ = i2c.read(ADDRESS, &mut data_bytes)?;

    for n in 0..buf.len() {
        let chunk = &data_bytes[3 * n..];
        let crc = calc_crc(chunk, 2);
        if crc != chunk[2] {
            return Err(Error::InvalidChecksum);
        }
        buf[n] = ((chunk[0] as u16) << 8) | (chunk[1] as u16);
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

pub fn start_periodic_measurement(i2c: &mut FeatherI2C) -> Result<(), Error> {
    let _ = write_data(i2c, 0x21b1, &[])?;
    Ok(())
}

pub fn read_measurement(delay: &mut Delay, i2c: &mut FeatherI2C) -> Result<Measurement, Error> {
    let mut buf: [u16; 3] = [0; 3];
    let _ = read_data_with_delay(delay, i2c, 0xec05, 5, &mut buf)?;

    let co2 = buf[0]; // QU0.16

    // temp = 175 * temp_raw - 45
    let temp_raw: i16 = buf[1] as i16; // Q0.15
    let temp32: i32 = (175 << 7) * (temp_raw as i32) + (-45 << 23);
    let temp: i16 = (temp32 >> 16) as i16;

    let rh: u16 = buf[2]; // QU0.16

    Ok(Measurement { co2, temp, rh })
}

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
pub fn get_data_ready_status(i2c: &mut FeatherI2C) -> Result<bool, Error> {
    let mut buf: [u16; 1] = [0; 1];
    let _ = read_data(i2c, 0xe4b8, &mut buf)?;
    Ok(buf[0] & 0x07ff != 0)
}

#[allow(dead_code)]
pub fn persist_settings(i2c: &mut FeatherI2C) -> Result<(), Error> {
    let _ = write_data(i2c, 0x3615, &[])?;
    Ok(())
}

#[allow(dead_code)]
pub fn get_serial_number(i2c: &mut FeatherI2C) -> Result<u64, Error> {
    let mut buf: [u16; 3] = [0; 3];
    let _ = read_data(i2c, 0x3682, &mut buf)?;
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

#[derive(PartialEq, Eq)]
enum State {
    Idle,
    PeriodicMeasurement,
    Start,
}

pub struct Scd41 {
    state: State,
}

impl Scd41 {
    pub fn new() -> Scd41 {
        Self { state: State::Start }
    }

    pub fn process(&mut self, i2c: &mut FeatherI2C, tokens: &Vec<&str, MAX_TOKENS>) -> CommandResult {
        if tokens.len() <= 0 || tokens[0] != "scd41" {
            return CommandResult::NotHandled;
        }

        if tokens.len() == 2 && tokens[1] == "getstate" {
            match self.state {
                State::Idle => CommandResult::Info(String::from("Idle")),
                State::PeriodicMeasurement => CommandResult::Info(String::from("PeriodicMeasurement")),
                State::Start => CommandResult::Info(String::from("Start")),
            }
        } else if tokens.len() == 2 && tokens[1] == "start" {
            match self.state {
                State::Idle => {
                    match start_periodic_measurement(i2c) {
                        Ok(_) => {
                            self.state = State::PeriodicMeasurement;
                            CommandResult::Handled
                        },
                        Err(_) => CommandResult::Error("failed starting periodic measurement"),
                    }
                },
                _ => CommandResult::Error("invalid state")
            }
        } else if tokens.len() == 2 && tokens[1] == "stop" {
            match self.state {
                State::PeriodicMeasurement => {
                    match stop_periodic_measurement(i2c) {
                        Ok(_) => {
                            self.state = State::Idle;
                            CommandResult::Handled
                        },
                        Err(_) => CommandResult::Error("failed stoping periodic measurement"),
                    }
                },
                _ => CommandResult::Error("invalid state")
            }
        } else {
            return CommandResult::Error("invalid arguments");
        }
    }

    pub fn service(&mut self, delay: &mut Delay, i2c: &mut FeatherI2C) {
        match self.state {
            State::Start => {
                status("scd41: initializing");
                delay.delay_ms(1200);
                self.set_state(State::Idle);

                match stop_periodic_measurement(i2c) {
                    Ok(_) => {},
                    Err(_) => status("scd41: failed stopping periodic measurement"),
                }
                delay.delay_ms(500);

                match start_periodic_measurement(i2c) {
                    Ok(_) => {
                        self.set_state(State::PeriodicMeasurement);
                    },
                    Err(_) => {
                        status("scd41: failed starting periodic measurement");
                    },
                }
            },
            State::PeriodicMeasurement => {
                match get_data_ready_status(i2c) {
                    Ok(true) => {
                        match read_measurement(delay, i2c) {
                            Ok(meas) => {
                                let mut s: String<MSG_SIZE> = String::new();
                                let _ = uwrite!(s, "scd41: CO2: {} ppm", meas.co2);
                                status(s.as_str());
                            },
                            Err(_) => {
                                status("scd41: failed reading periodic measurement");
                            },
                        }
                    },
                    Ok(false) => {
                        status("scd41: data not ready");
                    },
                    Err(_) => {
                        status("scd41: failed getting data ready status");
                    }
                }
            },
            State::Idle => {},
        }
    }

    fn set_state(&mut self, new_state: State) {
        if self.state != new_state {
            self.state = new_state;

            match self.state {
                State::Idle => status("scd41: state: Idle"),
                State::PeriodicMeasurement => status("scd41: state: PeriodicMeasurement"),
                State::Start => status("scd41: state: Start"),
            }
        }
    }
}

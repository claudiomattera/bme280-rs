// Copyright Claudio Mattera 2022.
//
// Distributed under the MIT License or the Apache 2.0 License at your option.
// See the accompanying files License-MIT.txt and License-Apache-2.0.txt, or
// online at
// https://opensource.org/licenses/MIT
// https://opensource.org/licenses/Apache-2.0

use log::{debug, warn};

use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::blocking::i2c::{Read, Write, WriteRead};

use crate::calibration;
use crate::{CalibrationData, Configuration, Status};

/// Default I²C address
pub const DEFAULT_ADDRESS: u8 = 0x76;

/// Hardcoded chip ID
pub const CHIP_ID: u8 = 0x60;

const BME280_REGISTER_CHIPID: u8 = 0xD0;
const BME280_REGISTER_SOFTRESET: u8 = 0xE0;
const BME280_REGISTER_CONTROLHUMID: u8 = 0xF2;
const BME280_REGISTER_STATUS: u8 = 0xF3;
const BME280_REGISTER_CONTROL: u8 = 0xF4;
const BME280_REGISTER_CONFIG: u8 = 0xF5;
const BME280_REGISTER_PRESSUREDATA: u8 = 0xF7;
const BME280_REGISTER_TEMPDATA: u8 = 0xFA;
const BME280_REGISTER_HUMIDDATA: u8 = 0xFD;

const BME280_COMMAND_SOFTRESET: u8 = 0xB6;

const MODE_SLEEP: u8 = 0b00;

/// A full sample: temperature, pressure and humidity
///
/// Disabled measures are `None`.
pub type Sample = (Option<f32>, Option<f32>, Option<f32>);

/// A full raw sample before compensation: temperature, pressure and humidity
///
/// Disabled measures are `None`.
type RawSample = (Option<u32>, Option<u32>, Option<u16>);

/// Interface to BME280 sensor over I²C
///
/// ```no_run
/// # use embedded_hal_mock::delay::MockNoop as DelayMock;
/// # use embedded_hal_mock::i2c::{Mock as I2cMock};
/// use bme280::{Bme280, Configuration, Oversampling, SensorMode};
/// # let i2c = I2cMock::new(&[]);
/// # let delay = DelayMock;
///
/// let mut bme280 = Bme280::new(i2c, delay);
///
/// bme280.init()?;
///
/// let configuration = Configuration::default()
///     .with_temperature_oversampling(Oversampling::Oversample1)
///     .with_pressure_oversampling(Oversampling::Oversample1)
///     .with_humidity_oversampling(Oversampling::Oversample1)
///     .with_sensor_mode(SensorMode::Normal);
/// bme280.set_sampling_configuration(configuration)?;
///
/// if let Some(temperature) = bme280.read_temperature()? {
///     println!("Temperature: {} C", temperature);
/// } else {
///     println!("Temperature reading was disabled");
/// }
/// # Ok::<(), Box<dyn std::error::Error>>(())
/// ```
pub struct Bme280<I2c, Delay> {
    i2c: I2c,
    address: u8,
    delay: Delay,
    coefficients: CalibrationData,
    configuration: Configuration,
}

impl<I2C, D, E> Bme280<I2C, D>
where
    I2C: Read<Error = E> + Write<Error = E> + WriteRead<Error = E>,
    D: DelayMs<u32>,
{
    /// Create a new sensor using an I²C interface and a delay function using
    /// the sensor's default address [`DEFAULT_ADDRESS`])
    pub fn new(i2c: I2C, delay: D) -> Self {
        Self::new_with_address(i2c, DEFAULT_ADDRESS, delay)
    }

    /// Release the I²C interface
    pub fn release(self) -> I2C {
        self.i2c
    }

    /// Create a new sensor using an I²C interface and a delay function
    pub fn new_with_address(i2c: I2C, address: u8, delay: D) -> Self {
        Self::new_with_coefficients(i2c, address, delay, CalibrationData::default())
    }

    fn new_with_coefficients(
        i2c: I2C,
        address: u8,
        delay: D,
        coefficients: CalibrationData,
    ) -> Self {
        debug!("Creating new BME280 device at address 0x{:x}", address);
        Self {
            i2c,
            address: address as u8,
            delay,
            coefficients,
            configuration: Configuration::default(),
        }
    }

    /// Initialize the sensor
    ///
    /// Send a soft-reset signal, obtain the calibration coefficients, and set
    /// default sampling configuration.
    ///
    /// Note that the default sampling configuration disables measurement of
    /// temperature, pressure and humidity.
    pub fn init(&mut self) -> Result<(), E> {
        debug!("Sending soft-reset signal");
        self.write_u8(BME280_REGISTER_SOFTRESET, BME280_COMMAND_SOFTRESET)?;

        debug!("Waiting 10 ms");
        self.delay.delay_ms(10);

        while self.status()?.is_calibrating() {
            debug!("Calibration not complete, waiting 10 ms");
            self.delay.delay_ms(10);
        }

        debug!("Reading coefficients");
        self.read_calibration_coefficients()?;

        debug!("Set sampling");
        let configuration = Configuration::default();
        self.set_sampling_configuration(configuration)?;

        debug!("Waiting 100 ms");
        self.delay.delay_ms(100);

        Ok(())
    }

    /// Obtain the chip id
    ///
    /// The chip id is always [`CHIP_ID`], so this function can be used
    /// to validate that communication with the chip works fine.
    pub fn chip_id(&mut self) -> Result<u8, E> {
        debug!("Read chip id");
        let chip_id = self.read_u8(BME280_REGISTER_CHIPID)?;

        Ok(chip_id)
    }

    /// Obtain the chip status
    pub fn status(&mut self) -> Result<Status, E> {
        debug!("Read chip status");
        let status = self.read_u8(BME280_REGISTER_STATUS)?.into();

        Ok(status)
    }

    /// Set the sampling configuration
    pub fn set_sampling_configuration(&mut self, configuration: Configuration) -> Result<(), E> {
        self.configuration = configuration;

        let (config, ctrl_meas, ctrl_hum) = self.configuration.to_lowlevel_configuration();

        // making sure sensor is in sleep mode before setting configuration
        // as it otherwise may be ignored
        self.write_u8(BME280_REGISTER_CONTROL, MODE_SLEEP)?;

        // you must make sure to also set REGISTER_CONTROL after setting the
        // CONTROLHUMID register, otherwise the values won't be applied (see
        // DS 5.4.3)
        self.write_u8(BME280_REGISTER_CONTROLHUMID, ctrl_hum.into())?;
        self.write_u8(BME280_REGISTER_CONFIG, config.into())?;
        self.write_u8(BME280_REGISTER_CONTROL, ctrl_meas.into())?;

        Ok(())
    }

    /// Take a forced measurement
    ///
    /// When the chip is set to work in forced mode, it goes back to sleep
    /// after every measurement.
    /// It must be set again to forced mode in order to force a new
    /// measurement.
    pub fn take_forced_measurement(&mut self) -> Result<bool, E> {
        if self.configuration.is_forced() {
            debug!("Forcing taking a measurement");

            let (_config, ctrl_meas, _ctrl_hum) = self.configuration.to_lowlevel_configuration();
            self.write_u8(BME280_REGISTER_CONTROL, ctrl_meas.into())?;

            for _ in 0..10 {
                if !self.status()?.is_measuring() {
                    break;
                }

                debug!("Measuring not complete, waiting 10 ms");
                self.delay.delay_ms(10);
            }

            Ok(true)
        } else {
            Ok(false)
        }
    }

    fn read_raw_sample(&mut self) -> Result<RawSample, E> {
        let buffer: [u8; 1] = [BME280_REGISTER_PRESSUREDATA];
        let mut buf: [u8; 8] = [0; 8];
        self.i2c.write_read(self.address, &buffer, &mut buf)?;

        // msb [7:0] = p[19:12]
        // lsb [7:0] = p[11:4]
        // xlsb[7:4] = p[3:0]
        let adc_p: u32 = ((buf[0] as u32) << 12) | ((buf[1] as u32) << 4) | ((buf[2] as u32) >> 4);
        // msb [7:0] = t[19:12]
        // lsb [7:0] = t[11:4]
        // xlsb[7:4] = t[3:0]
        let adc_t: u32 = ((buf[3] as u32) << 12) | ((buf[4] as u32) << 4) | ((buf[5] as u32) >> 4);
        // msb [7:0] = h[15:8]
        // lsb [7:0] = h[7:0]
        let adc_h: u16 = ((buf[6] as u16) << 8) | (buf[7] as u16);

        Ok((
            if adc_t == 0x80000 { None } else { Some(adc_t) },
            if adc_p == 0x80000 { None } else { Some(adc_p) },
            if adc_h == 0x8000 { None } else { Some(adc_h) },
        ))
    }

    /// Read a sample of temperature, pressure and humidity
    ///
    /// Measures that are disabled in the sampling configuration have value
    /// `None`.
    pub fn read_sample(&mut self) -> Result<Sample, E> {
        let (adc_t, adc_p, adc_h) = self.read_raw_sample()?;

        if let Some(adc_t) = adc_t {
            let t_fine = self.coefficients.compensate_temperature(adc_t);
            let t = Some(self.temperature_fine_to_temperature(t_fine));
            let p = adc_p.map(|adc_p| self.coefficients.compensate_pressure(adc_p, t_fine));
            let h = adc_h.map(|adc_h| self.coefficients.compensate_humidity(adc_h, t_fine));

            let temperature = t;
            let pressure = p.map(|p| p as f32 / 256.0);
            let humidity = h.map(|h| h as f32 / 1024.0);

            Ok((temperature, pressure, humidity))
        } else {
            warn!("Temperature measurement is disabled");

            Ok((None, None, None))
        }
    }

    fn temperature_fine_to_temperature(&mut self, t_fine: u32) -> f32 {
        let t = (t_fine * 5 + 128) >> 8;
        t as f32 / 100.0
    }

    /// Read a sample of temperature
    ///
    /// If temperature is disabled in the sampling configuration, return `None`.
    pub fn read_temperature(&mut self) -> Result<Option<f32>, E> {
        if let Some(t_fine) = self.read_temperature_fine()? {
            Ok(Some(self.temperature_fine_to_temperature(t_fine)))
        } else {
            Ok(None)
        }
    }

    fn read_temperature_fine(&mut self) -> Result<Option<u32>, E> {
        let adc_t = self.read_raw_temperature()?;
        let t_fine = adc_t.map(|adc_t| self.coefficients.compensate_temperature(adc_t));
        Ok(t_fine)
    }

    fn read_raw_temperature(&mut self) -> Result<Option<u32>, E> {
        let adc_t = self.read_u24(BME280_REGISTER_TEMPDATA)?;

        if adc_t == 0x80000 {
            Ok(None)
        } else {
            Ok(Some(adc_t))
        }
    }

    /// Read a sample of pressure
    ///
    /// The temperature value, necessary to compute the compensated pressure
    /// value, is also read from the sensor.
    ///
    /// If pressure is disabled in the sampling configuration, return `None`.
    pub fn read_pressure(&mut self) -> Result<Option<f32>, E> {
        if let Some(t_fine) = self.read_temperature_fine()? {
            self.read_pressure_with_temperature_fine(t_fine)
        } else {
            warn!("Temperature measurement is disabled");
            Ok(None)
        }
    }

    /// Read a sample of pressure with a user-specified temperature value
    ///
    /// Unlike in [`Self::read_pressure()`], this function does not take the
    /// temperature from the sensor, so it can be used if the temperature
    /// is already known.
    ///
    /// If pressure is disabled in the sampling configuration, return `None`.
    pub fn read_pressure_with_temperature(&mut self, temperature: f32) -> Result<Option<f32>, E> {
        let t = (temperature * 100.0) as u32;
        let t_fine = ((t << 8) - 128) / 5;
        self.read_pressure_with_temperature_fine(t_fine)
    }

    fn read_pressure_with_temperature_fine(&mut self, t_fine: u32) -> Result<Option<f32>, E> {
        let adc_p = self.read_raw_pressure()?;
        let p = adc_p.map(|adc_p| {
            let p = self.coefficients.compensate_pressure(adc_p, t_fine);
            p as f32 / 256.0
        });
        Ok(p)
    }

    fn read_raw_pressure(&mut self) -> Result<Option<u32>, E> {
        let adc_p = self.read_u24(BME280_REGISTER_PRESSUREDATA)?;

        if adc_p == 0x80000 {
            Ok(None)
        } else {
            Ok(Some(adc_p))
        }
    }

    /// Read a sample of humidity
    ///
    /// The temperature value, necessary to compute the compensated pressure
    /// value, is also read from the sensor.
    ///
    /// If humidity is disabled in the sampling configuration, return `None`.
    pub fn read_humidity(&mut self) -> Result<Option<f32>, E> {
        if let Some(t_fine) = self.read_temperature_fine()? {
            self.read_humidity_with_temperature_fine(t_fine)
        } else {
            warn!("Temperature measurement is disabled");
            Ok(None)
        }
    }

    /// Read a sample of humidity with a user-specified temperature value
    ///
    /// Unlike in [`Self::read_humidity()`], this function does not take the
    /// temperature from the sensor, so it can be used if the temperature
    /// is already known.
    ///
    /// If humidity is disabled in the sampling configuration, return `None`.
    pub fn read_humidity_with_temperature(&mut self, temperature: f32) -> Result<Option<f32>, E> {
        let t = (temperature * 100.0) as u32;
        let t_fine = ((t << 8) - 128) / 5;
        self.read_humidity_with_temperature_fine(t_fine)
    }

    fn read_humidity_with_temperature_fine(&mut self, t_fine: u32) -> Result<Option<f32>, E> {
        let adc_h = self.read_raw_humidity()?;
        let h = adc_h.map(|adc_h| {
            let h = self.coefficients.compensate_humidity(adc_h, t_fine);
            h as f32 / 1024.0
        });
        Ok(h)
    }

    fn read_raw_humidity(&mut self) -> Result<Option<u16>, E> {
        let adc_h = self.read_u16(BME280_REGISTER_HUMIDDATA)?;

        if adc_h == 0x8000 {
            Ok(None)
        } else {
            Ok(Some(adc_h))
        }
    }

    fn read_calibration_coefficients(&mut self) -> Result<(), E> {
        let buffer: [u8; 1] = [calibration::FIRST_REGISTER];

        let mut out: [u8; calibration::TOTAL_LENGTH] = [0; calibration::TOTAL_LENGTH];
        self.i2c.write_read(
            self.address,
            &buffer,
            &mut out[0..calibration::FIRST_LENGTH],
        )?;

        let buffer: [u8; 1] = [calibration::SECOND_REGISTER];
        self.i2c.write_read(
            self.address,
            &buffer,
            &mut out[calibration::FIRST_LENGTH as usize..calibration::TOTAL_LENGTH],
        )?;

        self.coefficients = (&out).into();

        Ok(())
    }

    fn write_u8(&mut self, register: u8, value: u8) -> Result<(), E> {
        let buffer: [u8; 2] = [register, value];
        self.i2c.write(self.address, &buffer)?;
        Ok(())
    }

    fn read_u8(&mut self, register: u8) -> Result<u8, E> {
        let buffer: [u8; 1] = [register];
        let mut output_buffer: [u8; 1] = [0];
        self.i2c
            .write_read(self.address, &buffer, &mut output_buffer)?;
        Ok(output_buffer[0])
    }

    fn read_u16(&mut self, register: u8) -> Result<u16, E> {
        let buffer: [u8; 1] = [register];
        let mut output_buffer: [u8; 2] = [0, 0];
        self.i2c
            .write_read(self.address, &buffer, &mut output_buffer)?;
        Ok((output_buffer[0] as u16) << 8 | (output_buffer[1] as u16))
    }

    fn read_u24(&mut self, register: u8) -> Result<u32, E> {
        let buffer: [u8; 1] = [register];
        let mut output_buffer: [u8; 3] = [0, 0, 0];
        self.i2c
            .write_read(self.address, &buffer, &mut output_buffer)?;
        Ok((output_buffer[0] as u32) << 12
            | (output_buffer[1] as u32) << 4
            | (output_buffer[2] as u32) >> 4)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use embedded_hal_mock::delay::MockNoop as DelayMock;
    use embedded_hal_mock::i2c::{Mock as I2cMock, Transaction as I2cTransaction};

    use calibration::TEST_CALIBRATION_DATA;

    #[test]
    fn test_chip_id() -> Result<(), Box<dyn std::error::Error>> {
        let expectations = [I2cTransaction::write_read(
            DEFAULT_ADDRESS,
            vec![BME280_REGISTER_CHIPID],
            vec![CHIP_ID],
        )];
        let i2c = I2cMock::new(&expectations);

        let mut bme280 = Bme280::new(i2c, DelayMock);

        let chip_id = bme280.chip_id()?;

        assert_eq!(chip_id, CHIP_ID);

        Ok(())
    }

    #[test]
    fn test_chip_status() -> Result<(), Box<dyn std::error::Error>> {
        let expectations = [I2cTransaction::write_read(
            DEFAULT_ADDRESS,
            vec![BME280_REGISTER_STATUS],
            vec![0x00],
        )];
        let i2c = I2cMock::new(&expectations);

        let mut bme280 = Bme280::new(i2c, DelayMock);

        let status = bme280.status()?;

        assert!(!status.is_measuring());
        assert!(!status.is_calibrating());

        Ok(())
    }

    #[test]
    fn test_chip_status_measuring() -> Result<(), Box<dyn std::error::Error>> {
        let expectations = [I2cTransaction::write_read(
            DEFAULT_ADDRESS,
            vec![BME280_REGISTER_STATUS],
            vec![0b0000_0100],
        )];
        let i2c = I2cMock::new(&expectations);

        let mut bme280 = Bme280::new(i2c, DelayMock);

        let status = bme280.status()?;

        assert!(status.is_measuring());
        assert!(!status.is_calibrating());

        Ok(())
    }

    #[test]
    fn test_read_temperature_disabled() -> Result<(), Box<dyn std::error::Error>> {
        let expectations = [I2cTransaction::write_read(
            DEFAULT_ADDRESS,
            vec![BME280_REGISTER_TEMPDATA],
            vec![0x80, 0x00, 0x00],
        )];
        let i2c = I2cMock::new(&expectations);

        let mut bme280 = Bme280::new_with_coefficients(
            i2c,
            DEFAULT_ADDRESS,
            DelayMock,
            TEST_CALIBRATION_DATA.clone(),
        );

        let expected = None;

        let temperature = bme280.read_temperature()?;

        assert_eq!(temperature, expected);

        Ok(())
    }

    #[test]
    fn test_read_temperature() -> Result<(), Box<dyn std::error::Error>> {
        let expectations = [I2cTransaction::write_read(
            DEFAULT_ADDRESS,
            vec![BME280_REGISTER_TEMPDATA],
            vec![0x84, 0x47, 0x00],
        )];
        let i2c = I2cMock::new(&expectations);

        let mut bme280 = Bme280::new_with_coefficients(
            i2c,
            DEFAULT_ADDRESS,
            DelayMock,
            TEST_CALIBRATION_DATA.clone(),
        );

        let expected = Some(27.33);

        let temperature = bme280.read_temperature()?;

        assert_eq!(temperature, expected);

        Ok(())
    }

    #[test]
    fn test_read_pressure_disabled() -> Result<(), Box<dyn std::error::Error>> {
        let expectations = [
            I2cTransaction::write_read(
                DEFAULT_ADDRESS,
                vec![BME280_REGISTER_TEMPDATA],
                vec![0x84, 0x47, 0x00],
            ),
            I2cTransaction::write_read(
                DEFAULT_ADDRESS,
                vec![BME280_REGISTER_PRESSUREDATA],
                vec![0x80, 0x00, 0x00],
            ),
        ];
        let i2c = I2cMock::new(&expectations);

        let mut bme280 = Bme280::new_with_coefficients(
            i2c,
            DEFAULT_ADDRESS,
            DelayMock,
            TEST_CALIBRATION_DATA.clone(),
        );

        let expected = None;

        let pressure = bme280.read_pressure()?;

        assert_eq!(pressure, expected);

        Ok(())
    }

    #[test]
    fn test_read_pressure() -> Result<(), Box<dyn std::error::Error>> {
        let expectations = [
            I2cTransaction::write_read(
                DEFAULT_ADDRESS,
                vec![BME280_REGISTER_TEMPDATA],
                vec![0x84, 0x47, 0x00],
            ),
            I2cTransaction::write_read(
                DEFAULT_ADDRESS,
                vec![BME280_REGISTER_PRESSUREDATA],
                vec![0x4f, 0x50, 0x00],
            ),
        ];
        let i2c = I2cMock::new(&expectations);

        let mut bme280 = Bme280::new_with_coefficients(
            i2c,
            DEFAULT_ADDRESS,
            DelayMock,
            TEST_CALIBRATION_DATA.clone(),
        );

        let expected = Some(101_233.016);

        let pressure = bme280.read_pressure()?;

        assert_eq!(pressure, expected);

        Ok(())
    }

    #[test]
    fn test_read_humidity_disabled() -> Result<(), Box<dyn std::error::Error>> {
        let expectations = [
            I2cTransaction::write_read(
                DEFAULT_ADDRESS,
                vec![BME280_REGISTER_TEMPDATA],
                vec![0x84, 0x47, 0x00],
            ),
            I2cTransaction::write_read(
                DEFAULT_ADDRESS,
                vec![BME280_REGISTER_HUMIDDATA],
                vec![0x80, 0x00],
            ),
        ];
        let i2c = I2cMock::new(&expectations);

        let mut bme280 = Bme280::new_with_coefficients(
            i2c,
            DEFAULT_ADDRESS,
            DelayMock,
            TEST_CALIBRATION_DATA.clone(),
        );

        let expected = None;

        let humidity = bme280.read_humidity()?;

        assert_eq!(humidity, expected);

        Ok(())
    }

    #[test]
    fn test_read_humidity() -> Result<(), Box<dyn std::error::Error>> {
        let expectations = [
            I2cTransaction::write_read(
                DEFAULT_ADDRESS,
                vec![BME280_REGISTER_TEMPDATA],
                vec![0x84, 0x47, 0x00],
            ),
            I2cTransaction::write_read(
                DEFAULT_ADDRESS,
                vec![BME280_REGISTER_HUMIDDATA],
                vec![0x60, 0x02],
            ),
        ];
        let i2c = I2cMock::new(&expectations);

        let mut bme280 = Bme280::new_with_coefficients(
            i2c,
            DEFAULT_ADDRESS,
            DelayMock,
            TEST_CALIBRATION_DATA.clone(),
        );

        let expected = Some(34.854_492);

        let humidity = bme280.read_humidity()?;

        assert_eq!(humidity, expected);

        Ok(())
    }
}

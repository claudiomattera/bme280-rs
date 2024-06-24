// Copyright Claudio Mattera 2022-2024.
//
// Distributed under the MIT License or the Apache 2.0 License at your option.
// See the accompanying files License-MIT.txt and License-Apache-2.0.txt, or
// online at
// https://opensource.org/licenses/MIT
// https://opensource.org/licenses/Apache-2.0

//! Data types and functions for BME280 sensor interface

use log::debug;
use log::warn;

use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::i2c::I2c;

use crate::calibration;
use crate::constants::BME280_COMMAND_SOFTRESET;
use crate::constants::BME280_REGISTER_CHIPID;
use crate::constants::BME280_REGISTER_CONFIG;
use crate::constants::BME280_REGISTER_CONTROL;
use crate::constants::BME280_REGISTER_CONTROLHUMID;
use crate::constants::BME280_REGISTER_HUMIDDATA;
use crate::constants::BME280_REGISTER_PRESSUREDATA;
use crate::constants::BME280_REGISTER_SOFTRESET;
use crate::constants::BME280_REGISTER_STATUS;
use crate::constants::BME280_REGISTER_TEMPDATA;
use crate::constants::DEFAULT_ADDRESS;
use crate::constants::MODE_SLEEP;
use crate::constants::SKIPPED_HUMIDITY_OUTPUT;
use crate::constants::SKIPPED_PRESSURE_OUTPUT;
use crate::constants::SKIPPED_TEMPERATURE_OUTPUT;
use crate::sample::humidity_from_number;
use crate::sample::pressure_from_pascal;
use crate::sample::temperature_from_celsius;
use crate::sample::Humidity;
use crate::sample::Pressure;
use crate::sample::RawSample;
use crate::sample::Sample;
use crate::sample::Temperature;
use crate::CalibrationData;
use crate::Configuration;
use crate::Status;

/// Async interface to BME280 sensor over I²C
pub struct Bme280<I2c, Delay> {
    /// I²C device
    i2c: I2c,

    /// I²C address
    address: u8,

    /// Delay function
    delay: Delay,

    /// Calibration coefficients
    coefficients: CalibrationData,

    /// Sensor configuration
    configuration: Configuration,
}

impl<I2C, D> Bme280<I2C, D>
where
    I2C: I2c,
    D: DelayNs,
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

    /// Create a new sensor with specific calibration coefficients
    fn new_with_coefficients(
        i2c: I2C,
        address: u8,
        delay: D,
        coefficients: CalibrationData,
    ) -> Self {
        debug!("Creating new BME280 device at address 0x{:x}", address);
        Self {
            i2c,
            address,
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
    ///
    /// # Errors
    ///
    /// Return an error if it cannot communicate with the sensor.
    pub async fn init(&mut self) -> Result<(), I2C::Error> {
        debug!("Sending soft-reset signal");
        self.write_u8(BME280_REGISTER_SOFTRESET, BME280_COMMAND_SOFTRESET)
            .await?;

        debug!("Waiting 10 ms");
        self.delay.delay_ms(10).await;

        while self.status().await?.is_calibrating() {
            debug!("Calibration not complete, waiting 10 ms");
            self.delay.delay_ms(10).await;
        }

        debug!("Reading coefficients");
        self.read_calibration_coefficients().await?;

        debug!("Set sampling");
        let configuration = Configuration::default();
        self.set_sampling_configuration(configuration).await?;

        debug!("Waiting 100 ms");
        self.delay.delay_ms(100).await;

        Ok(())
    }

    /// Obtain the chip id
    ///
    /// The chip id is always [`crate::constants::CHIP_ID`], so this function
    /// can be used to validate that communication with the chip works fine.
    ///
    /// # Errors
    ///
    /// Return an error if it cannot communicate with the sensor.
    pub async fn chip_id(&mut self) -> Result<u8, I2C::Error> {
        debug!("Read chip id");
        let chip_id = self.read_u8(BME280_REGISTER_CHIPID).await?;

        Ok(chip_id)
    }

    /// Obtain the chip status
    ///
    /// # Errors
    ///
    /// Return an error if it cannot communicate with the sensor.
    pub async fn status(&mut self) -> Result<Status, I2C::Error> {
        debug!("Read chip status");
        let status = self.read_u8(BME280_REGISTER_STATUS).await?.into();

        Ok(status)
    }

    /// Set the sampling configuration
    ///
    /// # Errors
    ///
    /// Return an error if it cannot communicate with the sensor.
    pub async fn set_sampling_configuration(
        &mut self,
        configuration: Configuration,
    ) -> Result<(), I2C::Error> {
        self.configuration = configuration;

        let (config, ctrl_meas, ctrl_hum) = self.configuration.to_lowlevel_configuration();

        // making sure sensor is in sleep mode before setting configuration
        // as it otherwise may be ignored
        self.write_u8(BME280_REGISTER_CONTROL, MODE_SLEEP).await?;

        // you must make sure to also set REGISTER_CONTROL after setting the
        // CONTROLHUMID register, otherwise the values won't be applied (see
        // DS 5.4.3)
        self.write_u8(BME280_REGISTER_CONTROLHUMID, ctrl_hum.into())
            .await?;
        self.write_u8(BME280_REGISTER_CONFIG, config.into()).await?;
        self.write_u8(BME280_REGISTER_CONTROL, ctrl_meas.into())
            .await?;

        Ok(())
    }

    /// Take a forced measurement
    ///
    /// When the chip is set to work in forced mode, it goes back to sleep
    /// after every measurement.
    /// It must be set again to forced mode in order to force a new
    /// measurement.
    ///
    /// # Errors
    ///
    /// Return an error if it cannot communicate with the sensor.
    pub async fn take_forced_measurement(&mut self) -> Result<bool, I2C::Error> {
        if self.configuration.is_forced() {
            debug!("Forcing taking a measurement");

            let (_config, ctrl_meas, _ctrl_hum) = self.configuration.to_lowlevel_configuration();
            self.write_u8(BME280_REGISTER_CONTROL, ctrl_meas.into())
                .await?;

            for _ in 0..10 {
                if !self.status().await?.is_measuring() {
                    break;
                }

                debug!("Measuring not complete, waiting 10 ms");
                self.delay.delay_ms(10).await;
            }

            Ok(true)
        } else {
            Ok(false)
        }
    }

    /// Read a raw sample from sensor
    ///
    /// Raw sample must be converted to human-readable quantities using
    /// compensation formulas from the data sheet.
    async fn read_raw_sample(&mut self) -> Result<RawSample, I2C::Error> {
        let buffer: [u8; 1] = [BME280_REGISTER_PRESSUREDATA];
        let mut buf: [u8; 8] = [0; 8];
        self.i2c.write_read(self.address, &buffer, &mut buf).await?;

        // msb [7:0] = p[19:12]
        // lsb [7:0] = p[11:4]
        // xlsb[7:4] = p[3:0]
        let adc_p: u32 =
            (u32::from(buf[0]) << 12) | (u32::from(buf[1]) << 4) | (u32::from(buf[2]) >> 4);
        // msb [7:0] = t[19:12]
        // lsb [7:0] = t[11:4]
        // xlsb[7:4] = t[3:0]
        let adc_t: u32 =
            (u32::from(buf[3]) << 12) | (u32::from(buf[4]) << 4) | (u32::from(buf[5]) >> 4);
        // msb [7:0] = h[15:8]
        // lsb [7:0] = h[7:0]
        let adc_h: u16 = (u16::from(buf[6]) << 8) | u16::from(buf[7]);

        Ok(RawSample {
            adc_t: if adc_t == SKIPPED_TEMPERATURE_OUTPUT {
                None
            } else {
                Some(adc_t)
            },
            adc_p: if adc_p == SKIPPED_PRESSURE_OUTPUT {
                None
            } else {
                Some(adc_p)
            },
            adc_h: if adc_h == SKIPPED_HUMIDITY_OUTPUT {
                None
            } else {
                Some(adc_h)
            },
        })
    }

    /// Read a sample of temperature, pressure and humidity
    ///
    /// Measures that are disabled in the sampling configuration have value
    /// `None`.
    ///
    /// # Errors
    ///
    /// Return an error if it cannot communicate with the sensor.
    pub async fn read_sample(&mut self) -> Result<Sample, I2C::Error> {
        let RawSample {
            adc_t,
            adc_p,
            adc_h,
        } = self.read_raw_sample().await?;

        if let Some(adc_t) = adc_t {
            let t_fine = self.coefficients.compensate_temperature(adc_t);
            let t = Some(Self::temperature_fine_to_temperature(t_fine));
            let p = adc_p.map(|adc_p| self.coefficients.compensate_pressure(adc_p, t_fine));
            let h = adc_h.map(|adc_h| self.coefficients.compensate_humidity(adc_h, t_fine));

            let temperature = t;

            #[allow(clippy::cast_precision_loss)] // Acceptable precision loss
            let pressure = p.map(|p| p as f32 / 256.0);
            let pressure = pressure.map(pressure_from_pascal);
            #[allow(clippy::cast_precision_loss)] // Acceptable precision loss
            let humidity = h.map(|h| h as f32 / 1024.0);
            let humidity = humidity.map(humidity_from_number);

            Ok(Sample {
                temperature,
                pressure,
                humidity,
            })
        } else {
            warn!("Temperature measurement is disabled");

            Ok(Sample::default())
        }
    }

    /// Compute raw temperature from human-readable temperature
    fn temperature_fine_to_temperature(t_fine: i32) -> Temperature {
        let t = (t_fine * 5 + 128) >> 8;

        #[allow(clippy::cast_precision_loss)] // Acceptable precision loss
        let t = t as f32;

        temperature_from_celsius(t / 100.0)
    }

    /// Read a sample of temperature
    ///
    /// If temperature is disabled in the sampling configuration, return `None`.
    ///
    /// # Errors
    ///
    /// Return an error if it cannot communicate with the sensor.
    pub async fn read_temperature(&mut self) -> Result<Option<Temperature>, I2C::Error> {
        if let Some(t_fine) = self.read_temperature_fine().await? {
            Ok(Some(Self::temperature_fine_to_temperature(t_fine)))
        } else {
            Ok(None)
        }
    }

    /// Read temperature from sensor
    async fn read_temperature_fine(&mut self) -> Result<Option<i32>, I2C::Error> {
        let adc_t = self.read_raw_temperature().await?;
        let t_fine = adc_t.map(|adc_t| self.coefficients.compensate_temperature(adc_t));
        Ok(t_fine)
    }

    /// Read raw temperature from sensor
    ///
    /// Raw temperature must be converted to human-readable temperature using
    /// compensation formulas from the data sheet.
    async fn read_raw_temperature(&mut self) -> Result<Option<u32>, I2C::Error> {
        let adc_t = self.read_u24(BME280_REGISTER_TEMPDATA).await?;

        if adc_t == SKIPPED_TEMPERATURE_OUTPUT {
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
    ///
    /// # Errors
    ///
    /// Return an error if it cannot communicate with the sensor.
    pub async fn read_pressure(&mut self) -> Result<Option<Pressure>, I2C::Error> {
        if let Some(t_fine) = self.read_temperature_fine().await? {
            self.read_pressure_with_temperature_fine(t_fine).await
        } else {
            warn!("Pressure measurement is disabled");
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
    ///
    /// # Errors
    ///
    /// Return an error if it cannot communicate with the sensor.
    pub async fn read_pressure_with_temperature(
        &mut self,
        temperature: f32,
    ) -> Result<Option<Pressure>, I2C::Error> {
        #[allow(clippy::cast_possible_truncation)] // Acceptable truncation
        let t = (temperature * 100.0) as i32;
        let t_fine = ((t << 8) - 128) / 5;
        self.read_pressure_with_temperature_fine(t_fine).await
    }

    /// Read pressure from sensor usings pecific temperature reference
    async fn read_pressure_with_temperature_fine(
        &mut self,
        t_fine: i32,
    ) -> Result<Option<Pressure>, I2C::Error> {
        let adc_p = self.read_raw_pressure().await?;
        let p = adc_p.map(|adc_p| {
            let p = self.coefficients.compensate_pressure(adc_p, t_fine);

            #[allow(clippy::cast_precision_loss)] // Acceptable precision loss
            let p = p as f32;

            pressure_from_pascal(p / 256.0)
        });
        Ok(p)
    }

    /// Read raw pressure from sensor
    ///
    /// Raw pressure must be converted to human-readable pressure using
    /// compensation formulas from the data sheet.
    async fn read_raw_pressure(&mut self) -> Result<Option<u32>, I2C::Error> {
        let adc_p = self.read_u24(BME280_REGISTER_PRESSUREDATA).await?;

        if adc_p == SKIPPED_PRESSURE_OUTPUT {
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
    ///
    /// # Errors
    ///
    /// Return an error if it cannot communicate with the sensor.
    pub async fn read_humidity(&mut self) -> Result<Option<Humidity>, I2C::Error> {
        if let Some(t_fine) = self.read_temperature_fine().await? {
            self.read_humidity_with_temperature_fine(t_fine).await
        } else {
            warn!("Humidity measurement is disabled");
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
    ///
    /// # Errors
    ///
    /// Return an error if it cannot communicate with the sensor.
    pub async fn read_humidity_with_temperature(
        &mut self,
        temperature: f32,
    ) -> Result<Option<Humidity>, I2C::Error> {
        #[allow(clippy::cast_possible_truncation)] // Acceptable truncation
        let t = (temperature * 100.0) as i32;
        let t_fine = ((t << 8) - 128) / 5;
        self.read_humidity_with_temperature_fine(t_fine).await
    }

    /// Read humidity from sensor using specific reference temperature
    async fn read_humidity_with_temperature_fine(
        &mut self,
        t_fine: i32,
    ) -> Result<Option<Humidity>, I2C::Error> {
        let adc_h = self.read_raw_humidity().await?;
        let h = adc_h.map(|adc_h| {
            let h = self.coefficients.compensate_humidity(adc_h, t_fine);

            #[allow(clippy::cast_precision_loss)] // Acceptable precision loss
            let h = h as f32;

            humidity_from_number(h / 1024.0)
        });
        Ok(h)
    }

    /// Read raw humidity from sensor
    ///
    /// Raw humidity must be converted to human-readable humidity using
    /// compensation formulas from the data sheet.
    async fn read_raw_humidity(&mut self) -> Result<Option<u16>, I2C::Error> {
        let adc_h = self.read_u16(BME280_REGISTER_HUMIDDATA).await?;

        if adc_h == SKIPPED_HUMIDITY_OUTPUT {
            Ok(None)
        } else {
            Ok(Some(adc_h))
        }
    }

    /// Read calibration coefficients from sensor
    async fn read_calibration_coefficients(&mut self) -> Result<(), I2C::Error> {
        let buffer: [u8; 1] = [calibration::FIRST_REGISTER];

        let mut out: [u8; calibration::TOTAL_LENGTH] = [0; calibration::TOTAL_LENGTH];
        self.i2c
            .write_read(
                self.address,
                &buffer,
                &mut out[0..calibration::FIRST_LENGTH],
            )
            .await?;

        let buffer: [u8; 1] = [calibration::SECOND_REGISTER];
        self.i2c
            .write_read(
                self.address,
                &buffer,
                &mut out[calibration::FIRST_LENGTH..calibration::TOTAL_LENGTH],
            )
            .await?;

        self.coefficients = (&out).into();

        Ok(())
    }

    /// Write an unsigned byte to an I²C register
    async fn write_u8(&mut self, register: u8, value: u8) -> Result<(), I2C::Error> {
        let buffer: [u8; 2] = [register, value];
        self.i2c.write(self.address, &buffer).await?;
        Ok(())
    }

    /// Write an unsigned byte from an I²C register
    async fn read_u8(&mut self, register: u8) -> Result<u8, I2C::Error> {
        let buffer: [u8; 1] = [register];
        let mut output_buffer: [u8; 1] = [0];
        self.i2c
            .write_read(self.address, &buffer, &mut output_buffer)
            .await?;
        Ok(output_buffer[0])
    }

    /// Write two unsigned bytes to an I²C register
    async fn read_u16(&mut self, register: u8) -> Result<u16, I2C::Error> {
        let buffer: [u8; 1] = [register];
        let mut output_buffer: [u8; 2] = [0, 0];
        self.i2c
            .write_read(self.address, &buffer, &mut output_buffer)
            .await?;
        Ok(u16::from(output_buffer[0]) << 8 | u16::from(output_buffer[1]))
    }

    /// Write three unsigned bytes to an I²C register
    async fn read_u24(&mut self, register: u8) -> Result<u32, I2C::Error> {
        let buffer: [u8; 1] = [register];
        let mut output_buffer: [u8; 3] = [0, 0, 0];
        self.i2c
            .write_read(self.address, &buffer, &mut output_buffer)
            .await?;
        Ok(u32::from(output_buffer[0]) << 12
            | u32::from(output_buffer[1]) << 4
            | u32::from(output_buffer[2]) >> 4)
    }
}

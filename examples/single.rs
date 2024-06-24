// Copyright Claudio Mattera 2024.
//
// Distributed under the MIT License or the Apache 2.0 License at your option.
// See the accompanying files License-MIT.txt and License-Apache-2.0.txt, or
// online at
// https://opensource.org/licenses/MIT
// https://opensource.org/licenses/Apache-2.0

//! Read data from BME280 sensor through a FT232H board

use std::error::Error;

use env_logger::init as init_logger;

use log::info;

use embedded_hal::delay::DelayNs as _;
use embedded_hal::i2c::I2c;

use ftdi_embedded_hal::Delay;
use ftdi_embedded_hal::FtHal;

use ftdi::find_by_vid_pid;
use ftdi::Interface as FtdiInterface;

use uom::si::pressure::hectopascal;
use uom::si::ratio::percent;
use uom::si::thermodynamic_temperature::degree_celsius;

use bme280_rs::Bme280;
use bme280_rs::Configuration;
use bme280_rs::Oversampling;
use bme280_rs::SensorMode;

/// Main entry point
fn main() -> Result<(), Box<dyn Error>> {
    init_logger();

    info!("Create FTDI device");
    let device = find_by_vid_pid(0x0403, 0x6014)
        .interface(FtdiInterface::A)
        .open()?;

    info!("Initialize I²C bus");
    let hal = FtHal::init_freq(device, 400_000)?;
    let i2c = hal.i2c()?;

    let _i2c = handle_bme280(i2c)?;

    Ok(())
}

/// Execute operations on sensor BME280
fn handle_bme280<I2C>(i2c: I2C) -> Result<I2C, I2C::Error>
where
    I2C: I2c,
{
    let mut delay = Delay::new();

    let mut sensor = Bme280::new(i2c, delay);

    sensor.init()?;

    sensor.set_sampling_configuration(
        Configuration::default()
            .with_temperature_oversampling(Oversampling::Oversample1)
            .with_pressure_oversampling(Oversampling::Oversample1)
            .with_humidity_oversampling(Oversampling::Oversample1)
            .with_sensor_mode(SensorMode::Normal),
    )?;

    delay.delay_ms(10);

    let sample = sensor.read_sample()?;

    let temperature = sample
        .temperature
        .map(|t| t.get::<degree_celsius>())
        .unwrap_or(0.0);
    let humidity = sample.humidity.map(|t| t.get::<percent>()).unwrap_or(0.0);
    let pressure = sample
        .pressure
        .map(|t| t.get::<hectopascal>())
        .unwrap_or(0.0);

    info!("Sample: ┳ Temperature: {} C", temperature);
    info!("        ┣ Humidity: {} %", humidity);
    info!("        ┗ Pressure: {} hPa", pressure);

    let i2c = sensor.release();

    Ok(i2c)
}

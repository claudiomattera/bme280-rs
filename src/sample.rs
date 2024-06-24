// Copyright Claudio Mattera 2022-2024.
//
// Distributed under the MIT License or the Apache 2.0 License at your option.
// See the accompanying files License-MIT.txt and License-Apache-2.0.txt, or
// online at
// https://opensource.org/licenses/MIT
// https://opensource.org/licenses/Apache-2.0

//! Data types and functions for BME280 sensor samples

#[cfg(feature = "uom")]
use uom::si::f32::Pressure as UomPressure;
#[cfg(feature = "uom")]
use uom::si::f32::Ratio as UomHumidity;
#[cfg(feature = "uom")]
use uom::si::f32::ThermodynamicTemperature as UomTemperature;
#[cfg(feature = "uom")]
use uom::si::pressure::pascal;
#[cfg(feature = "uom")]
use uom::si::ratio::percent;
#[cfg(feature = "uom")]
use uom::si::thermodynamic_temperature::degree_celsius;

#[cfg(feature = "uom")]
/// Type for temperature measurements
pub type Temperature = UomTemperature;

#[cfg(feature = "uom")]
/// Type for pressure measurements
pub type Pressure = UomPressure;

#[cfg(feature = "uom")]
/// Type for humidity measurements
pub type Humidity = UomHumidity;

#[cfg(feature = "uom")]
/// Convert a raw value in Celsius to a temperature
pub(crate) fn temperature_from_celsius(raw: f32) -> Temperature {
    Temperature::new::<degree_celsius>(raw)
}

#[cfg(feature = "uom")]
/// Convert a raw value in Pascal to a pressure
pub(crate) fn pressure_from_pascal(raw: f32) -> Pressure {
    Pressure::new::<pascal>(raw)
}

#[cfg(feature = "uom")]
/// Convert a raw value to a humidity
pub(crate) fn humidity_from_number(raw: f32) -> Humidity {
    Humidity::new::<percent>(raw)
}

#[cfg(not(feature = "uom"))]
/// Type for pressure measurements
pub type Pressure = f32;

#[cfg(not(feature = "uom"))]
/// Type for temperature measurements
pub type Temperature = f32;

#[cfg(not(feature = "uom"))]
/// Type for temperature measurements
pub type Humidity = f32;

#[cfg(not(feature = "uom"))]
/// Convert a raw value in Celsius to a temperature
pub(crate) fn temperature_from_celsius(raw: f32) -> Temperature {
    raw
}

#[cfg(not(feature = "uom"))]
/// Convert a raw value in Pascal to a pressure
pub(crate) fn pressure_from_pascal(raw: f32) -> Pressure {
    raw
}

#[cfg(not(feature = "uom"))]
/// Convert a raw value to a humidity
pub(crate) fn humidity_from_number(raw: f32) -> Humidity {
    raw
}

/// A full sample: temperature, pressure and humidity
///
/// Disabled measures are `None`.
#[derive(Debug, Default)]
pub struct Sample {
    /// Temperature reading
    pub temperature: Option<Temperature>,

    /// Pressure reading
    pub pressure: Option<Pressure>,

    /// Humidity reading
    pub humidity: Option<Humidity>,
}

/// A full raw sample before compensation: temperature, pressure and humidity
///
/// Disabled measures are `None`.
#[derive(Debug, Default)]
#[allow(clippy::struct_field_names)]
pub(crate) struct RawSample {
    /// Temperature raw reading
    pub(crate) adc_t: Option<u32>,

    /// Pressure raw reading
    pub(crate) adc_p: Option<u32>,

    /// Humidity raw reading
    pub(crate) adc_h: Option<u16>,
}

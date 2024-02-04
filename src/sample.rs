// Copyright Claudio Mattera 2022-2024.
//
// Distributed under the MIT License or the Apache 2.0 License at your option.
// See the accompanying files License-MIT.txt and License-Apache-2.0.txt, or
// online at
// https://opensource.org/licenses/MIT
// https://opensource.org/licenses/Apache-2.0

//! Data types and functions for BME280 sensor samples

/// A full sample: temperature, pressure and humidity
///
/// Disabled measures are `None`.
#[derive(Debug, Default)]
pub struct Sample {
    /// Temperature reading
    pub temperature: Option<f32>,

    /// Pressure reading
    pub pressure: Option<f32>,

    /// Humidity reading
    pub humidity: Option<f32>,
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

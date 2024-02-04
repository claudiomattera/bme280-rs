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
pub type Sample = (Option<f32>, Option<f32>, Option<f32>);

/// A full raw sample before compensation: temperature, pressure and humidity
///
/// Disabled measures are `None`.
pub(crate) type RawSample = (Option<u32>, Option<u32>, Option<u16>);

// Copyright Claudio Mattera 2022-2024.
//
// Distributed under the MIT License or the Apache 2.0 License at your option.
// See the accompanying files License-MIT.txt and License-Apache-2.0.txt, or
// online at
// https://opensource.org/licenses/MIT
// https://opensource.org/licenses/Apache-2.0

#![cfg_attr(not(doctest), doc = include_str!("../README.md"))]
#![cfg_attr(not(test), no_std)]

mod r#async;
pub use self::r#async::Bme280 as AsyncBme280;

mod bme280;
pub use self::bme280::Bme280;

mod calibration;
use self::calibration::CalibrationData;

mod configuration;
pub use self::configuration::{
    Configuration, Filter, Oversampling, SensorMode, StandbyTime, Status,
};

mod constants;
pub use self::constants::{CHIP_ID, DEFAULT_ADDRESS};

mod sample;
pub use self::sample::Sample;

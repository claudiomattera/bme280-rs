// Copyright Claudio Mattera 2022.
//
// Distributed under the MIT License or the Apache 2.0 License at your option.
// See the accompanying files License-MIT.txt and License-Apache-2.0.txt, or
// online at
// https://opensource.org/licenses/MIT
// https://opensource.org/licenses/Apache-2.0

#![cfg_attr(not(doctest), doc = include_str!("../Readme.md"))]
#![cfg_attr(not(test), no_std)]

mod bme280;
pub use crate::bme280::{Bme280, CHIP_ID, DEFAULT_ADDRESS};

mod calibration;
use crate::calibration::CalibrationData;

mod configuration;
pub use crate::configuration::{
    Configuration, Filter, Oversampling, SensorMode, StandbyTime, Status,
};

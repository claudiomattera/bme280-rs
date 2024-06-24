// Copyright Claudio Mattera 2022-2024.
//
// Distributed under the MIT License or the Apache 2.0 License at your option.
// See the accompanying files License-MIT.txt and License-Apache-2.0.txt, or
// online at
// https://opensource.org/licenses/MIT
// https://opensource.org/licenses/Apache-2.0

#![cfg_attr(not(doctest), doc = include_str!("../README.md"))]
#![cfg_attr(not(test), no_std)]

#[cfg(feature = "async")]
mod r#async;
#[cfg(feature = "async")]
pub use self::r#async::Bme280 as AsyncBme280;

#[cfg(feature = "blocking")]
mod blocking;
#[cfg(feature = "blocking")]
pub use self::blocking::Bme280;

#[cfg(any(feature = "async", feature = "blocking"))]
mod calibration;
#[cfg(any(feature = "async", feature = "blocking"))]
use self::calibration::CalibrationData;

#[cfg(any(feature = "async", feature = "blocking"))]
mod configuration;
#[cfg(any(feature = "async", feature = "blocking"))]
pub use self::configuration::Configuration;
#[cfg(any(feature = "async", feature = "blocking"))]
pub use self::configuration::Filter;
#[cfg(any(feature = "async", feature = "blocking"))]
pub use self::configuration::Oversampling;
#[cfg(any(feature = "async", feature = "blocking"))]
pub use self::configuration::SensorMode;
#[cfg(any(feature = "async", feature = "blocking"))]
pub use self::configuration::StandbyTime;
#[cfg(any(feature = "async", feature = "blocking"))]
pub use self::configuration::Status;

#[cfg(any(feature = "async", feature = "blocking"))]
mod constants;
#[cfg(any(feature = "async", feature = "blocking"))]
pub use self::constants::CHIP_ID;
#[cfg(any(feature = "async", feature = "blocking"))]
pub use self::constants::DEFAULT_ADDRESS;

#[cfg(any(feature = "async", feature = "blocking"))]
mod sample;
#[cfg(any(feature = "async", feature = "blocking"))]
pub use self::sample::Humidity;
#[cfg(any(feature = "async", feature = "blocking"))]
pub use self::sample::Pressure;
#[cfg(any(feature = "async", feature = "blocking"))]
pub use self::sample::Sample;
#[cfg(any(feature = "async", feature = "blocking"))]
pub use self::sample::Temperature;

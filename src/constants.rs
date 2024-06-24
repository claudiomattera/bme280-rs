// Copyright Claudio Mattera 2022-2024.
//
// Distributed under the MIT License or the Apache 2.0 License at your option.
// See the accompanying files License-MIT.txt and License-Apache-2.0.txt, or
// online at
// https://opensource.org/licenses/MIT
// https://opensource.org/licenses/Apache-2.0

//! Constants for BME280 sensor interface

/// Default I²C address
pub const DEFAULT_ADDRESS: u8 = 0x76;

/// Hardcoded chip ID
pub const CHIP_ID: u8 = 0x60;

/// I²C register for chip ID
pub(crate) const BME280_REGISTER_CHIPID: u8 = 0xd0;
/// I²C register for soft reset
pub(crate) const BME280_REGISTER_SOFTRESET: u8 = 0xe0;
/// I²C register for humidity configuration
pub(crate) const BME280_REGISTER_CONTROLHUMID: u8 = 0xf2;
/// I²C register for chip status
pub(crate) const BME280_REGISTER_STATUS: u8 = 0xf3;
/// I²C register for chip control
pub(crate) const BME280_REGISTER_CONTROL: u8 = 0xf4;
/// I²C register for chip configuration
pub(crate) const BME280_REGISTER_CONFIG: u8 = 0xf5;
/// I²C register for pressure data
pub(crate) const BME280_REGISTER_PRESSUREDATA: u8 = 0xf7;
/// I²C register for temperature data
pub(crate) const BME280_REGISTER_TEMPDATA: u8 = 0xfa;
/// I²C register for humidity data
pub(crate) const BME280_REGISTER_HUMIDDATA: u8 = 0xfd;

/// Command for soft reset
pub(crate) const BME280_COMMAND_SOFTRESET: u8 = 0xb6;

/// Bitmask for sleep mode
pub(crate) const MODE_SLEEP: u8 = 0b00;

/// Temperature output value when measurement was skipped
pub(crate) const SKIPPED_TEMPERATURE_OUTPUT: u32 = 0x80000;

/// Temperature output value when measurement was skipped
pub(crate) const SKIPPED_PRESSURE_OUTPUT: u32 = 0x80000;

/// Temperature output value when measurement was skipped
pub(crate) const SKIPPED_HUMIDITY_OUTPUT: u16 = 0x8000;

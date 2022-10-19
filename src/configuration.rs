// Copyright Claudio Mattera 2022.
//
// Distributed under the MIT License or the Apache 2.0 License at your option.
// See the accompanying files License-MIT.txt and License-Apache-2.0.txt, or
// online at
// https://opensource.org/licenses/MIT
// https://opensource.org/licenses/Apache-2.0

/// Chip configuration
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq)]
pub struct Configuration {
    standby_time: StandbyTime,
    filter: Filter,
    spi3w: bool,
    temperature_oversampling: Oversampling,
    pressure_oversampling: Oversampling,
    humidity_oversampling: Oversampling,
    sensor_mode: SensorMode,
}

impl From<&Configuration> for (Config, ControlMeasurement, ControlHumidity) {
    fn from(configuration: &Configuration) -> Self {
        let config = (
            configuration.standby_time,
            configuration.filter,
            configuration.spi3w,
        )
            .into();
        let control_measurement = (
            configuration.temperature_oversampling,
            configuration.pressure_oversampling,
            configuration.sensor_mode,
        )
            .into();
        let control_humidity = configuration.humidity_oversampling.into();
        (config, control_measurement, control_humidity)
    }
}

impl Configuration {
    #[doc(hidden)]
    pub fn to_lowlevel_configuration(&self) -> (Config, ControlMeasurement, ControlHumidity) {
        self.into()
    }

    /// Set the standby time
    pub fn with_standby_time(mut self, standby_time: StandbyTime) -> Self {
        self.standby_time = standby_time;
        self
    }

    /// Set the filter
    pub fn with_filter(mut self, filter: Filter) -> Self {
        self.filter = filter;
        self
    }

    #[doc(hidden)]
    pub fn with_spi3w(mut self, spi3w: bool) -> Self {
        self.spi3w = spi3w;
        self
    }

    /// Set the oversampling factor for temperature
    pub fn with_temperature_oversampling(mut self, temperature_oversampling: Oversampling) -> Self {
        self.temperature_oversampling = temperature_oversampling;
        self
    }

    /// Set the oversampling factor for pressure
    pub fn with_pressure_oversampling(mut self, pressure_oversampling: Oversampling) -> Self {
        self.pressure_oversampling = pressure_oversampling;
        self
    }

    /// Set the oversampling factor for humidity
    pub fn with_humidity_oversampling(mut self, humidity_oversampling: Oversampling) -> Self {
        self.humidity_oversampling = humidity_oversampling;
        self
    }

    /// Set the sensor mode
    pub fn with_sensor_mode(mut self, sensor_mode: SensorMode) -> Self {
        self.sensor_mode = sensor_mode;
        self
    }

    #[doc(hidden)]
    pub fn is_forced(&self) -> bool {
        self.sensor_mode == SensorMode::Forced
    }
}

/// Chip status
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq)]
pub struct Status {
    measuring: bool,
    calibrating: bool,
}

impl Status {
    /// Return `true` if the chip is measuring data
    pub fn is_measuring(&self) -> bool {
        self.measuring
    }

    /// Return `true` if the chip is computing calibration data
    pub fn is_calibrating(&self) -> bool {
        self.calibrating
    }
}

impl core::fmt::Display for Status {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if self.calibrating && self.measuring {
            write!(f, "calibrating and measuring")?;
        } else if self.calibrating {
            write!(f, "calibrating")?;
        } else if self.measuring {
            write!(f, "measuring")?;
        } else {
            write!(f, "ready")?;
        }
        Ok(())
    }
}

impl From<u8> for Status {
    fn from(arg: u8) -> Self {
        Self {
            measuring: (arg & 0b0000_0100) != 0,
            calibrating: (arg & 0b0000_0001) != 0,
        }
    }
}

/// Oversampling setting
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Oversampling {
    /// Skip the measurement altogether
    Skip,

    /// Take a single sample
    Oversample1,

    /// Take two samples
    Oversample2,

    /// Take four samples
    Oversample4,

    /// Take eight samples
    Oversample8,

    /// Take sixteen samples
    Oversample16,
}

impl Oversampling {
    #[doc(hidden)]
    pub fn to_value(&self) -> u8 {
        match self {
            Self::Skip => 0b000,
            Self::Oversample1 => 0b001,
            Self::Oversample2 => 0b010,
            Self::Oversample4 => 0b011,
            Self::Oversample8 => 0b100,
            Self::Oversample16 => 0b101,
        }
    }
}

impl Default for Oversampling {
    fn default() -> Self {
        Self::Skip
    }
}

/// Sensor working mode
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum SensorMode {
    /// Sleep mode
    ///
    /// The sensor is not active.
    Sleep,

    /// Forced mode
    ///
    /// The sensor takes a single sample and then enters sleep mode.
    Forced,

    /// Normal mode
    ///
    /// The sensor takes samples at regular times, and returns the latest one
    /// when queried.
    /// The time between measurements can be specified as [`StandbyTime`]
    Normal,
}

impl SensorMode {
    #[doc(hidden)]
    pub fn to_value(&self) -> u8 {
        match self {
            Self::Sleep => 0b00,
            Self::Forced => 0b01,
            Self::Normal => 0b11,
        }
    }
}

impl Default for SensorMode {
    fn default() -> Self {
        Self::Sleep
    }
}

/// Standby time between readings in normal mode
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum StandbyTime {
    /// 0.5 ms
    Millis0_5,

    /// 10 ms
    Millis10,

    /// 20 ms
    Millis20,

    /// 62.5 ms
    Millis62_5,

    /// 125 ms
    Millis125,

    /// 250 ms
    Millis250,

    /// 500 ms
    Millis500,

    /// 1000 ms
    Millis1000,
}

impl StandbyTime {
    #[doc(hidden)]
    pub fn to_value(&self) -> u8 {
        match self {
            Self::Millis0_5 => 0b000,
            Self::Millis10 => 0b110,
            Self::Millis20 => 0b111,
            Self::Millis62_5 => 0b001,
            Self::Millis125 => 0b010,
            Self::Millis250 => 0b011,
            Self::Millis500 => 0b100,
            Self::Millis1000 => 0b101,
        }
    }
}

impl Default for StandbyTime {
    fn default() -> Self {
        Self::Millis0_5
    }
}

/// Filter coefficient
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Filter {
    /// No filtering
    Off,

    /// Filter ×2
    Filter2,

    /// Filter ×4
    Filter4,

    /// Filter ×8
    Filter8,

    /// Filter ×16
    Filter16,
}

impl Filter {
    #[doc(hidden)]
    pub fn to_value(&self) -> u8 {
        match self {
            Self::Off => 0b000,
            Self::Filter2 => 0b001,
            Self::Filter4 => 0b010,
            Self::Filter8 => 0b011,
            Self::Filter16 => 0b100,
        }
    }
}

impl Default for Filter {
    fn default() -> Self {
        Self::Off
    }
}

#[derive(Copy, Clone, Debug, Default, Eq, PartialEq)]
pub struct Config(u8);

impl From<(StandbyTime, Filter, bool)> for Config {
    fn from((standby_time, filter, spi3w): (StandbyTime, Filter, bool)) -> Self {
        let standby_time = standby_time.to_value() & 0b111;
        let filter = filter.to_value() & 0b111;
        let spi3w = spi3w as u8 & 0b1;
        Self(standby_time << 5 | filter << 2 | spi3w)
    }
}

impl From<Config> for u8 {
    fn from(config: Config) -> Self {
        config.0
    }
}

#[derive(Copy, Clone, Debug, Default, Eq, PartialEq)]
pub struct ControlHumidity(u8);

impl From<Oversampling> for ControlHumidity {
    fn from(humidity_oversampling: Oversampling) -> Self {
        Self(humidity_oversampling.to_value() & 0b111)
    }
}

impl From<ControlHumidity> for u8 {
    fn from(ctrl_hum: ControlHumidity) -> Self {
        ctrl_hum.0
    }
}

#[derive(Copy, Clone, Debug, Default, Eq, PartialEq)]
pub struct ControlMeasurement(u8);

impl From<(Oversampling, Oversampling, SensorMode)> for ControlMeasurement {
    fn from(
        (oversampling_temperature, oversampling_pressure, sensor_mode): (
            Oversampling,
            Oversampling,
            SensorMode,
        ),
    ) -> Self {
        let oversampling_temperature = oversampling_temperature.to_value() & 0b111;
        let oversampling_pressure = oversampling_pressure.to_value() & 0b111;
        let sensor_mode = sensor_mode.to_value() & 0b11;
        Self(oversampling_temperature << 5 | oversampling_pressure << 2 | sensor_mode)
    }
}

impl From<ControlMeasurement> for u8 {
    fn from(ctrl_meas: ControlMeasurement) -> Self {
        ctrl_meas.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use assert2::check;

    #[test]
    fn test_status() {
        let raw_status = 0b0000_0000;
        let status: Status = raw_status.into();

        let expected = Status {
            measuring: false,
            calibrating: false,
        };

        check!(status == expected);
    }

    #[test]
    fn test_status_calibrating() {
        let raw_status = 0b0000_0001;
        let status: Status = raw_status.into();

        let expected = Status {
            measuring: false,
            calibrating: true,
        };

        check!(status == expected);
    }

    #[test]
    fn test_standby() {
        let standby = StandbyTime::Millis125;

        let expected = 0b010;
        let actual = standby.to_value();

        check!(actual == expected, "0b{:03b} == 0b{:03b}", actual, expected);
    }

    #[test]
    fn test_config() {
        let configuration = Configuration::default()
            .with_standby_time(StandbyTime::Millis125)
            .with_filter(Filter::Filter2)
            .with_spi3w(true);
        let (config, _ctrl_meas, _ctrl_hum) = configuration.to_lowlevel_configuration();
        let actual: u8 = config.into();

        let expected = 0b0100_0101;

        check!(actual == expected, "0b{:08b} == 0b{:08b}", actual, expected);
    }

    #[test]
    fn test_control_measurement() {
        let configuration = Configuration::default()
            .with_temperature_oversampling(Oversampling::Oversample8)
            .with_pressure_oversampling(Oversampling::Oversample4)
            .with_sensor_mode(SensorMode::Normal);
        let (_config, ctrl_meas, _ctrl_hum) = configuration.to_lowlevel_configuration();
        let actual: u8 = ctrl_meas.into();

        let expected = 0b1000_1111;

        check!(actual == expected, "0b{:08b} == 0b{:08b}", actual, expected);
    }

    #[test]
    fn test_control_humidity() {
        let configuration =
            Configuration::default().with_humidity_oversampling(Oversampling::Oversample8);
        let (_config, _ctrl_meas, ctrl_hum) = configuration.to_lowlevel_configuration();
        let actual: u8 = ctrl_hum.into();

        let expected = 0b100;

        check!(actual == expected, "0b{:03b} == 0b{:03b}", actual, expected);
    }
}

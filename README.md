Rust BME280 Crate
====

A Rust crate to query temperature, pressure and humidity from sensor [BME280]

<https://gitlab.com/claudiomattera/bme280-rs/>

[BME280]: https://www.bosch-sensortec.com/products/environmental-sensors/humidity-sensors-bme280/

This crate support both [`embedded-hal`][embedded-hal] and [`embedded-hal-async`][embedded-hal-async].

[embedded-hal]: https://crates.io/crates/embedded-hal
[embedded-hal-async]: https://crates.io/crates/embedded-hal-async


Usage
----

Add the dependency to `Cargo.toml`.

~~~~toml
[dependencies.bme280-rs]
version = "0.2.0"
~~~~

A `Bme280` structure can be created from an I²C interface and a delay function.
The initial sampling configuration disables all measurements, so it is necessary to reconfigure the chip with the desired settings before read samples.

~~~~rust
use bme280_rs::{Bme280, Configuration, Oversampling, SensorMode};

let i2c = ...
let delay = ...

let mut bme280 = Bme280::new(i2c, delay);

bme280.init()?;

bme280.set_sampling_configuration(
    Configuration::default()
        .with_temperature_oversampling(Oversampling::Oversample1)
        .with_pressure_oversampling(Oversampling::Oversample1)
        .with_humidity_oversampling(Oversampling::Oversample1)
        .with_sensor_mode(SensorMode::Normal)
)?;

if let Some(temperature) = bme280.read_temperature()? {
    println!("Temperature: {} C", temperature);
} else {
    println!("Temperature reading was disabled");
}
~~~~

An `AsyncBme280` structure can be used with asynchronous HALs.
Its API is completely identical to `Bme280`, just with `.async` at the end of function calls.

~~~~rust
use bme280_rs::{AsyncBme280, Configuration, Oversampling, SensorMode};

let i2c = ...
let delay = ...

let mut bme280 = AsyncBme280::new(i2c, delay);

bme280.init()?;

bme280.set_sampling_configuration(
    Configuration::default()
        .with_temperature_oversampling(Oversampling::Oversample1)
        .with_pressure_oversampling(Oversampling::Oversample1)
        .with_humidity_oversampling(Oversampling::Oversample1)
        .with_sensor_mode(SensorMode::Normal)
).await?;

if let Some(temperature) = bme280.read_temperature().await? {
    println!("Temperature: {} C", temperature);
} else {
    println!("Temperature reading was disabled");
}
~~~~


Unit of Measurements
----

By default, this crate uses `f32` values for all the measurements temperature, pressure and humidity.
When instead enabling the Cargo feature `uom`, it uses quantities from crate [uom].
Temperature measurements have type `uom::si::f32::ThermodynamicTemperature`, pressure measurements have type `uom::si::f32::Pressure`, and humidity measurements have type `uom::si::f32::Ratio`.

[uom]: crates.io/crates/uom


License
----

Copyright Claudio Mattera 2022-2024


You are free to copy, modify, and distribute this application with attribution under the terms of either

 * Apache License, Version 2.0
   ([LICENSE-Apache-2.0](./LICENSE-APACHE-2.0.txt) or <https://opensource.org/licenses/Apache-2.0>)
 * MIT license
   ([LICENSE-MIT](./LICENSE-MIT.txt) or <https://opensource.org/licenses/MIT>)

at your option.

This project is entirely original work, and it is not affiliated with nor endorsed in any way by Bosch Sensortec.

mod trait_def;

#[cfg(feature = "scd4x")]
mod scd4x;

#[cfg(feature = "bme280")]
mod bme280;

#[cfg(feature = "tsl2591")]
mod tsl2591;

pub(crate) use trait_def::{Measurement, Sensor};
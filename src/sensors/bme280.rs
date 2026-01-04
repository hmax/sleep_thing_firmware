use embedded_hal_bus::i2c::RcDevice;
use esp_idf_svc::hal::delay::Delay;
use esp_idf_svc::hal::i2c::I2cDriver;
use log::error;
use bme280_rs::{Bme280, Configuration as Bme280Configuration};

use super::trait_def::{Measurement, Sensor};

impl<'a> Sensor<'a> for Bme280<RcDevice<I2cDriver<'a>>, Delay> {
    fn measure(&mut self) -> Vec<Measurement> {
        let mut measurements: Vec<Measurement> = Vec::new();
        self.take_forced_measurement().unwrap();
        match self.read_sample() {
            Ok(sample) => {
                match sample.temperature {
                    Some(value) => {
                        measurements.push(Measurement {
                            name: "temperature".to_string(),
                            value: value,
                        });
                    }
                    None => {
                        error!("Temperature measurement is disabled");
                    }
                };
                match sample.pressure {
                    Some(value) => {
                        measurements.push(Measurement {
                            name: "pressure".to_string(),
                            value: value * 0.0075,
                        });
                    }
                    None => {
                        error!("Pressure measurement is disabled");
                    }
                };
                match sample.humidity {
                    Some(value) => {
                        measurements.push(Measurement {
                            name: "humidity".to_string(),
                            value: value,
                        });
                    }
                    None => {
                        error!("Humidity measurement is disabled");
                    }
                }
            }
            Err(err) => {
                error!("Error reading sample: {:?}", err);
            }
        }
        measurements
    }

    fn get_sensor(i2c_device: RcDevice<I2cDriver<'a>>) -> Self {
        println!("Waking up a sensor");
        let delay = Delay::new_default();
        let mut sensor: Bme280<RcDevice<I2cDriver<'a>>, Delay> = Bme280::new(i2c_device, delay);
        sensor.init().unwrap();
        sensor
            .set_sampling_configuration(
                Bme280Configuration::default()
                    .with_sensor_mode(bme280_rs::SensorMode::Forced)
                    .with_humidity_oversampling(bme280_rs::Oversampling::Oversample4)
                    .with_temperature_oversampling(bme280_rs::Oversampling::Oversample4)
                    .with_pressure_oversampling(bme280_rs::Oversampling::Oversample4)
            )
            .unwrap();

        delay.delay_ms(100);

        sensor
    }
}

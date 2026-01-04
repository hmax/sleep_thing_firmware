use std::time::Duration;
use embedded_hal_bus::i2c::RcDevice;
use esp_idf_svc::hal::delay::Delay;
use esp_idf_svc::hal::i2c::I2cDriver;
use log::{error, info};
use scd4x::Scd4x;

use super::trait_def::{Measurement, Sensor};

impl<'a> Sensor<'a> for Scd4x<RcDevice<I2cDriver<'a>>, Delay> {
    fn measure(&mut self) -> Vec<Measurement> {
        self.wake_up();
        self.wake_up(); // For some reason if you just do the one wakeup it doesn't work, need to check it with an LA or scope
        std::thread::sleep(Duration::from_millis(200)); // according to spec should not take more than 20msec, since wake_up doesn't get an ACK, so we are waiting 10x

        let _ = self.measure_single_shot(); // Discarding the first reading after waking up, according to the spec
        let result = self.measure_single_shot();
        let measurements: Vec<Measurement> = match result {
            Ok(_) => match self.measurement() {
                Ok(measurement) => {
                    info!(
                        "CO2: {:?}, Humidity: {} RH, Temperature: {} C",
                        measurement.co2, measurement.humidity, measurement.temperature
                    );
                    vec![
                        Measurement {
                            name: "co2".to_string(),
                            value: measurement.co2 as f32,
                        },
                        Measurement {
                            name: "humidity".to_string(),
                            value: measurement.humidity,
                        },
                        Measurement {
                            name: "temperature".to_string(),
                            value: measurement.temperature,
                        },
                    ]
                }
                Err(error) => {
                    error!("Error trying to measure co2: {:?}", error);
                    vec![]
                }
            },
            Err(_) => vec![],
        };
        let _ = self.power_down();
        measurements
    }

    fn get_sensor(i2c_device: RcDevice<I2cDriver<'a>>) -> Self {
        println!("Setting up a sensor");
        let mut sensor = Scd4x::new(i2c_device, Delay::new_default());
        println!("Stopping periodic measurement in a sensor");
        _ = sensor.stop_periodic_measurement();
        println!("Re-initializing a sensor");
        sensor.reinit().unwrap();

        let serial = sensor.serial_number().unwrap();
        println!("serial: {:#04x}", serial);
        sensor
    }
}

use std::time::Duration;
use embedded_hal_bus::i2c::RcDevice;
use esp_idf_svc::hal::delay::Delay;
use esp_idf_svc::hal::i2c::I2cDriver;
use log::{error, info};

#[cfg(feature = "scd4x")]
use scd4x::Scd4x;



#[derive(Debug)]
pub struct Measurement {
    pub name: String,
    pub value: f32,
}

pub trait Sensor<'a> {
    fn measure(&mut self) -> Vec<Measurement>;
    fn get_sensor(i2c_device: RcDevice<I2cDriver<'a>>) -> Self
    where
        Self: Sized;
}

#[cfg(feature = "scd4x")]
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

#[cfg(feature = "tsl2591")]
impl<'a> Sensor<'a> for tsl2591_eh_driver::Driver<RcDevice<I2cDriver<'a>>> {
    fn measure(&mut self) -> Vec<Measurement> {
        let mut current_gain = tsl2591_eh_driver::Gain::MED;
        let current_scan = tsl2591_eh_driver::IntegrationTimes::_100MS;

        loop {
            self.set_gain(current_gain).unwrap();
            self.set_timing(current_scan).unwrap();

            self.enable().unwrap();
            let mut loop_count = 0;
            while loop_count < 10 {
                let lux_sensor_status = self.get_status().unwrap();
                if lux_sensor_status.avalid() {
                    println!("Lux sensor status: {:?}", lux_sensor_status);
                    break;
                } else {
                    loop_count = loop_count + 1;
                    std::thread::sleep(Duration::from_millis(100));
                }
            }

            let (ch0, ch1) = self.get_channel_data().unwrap();
            self.disable().unwrap();

            match self.calculate_lux(ch0, ch1) {
                Ok(lux) => {
                    if lux.is_nan() {
                        // Basically we got an underflow
                        match increment_gain(current_gain) {
                            Ok(gain) => {
                                current_gain = gain;
                            }
                            // We are already at max gain, we can consider this to be pitch-black
                            Err(_) => {
                                return vec![Measurement {
                                    name: "lux".to_string(),
                                    value: 0.0,
                                }]
                            }
                        }
                    } else if lux.is_infinite() {
                        return vec![];
                    } else {
                        info!("Lux: {} lx", lux);
                        return vec![Measurement {
                            name: "lux".to_string(),
                            value: lux,
                        }];
                    }
                }
                // We have an overflow
                Err(_) => {
                    match decrement_gain(current_gain) {
                        Ok(gain) => {
                            current_gain = gain;
                        }
                        // If we are at the lowest gain already and are still getting an overflow we can return the brightest sunlight levels
                        Err(_) => return vec![],
                    }
                }
            }
        }
    }

    fn get_sensor(i2c_device: RcDevice<I2cDriver<'a>>) -> Self {
        let mut lux_sensor = tsl2591_eh_driver::Driver::new(i2c_device).unwrap();
        lux_sensor.enable().unwrap();
        std::thread::sleep(Duration::from_millis(1000));

        println!("Lux sensor status: {:?}", lux_sensor.get_status().unwrap());
        lux_sensor.disable().unwrap();
        lux_sensor
    }
}

#[cfg(feature = "bme280")]
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

#[cfg(feature = "tsl2591")]
fn increment_gain(gain: tsl2591_eh_driver::Gain) -> Result<tsl2591_eh_driver::Gain, &'static str> {
    match gain {
        tsl2591_eh_driver::Gain::LOW => Ok(tsl2591_eh_driver::Gain::MED),
        tsl2591_eh_driver::Gain::MED => Ok(tsl2591_eh_driver::Gain::HIGH),
        tsl2591_eh_driver::Gain::HIGH => Ok(tsl2591_eh_driver::Gain::MAX),
        tsl2591_eh_driver::Gain::MAX => Err("Gain maxed out"),
    }
}

#[cfg(feature = "tsl2591")]
fn decrement_gain(gain: tsl2591_eh_driver::Gain) -> Result<tsl2591_eh_driver::Gain, &'static str> {
    match gain {
        tsl2591_eh_driver::Gain::LOW => Err("Gain already minimal"),
        tsl2591_eh_driver::Gain::MED => Ok(tsl2591_eh_driver::Gain::LOW),
        tsl2591_eh_driver::Gain::HIGH => Ok(tsl2591_eh_driver::Gain::MED),
        tsl2591_eh_driver::Gain::MAX => Ok(tsl2591_eh_driver::Gain::HIGH),
    }
}

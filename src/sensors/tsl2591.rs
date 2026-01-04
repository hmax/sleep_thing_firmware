use std::time::Duration;
use embedded_hal_bus::i2c::RcDevice;
use esp_idf_svc::hal::i2c::I2cDriver;
use log::{error, info, warn};
use tsl2591_eh_driver;

use super::trait_def::{Measurement, Sensor};

impl<'a> Sensor<'a> for tsl2591_eh_driver::Driver<RcDevice<I2cDriver<'a>>> {
    fn measure(&mut self) -> Vec<Measurement> {
        let mut current_gain = tsl2591_eh_driver::Gain::MED;
        let current_scan = tsl2591_eh_driver::IntegrationTimes::_100MS;
        let max_iterations = 10; // Prevent infinite loop
        let mut iteration = 0;

        loop {
            if iteration >= max_iterations {
                error!("TSL2591: Max iterations reached in gain adjustment loop");
                return vec![];
            }
            iteration += 1;

            if let Err(e) = self.set_gain(current_gain) {
                error!("TSL2591: Failed to set gain: {:?}", e);
                return vec![];
            }
            if let Err(e) = self.set_timing(current_scan) {
                error!("TSL2591: Failed to set timing: {:?}", e);
                return vec![];
            }

            if let Err(e) = self.enable() {
                error!("TSL2591: Failed to enable sensor: {:?}", e);
                return vec![];
            }

            let mut loop_count = 0;
            while loop_count < 10 {
                let lux_sensor_status = match self.get_status() {
                    Ok(status) => status,
                    Err(e) => {
                        error!("TSL2591: Failed to get status: {:?}", e);
                        return vec![];
                    }
                };
                if lux_sensor_status.avalid() {
                    println!("Lux sensor status: {:?}", lux_sensor_status);
                    break;
                } else {
                    loop_count += 1;
                    std::thread::sleep(Duration::from_millis(100));
                }
            }

            let (ch0, ch1) = match self.get_channel_data() {
                Ok(data) => data,
                Err(e) => {
                    error!("TSL2591: Failed to get channel data: {:?}", e);
                    return vec![];
                }
            };

            if let Err(e) = self.disable() {
                warn!("TSL2591: Failed to disable sensor: {:?}", e);
            }

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
        println!("Initializing TSL2591 light sensor");
        let mut lux_sensor = tsl2591_eh_driver::Driver::new(i2c_device)
            .expect("Failed to create TSL2591 sensor - check I2C connection");
        lux_sensor.enable()
            .expect("Failed to enable TSL2591 sensor");
        std::thread::sleep(Duration::from_millis(1000));

        let status = lux_sensor.get_status()
            .expect("Failed to read TSL2591 status");
        println!("TSL2591 status: {:?}", status);
        lux_sensor.disable()
            .expect("Failed to disable TSL2591 sensor");
        lux_sensor
    }
}

fn increment_gain(gain: tsl2591_eh_driver::Gain) -> Result<tsl2591_eh_driver::Gain, &'static str> {
    match gain {
        tsl2591_eh_driver::Gain::LOW => Ok(tsl2591_eh_driver::Gain::MED),
        tsl2591_eh_driver::Gain::MED => Ok(tsl2591_eh_driver::Gain::HIGH),
        tsl2591_eh_driver::Gain::HIGH => Ok(tsl2591_eh_driver::Gain::MAX),
        tsl2591_eh_driver::Gain::MAX => Err("Gain maxed out"),
    }
}

fn decrement_gain(gain: tsl2591_eh_driver::Gain) -> Result<tsl2591_eh_driver::Gain, &'static str> {
    match gain {
        tsl2591_eh_driver::Gain::LOW => Err("Gain already minimal"),
        tsl2591_eh_driver::Gain::MED => Ok(tsl2591_eh_driver::Gain::LOW),
        tsl2591_eh_driver::Gain::HIGH => Ok(tsl2591_eh_driver::Gain::MED),
        tsl2591_eh_driver::Gain::MAX => Ok(tsl2591_eh_driver::Gain::HIGH),
    }
}

use std::time::Duration;
use embedded_hal_bus::i2c::RcDevice;
use esp_idf_svc::hal::i2c::I2cDriver;
use log::info;
use tsl2591_eh_driver;

use super::trait_def::{Measurement, Sensor};

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

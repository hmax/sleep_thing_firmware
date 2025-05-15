use std::io;
use std::io::Write;
use std::net::TcpStream;

use core::time::Duration;
use embedded_hal_bus::i2c::RcDevice;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::hal::delay::Delay;
use esp_idf_svc::hal::i2c::{I2cConfig, I2cDriver};
use esp_idf_svc::hal::prelude::Peripherals;
use esp_idf_svc::hal::units::FromValueType;
use esp_idf_svc::log::EspLogger;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::sntp;
use esp_idf_svc::sntp::SyncStatus;
use esp_idf_svc::sys::EspError;
use esp_idf_svc::wifi::PmfConfiguration::NotCapable;
use esp_idf_svc::wifi::ScanMethod::FastScan;
use esp_idf_svc::wifi::{AuthMethod, BlockingWifi, ClientConfiguration, Configuration, EspWifi};
use log::{debug, error, info, trace, LevelFilter};
use rand::prelude::*;
use ringbuffer::{AllocRingBuffer, RingBuffer};
use std::cell::RefCell;
use std::env;
use std::rc::Rc;
use std::time::SystemTime;

#[cfg(feature = "tsl2591")]
use tsl2591_eh_driver;

#[cfg(feature = "bme280")]
use bme280_rs::{Bme280, Configuration as Bme280Configuration};
#[cfg(feature = "scd4x")]
use scd4x::{types::SensorData, Scd4x};

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("WIFI_PASSWORD");

const HOST: &str = "192.168.24.1";
const PORT: &str = "2003";

const SEND_TIMEOUT_SEC: i32 = 30;

const DATA_PREFIX: &str = "sensors.garage.basement.";

#[derive(Debug)]
struct Measurement {
    name: String,
    value: f32,
}
trait Sensor<'a> {
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
impl<'a> Sensor<'a>  for tsl2591_eh_driver::Driver<RcDevice<I2cDriver<'a>>> {
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
impl<'a> Sensor<'a>  for Bme280<RcDevice<I2cDriver<'a>>, Delay> {
    fn measure(&mut self) -> Vec<Measurement> {
        let mut measurements: Vec<Measurement> = Vec::new();
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
                    .with_humidity_oversampling(bme280_rs::Oversampling::Oversample8)
                    .with_temperature_oversampling(bme280_rs::Oversampling::Oversample8)
                    .with_pressure_oversampling(bme280_rs::Oversampling::Oversample8)
            )
            .unwrap();

        delay.delay_ms(100);

        sensor
    }
}

fn preamble() {
    esp_idf_svc::sys::link_patches();
    EspLogger::initialize_default();
    esp_idf_svc::log::set_target_level("wifi", LevelFilter::Error).unwrap();
}

// fn get_sensors<'a>(i2c_ref_cell: &'a RefCell<I2cDriver<'a>>) -> Vec<Box<dyn Sensor<'a, 'a> + 'a>> {
//     let mut sensors: Vec<Box<dyn Sensor<'a, 'a> + 'a>> = Vec::new();
//     sensors.push(Box::new(Bme280::get_sensor(RefCellDevice::new(
//         i2c_ref_cell,
//     ))));
//     sensors
// }

fn main() -> anyhow::Result<()> {
    preamble();

    let mut peripherals = Peripherals::take()?;

    let config = I2cConfig::new().baudrate(100u32.kHz().into());
    let i2c = I2cDriver::new(
        peripherals.i2c0,
        peripherals.pins.gpio19,
        peripherals.pins.gpio20,
        &config,
    )?;

    let sys_loop = EspSystemEventLoop::take()?;
    let nvs = EspDefaultNvsPartition::take()?;
    let mut wifi = BlockingWifi::wrap(
        EspWifi::new(&mut peripherals.modem, sys_loop.clone(), Some(nvs))?,
        sys_loop.clone(),
    )?;

    connect_wifi(&mut wifi)?;
    let _sntp = sntp::EspSntp::new_default()?;
    info!("SNTP initialized");

    while _sntp.get_sync_status() != SyncStatus::Completed {
        std::thread::sleep(Duration::from_millis(200));
    }
    info!("SNTP synced");

    trace!("Calling run");
    let i2c_ref_cell = Rc::new(RefCell::new(i2c));
    let mut sensors: Vec<Box<dyn Sensor>> = Vec::new();


    #[cfg(feature = "bme280")]
    sensors.push(Box::new(Bme280::get_sensor(RcDevice::new(i2c_ref_cell.clone()))));

    #[cfg(feature = "scd4x")]
    sensors.push(Box::new(Scd4x::get_sensor(RcDevice::new(i2c_ref_cell.clone()))));

    #[cfg(feature = "tsl2591")]
    sensors.push(Box::new(tsl2591_eh_driver::Driver::get_sensor(RcDevice::new(i2c_ref_cell.clone()))));
    run(wifi, &mut sensors)?;
    Ok(())
}

fn send_data(now: u64, measurements: &Vec<Measurement>) -> Result<(), io::Error> {
    let mut stream = TcpStream::connect(std::format!("{}:{}", HOST, PORT))?;

    for measurement in measurements {
        stream.write_all(
            format!(
                "{prefix}{name} {value} {ts}\n",
                prefix = DATA_PREFIX,
                name = measurement.name,
                value = measurement.value,
                ts = now
            )
            .as_bytes(),
        )?;
    }

    Ok(())
}

fn connect_wifi(wifi: &mut BlockingWifi<EspWifi>) -> anyhow::Result<()> {
    disconnect_wifi(wifi)?;

    let wifi_configuration: Configuration = Configuration::Client(ClientConfiguration {
        ssid: SSID.try_into().unwrap(),
        bssid: None,
        auth_method: AuthMethod::WPA2Personal,
        password: PASSWORD.try_into().unwrap(),
        channel: None,
        scan_method: FastScan,
        pmf_cfg: NotCapable,
    });

    wifi.set_configuration(&wifi_configuration)?;
    wifi.start()?;
    wifi.connect()?;
    wifi.wait_netif_up()?;
    Ok(())
}

fn disconnect_wifi(wifi: &mut BlockingWifi<EspWifi>) -> anyhow::Result<()> {
    if wifi.is_started()? {
        if wifi.is_connected()? {
            wifi.disconnect()?;
        }
        wifi.stop()?;
    }
    Ok(())
}

fn run<'a>(
    mut wifi: BlockingWifi<EspWifi>,
    sensors: &mut Vec<Box<dyn Sensor<'a> + 'a>>,
) -> Result<(), EspError> {
    debug!("Starting main loop");
    let mut measurements: AllocRingBuffer<(u64, Vec<Measurement>)> =
        AllocRingBuffer::new((24 * 60 * 60 / SEND_TIMEOUT_SEC) as usize); // Buffer large enough to hold a day of measurements
    loop {
        let mut new_measurements: Vec<Measurement> = Vec::new();

        // let co2 = measure_co2(&mut co2_sensor);
        // let lux = measure_lux(&mut lux_sensor);
        for sensor in &mut *sensors {
            let measurement = sensor.measure();
            println!("Measurement {:?}", measurement);
            new_measurements.extend(measurement);
        }

        if new_measurements.len() > 0 {
            let now = SystemTime::now()
                .duration_since(SystemTime::UNIX_EPOCH)
                .unwrap()
                .as_secs();

            measurements.push((now, new_measurements));
        }
        println!("Measurements available for sending: {}", measurements.len());
        match connect_wifi(&mut wifi) {
            Ok(_) => {
                while let Some((now, values)) = measurements.dequeue() {
                    match send_data(now, &values) {
                        Ok(_) => {}
                        Err(err) => {
                            error!("Error while sending data: {:?}", err);
                            measurements.push((now, values));
                            break;
                        }
                    }
                }

                std::thread::sleep(Duration::from_millis(5000));

                match disconnect_wifi(&mut wifi) {
                    Ok(_) => {}
                    Err(error) => {
                        error!("Error while trying to disconnect from wifi: {:?}", error);
                    }
                }
            }
            Err(error) => {
                error!("Error while trying to connect to wifi: {:?}", error);
            }
        };

        let spread = (SEND_TIMEOUT_SEC as f32 * 0.1) as i32;
        let jitter = rand::rng().random_range((-spread)..=spread);

        std::thread::sleep(Duration::from_secs((SEND_TIMEOUT_SEC + jitter) as u64));
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
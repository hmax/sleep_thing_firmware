mod sensors;

use std::io;
use std::io::Write;
use std::net::TcpStream;

use embedded_hal_bus::i2c::RcDevice;
use esp_idf_svc::eventloop::EspSystemEventLoop;
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
use std::time::{Duration, SystemTime};

#[cfg(feature = "tsl2591")]
use tsl2591_eh_driver;

#[cfg(feature = "bme280")]
use bme280_rs::{Bme280, Configuration as Bme280Configuration};
#[cfg(feature = "scd4x")]
use scd4x::Scd4x;
use esp_idf_svc::hal::i2c::{I2cConfig, I2cDriver};
use crate::sensors::Sensor;

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("WIFI_PASSWORD");

const HOST: &str = "192.168.24.1";
const PORT: &str = "2003";

const SEND_TIMEOUT_SEC: i32 = 300;

const DATA_PREFIX: &str = "sensors.hbase.bedroom.";


fn preamble() {
    esp_idf_svc::sys::link_patches();
    EspLogger::initialize_default();
    esp_idf_svc::log::set_target_level("wifi", LevelFilter::Error).unwrap();
}

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
    let mut sensors: Vec<Box<dyn sensors::Sensor>> = Vec::new();


    #[cfg(feature = "bme280")]
    sensors.push(Box::new(Bme280::get_sensor(RcDevice::new(i2c_ref_cell.clone()))));

    #[cfg(feature = "scd4x")]
    sensors.push(Box::new(Scd4x::get_sensor(RcDevice::new(i2c_ref_cell.clone()))));

    #[cfg(feature = "tsl2591")]
    sensors.push(Box::new(tsl2591_eh_driver::Driver::get_sensor(RcDevice::new(i2c_ref_cell.clone()))));
    run(wifi, &mut sensors)?;
    Ok(())
}

fn send_data(now: u64, measurements: &Vec<sensors::Measurement>) -> Result<(), io::Error> {
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
    sensors: &mut Vec<Box<dyn sensors::Sensor<'a> + 'a>>,
) -> Result<(), EspError> {
    debug!("Starting main loop");
    let mut measurements: AllocRingBuffer<(u64, Vec<sensors::Measurement>)> =
        AllocRingBuffer::new((24 * 60 * 60 / SEND_TIMEOUT_SEC) as usize); // Buffer large enough to hold a day of measurements
    loop {
        let mut new_measurements: Vec<sensors::Measurement> = Vec::new();

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


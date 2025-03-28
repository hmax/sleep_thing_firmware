use std::io;
use std::io::Write;
use std::net::TcpStream;

use core::time::Duration;
use embedded_hal_bus::i2c::RefCellDevice;
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
use log::{debug, error, info, trace};
use rand::prelude::*;
use ringbuffer::{AllocRingBuffer, RingBuffer};
use scd4x::types::SensorData;
use scd4x::Scd4x;
use std::cell::RefCell;
use std::env;
use std::time::SystemTime;
use tsl2591_eh_driver;

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("WIFI_PASSWORD");

const HOST: &str = "192.168.24.1";
const PORT: &str = "2003";

const SEND_TIMEOUT_SEC: i32 = 300;

fn preamble() {
    esp_idf_svc::sys::link_patches();
    EspLogger::initialize_default();
}

fn get_co2_sensor<'a, 'b>(
    i2c: RefCellDevice<'b, I2cDriver<'a>>,
) -> Scd4x<RefCellDevice<'b, I2cDriver<'a>>, Delay> {
    println!("Setting up a sensor");
    let mut sensor = Scd4x::new(i2c, Delay::new_default());
    println!("Stopping periodic measurement in a sensor");
    //_ = sensor.stop_periodic_measurement();
    println!("Re-initializing a sensor");
    sensor.reinit().unwrap();

    let serial = sensor.serial_number().unwrap();
    println!("serial: {:#04x}", serial);
    sensor
}

fn main() -> anyhow::Result<()> {
    preamble();

    let mut peripherals = Peripherals::take()?;

    let config = I2cConfig::new().baudrate(100u32.kHz().into());
    let i2c = I2cDriver::new(
        peripherals.i2c0,
        peripherals.pins.gpio9,
        peripherals.pins.gpio7,
        &config,
    )?;
    let i2c_ref_cell = RefCell::new(i2c);
    let co2_sensor = get_co2_sensor(RefCellDevice::new(&i2c_ref_cell));
    let lux_sensor = get_lux_sensor(RefCellDevice::new(&i2c_ref_cell));
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
        std::thread::sleep(std::time::Duration::from_millis(200));
    }
    info!("SNTP synced");

    trace!("Calling run");
    run(wifi, co2_sensor, lux_sensor)?;
    Ok(())
}

fn get_lux_sensor<'a, 'b>(
    i2c: RefCellDevice<'b, I2cDriver<'a>>,
) -> tsl2591_eh_driver::Driver<RefCellDevice<'b, I2cDriver<'a>>> {
    let mut lux_sensor = tsl2591_eh_driver::Driver::new(i2c).unwrap();
    lux_sensor.enable().unwrap();
    std::thread::sleep(Duration::from_millis(200)); // according to spec, since wake_up doesn't get an ACK

    println!("Lux sensor status: {:?}", lux_sensor.get_status().unwrap());
    lux_sensor.disable().unwrap();
    lux_sensor
}

fn send_data(now: u64, co2: &Option<SensorData>, lux: Option<f32>) -> Result<(), io::Error> {
    let mut stream = TcpStream::connect(std::format!("{}:{}", HOST, PORT))?;

    if lux.is_some() {
        stream.write_all(
            format!("sensors.hbase.cabinet.lux {} {}\n", lux.unwrap(), now).as_bytes(),
        )?;
        println!("Sent lux data for {}", now);
    }

    match co2 {
        Some(data) => {
            stream.write_all(
                format!("sensors.hbase.cabinet.temp {} {}\n", data.temperature, now).as_bytes(),
            )?;
            stream.write_all(
                format!("sensors.hbase.cabinet.humidity {} {}\n", data.humidity, now).as_bytes(),
            )?;
            stream.write_all(
                format!("sensors.hbase.cabinet.co2 {} {}\n", data.co2, now).as_bytes(),
            )?;
            println!("Sent co2 data for {}", now);
        }
        None => {}
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

fn run(
    mut wifi: BlockingWifi<EspWifi>,
    mut co2_sensor: Scd4x<RefCellDevice<I2cDriver>, Delay>,
    mut lux_sensor: tsl2591_eh_driver::Driver<RefCellDevice<I2cDriver>>,
) -> Result<(), EspError> {
    debug!("Starting main loop");
    let mut measurements: AllocRingBuffer<(u64, Option<SensorData>, Option<f32>)> =
        AllocRingBuffer::new((24 * 60 * 60 / SEND_TIMEOUT_SEC) as usize); // Buffer large enough to hold a day of measurements

    loop {
        let co2 = measure_co2(&mut co2_sensor);
        let lux = measure_lux(&mut lux_sensor);
        let now = SystemTime::now()
            .duration_since(SystemTime::UNIX_EPOCH)
            .unwrap()
            .as_secs();

        measurements.push((now, co2, lux));
        match connect_wifi(&mut wifi) {
            Ok(_) => {
                while let Some((now, co2, lux)) = measurements.dequeue() {
                    match send_data(now, &co2, lux) {
                        Ok(_) => {}
                        Err(err) => {
                            error!("Error while sending data: {:?}", err);
                            measurements.push((now, co2, lux));
                            break;
                        }
                    }
                }

                std::thread::sleep(Duration::from_millis(1000));

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

fn measure_co2(co2_sensor: &mut Scd4x<RefCellDevice<I2cDriver>, Delay>) -> Option<SensorData> {
    co2_sensor.wake_up();
    co2_sensor.wake_up(); // For some reason if you just do the one wakeup it doesn't work, need to check it with an LA or scope
    std::thread::sleep(Duration::from_millis(20)); // according to spec, since wake_up doesn't get an ACK

    let _ = co2_sensor.measure_single_shot(); // Discarding the first reading after waking up, according to the spec
    let result = co2_sensor.measure_single_shot();
    let measurement: Option<SensorData> = match result {
        Ok(_) => match co2_sensor.measurement() {
            Ok(measurement) => {
                info!(
                    "CO2: {:?}, Humidity: {} RH, Temperature: {} C",
                    measurement.co2, measurement.humidity, measurement.temperature
                );
                Some(measurement)
            }
            Err(error) => {
                error!("Error trying to measure co2: {:?}", error);
                None
            }
        },
        Err(_) => None,
    };
    let _ = co2_sensor.power_down();
    measurement
}

fn measure_lux(
    lux_sensor: &mut tsl2591_eh_driver::Driver<RefCellDevice<I2cDriver>>,
) -> Option<f32> {
    let mut current_gain = tsl2591_eh_driver::Gain::MED;
    let current_scan = tsl2591_eh_driver::IntegrationTimes::_100MS;

    loop {
        lux_sensor.set_gain(current_gain).unwrap();
        lux_sensor.set_timing(current_scan).unwrap();

        lux_sensor.enable().unwrap();
        let mut loop_count = 0;
        while loop_count < 10 {
            let lux_sensor_status = lux_sensor.get_status().unwrap();
            if lux_sensor_status.avalid() {
                println!("Lux sensor status: {:?}", lux_sensor_status);
                break;
            } else {
                loop_count = loop_count + 1;
                std::thread::sleep(Duration::from_millis(100));
            }
        }

        let (ch0, ch1) = lux_sensor.get_channel_data().unwrap();
        lux_sensor.disable().unwrap();

        match lux_sensor.calculate_lux(ch0, ch1) {
            Ok(lux) => {
                if lux.is_nan() {
                    // Basically we got an underflow
                    match increment_gain(current_gain) {
                        Ok(gain) => {
                            current_gain = gain;
                        }
                        // We are already at max gain, we can consider this to be pitch-black
                        Err(_) => return Some(0.0),
                    }
                } else if lux.is_infinite() {
                    return None
                } else {
                    info!("Lux: {} lx", lux);
                    return Some(lux);
                }
            }
            // We have an overflow
            Err(_) => {
                match decrement_gain(current_gain) {
                    Ok(gain) => {
                        current_gain = gain;
                    }
                    // If we are at the lowest gain already and are still getting an overflow we can return the brightest sunlight levels
                    Err(_) => return None,
                }
            }
        }
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

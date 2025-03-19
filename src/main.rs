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
use esp_idf_svc::wifi::{AuthMethod, BlockingWifi, ClientConfiguration, Configuration, EspWifi};
use log::{debug, error, info, trace};
use scd4x::types::SensorData;
use scd4x::Scd4x;
use std::cell::RefCell;

use esp_idf_svc::sys::EspError;
use esp_idf_svc::wifi::PmfConfiguration::NotCapable;
use esp_idf_svc::wifi::ScanMethod::FastScan;

const SSID: &str = "***REMOVED***";
const PASSWORD: &str = "***REMOVED***";

const HOST: &str = "192.168.24.1";
const PORT: &str = "2003";

const SEND_TIMEOUT_SEC: u64 = 600;

fn preamble() {
    esp_idf_svc::sys::link_patches();
    EspLogger::initialize_default();
}

fn get_co2_sensor<'a, 'b>(
    i2c: RefCellDevice<'b, I2cDriver<'a>>,
) -> Scd4x<RefCellDevice<'b, I2cDriver<'a>>, Delay> {
    println!("Setting up a sensor");
    let mut sensor = Scd4x::new(i2c, Delay::new_default());
    println!("Waking up a sensor");

    //sensor.wake_up();
    println!("Stopping periodic measurement in a sensor");
    sensor.stop_periodic_measurement().unwrap();
    println!("Re-initializing a sensor");
    sensor.reinit().unwrap();

    let serial = sensor.serial_number().unwrap();
    println!("serial: {:#04x}", serial);
    sensor
}

fn main() -> anyhow::Result<()> {
    println!("Shieeeet");
    preamble();

    let mut peripherals = Peripherals::take()?;

    let config = I2cConfig::new().baudrate(100u32.kHz().into());
    let i2c = I2cDriver::new(
        peripherals.i2c0,
        peripherals.pins.gpio9,
        peripherals.pins.gpio8,
        &config,
    )?;
    let i2c_ref_cell = RefCell::new(i2c);

    let co2_sensor = get_co2_sensor(RefCellDevice::new(&i2c_ref_cell));

    let sys_loop = EspSystemEventLoop::take()?;
    let nvs = EspDefaultNvsPartition::take()?;
    let wifi = BlockingWifi::wrap(
        EspWifi::new(&mut peripherals.modem, sys_loop.clone(), Some(nvs))?,
        sys_loop.clone(),
    )?;

    trace!("Calling run");

    run(wifi, co2_sensor)?;
    Ok(())
}

fn send_data(data: &SensorData) -> Result<(), io::Error> {
    let mut stream = TcpStream::connect(std::format!("{}:{}", HOST, PORT))?;

    stream.write_all(format!("sensors.hbase.cabinet.temp {} -1\n", data.temperature).as_bytes())?;
    stream.write_all(format!("sensors.hbase.cabinet.humidity {} -1\n", data.humidity).as_bytes())?;
    stream.write_all(format!("sensors.hbase.cabinet.co2 {} -1\n", data.co2).as_bytes())?;

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
) -> Result<(), EspError> {
    debug!("Starting main loop");
    loop {
        co2_sensor.wake_up();
        co2_sensor.wake_up();  // For some reason if you just do the one wakeup it doesn't work, need to check it with an LA or scope
        std::thread::sleep(Duration::from_millis(20));  // according to spec, since wake_up doesn't get an ACK

        co2_sensor.measure_single_shot().unwrap(); // Discarding the first reading after waking up, according to the spec
        co2_sensor.measure_single_shot().unwrap();
        let measurement = co2_sensor.measurement();
        co2_sensor.power_down().unwrap();
        match measurement {
            Ok(measurement) => {
                match connect_wifi(&mut wifi) {
                    Ok(_) => {}
                    Err(error) => {
                        error!("Error while trying to connect to wifi: {:?}", error);
                    }
                };

                info!(
                    "CO2: {}, Temperature: {} C, Humidity: {} RH",
                    measurement.co2, measurement.temperature, measurement.humidity
                );
                match send_data(&measurement) {
                    Ok(_) => {}
                    Err(err) => {
                        error!("Error while sending data: {:?}", err);
                    }
                }

                match disconnect_wifi(&mut wifi) {
                    Ok(_) => {}
                    Err(error) => {
                        error!("Error while trying to disconnect from wifi: {:?}", error);
                    }

                }
            }
            Err(error) => {
                error!("Error while measuring: {:?}", error);
            }
        }

        std::thread::sleep(Duration::from_secs(SEND_TIMEOUT_SEC));
    }
}

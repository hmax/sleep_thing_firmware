use std::io;
use std::io::Write;
use std::net::TcpStream;

use core::ffi::CStr;
use core::pin::pin;
use core::time::Duration;
use std::cell::RefCell;
use esp_idf_svc::hal::delay::Delay;
use esp_idf_svc::hal::i2c::{I2cConfig, I2cDriver};
use esp_idf_svc::hal::prelude::Peripherals;
use esp_idf_svc::hal::units::FromValueType;
use esp_idf_svc::log::EspLogger;
use esp_idf_svc::wifi::{AuthMethod, BlockingWifi, ClientConfiguration, Configuration, EspWifi};
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use log::{debug, error, info, trace};
use scd4x::Scd4x;
use scd4x::types::SensorData;
use embedded_hal_bus::i2c;
use embedded_hal_bus::i2c::RefCellDevice;
use esp_idf_svc::eventloop::{BackgroundLoopConfiguration, EspBackgroundEventLoop, EspEvent, EspEventDeserializer, EspEventPostData, EspEventSerializer, EspEventSource, EspSystemEventLoop};

use esp_idf_svc::hal::delay;
use esp_idf_svc::sys::EspError;
use esp_idf_svc::timer::EspTaskTimerService;
use esp_idf_svc::wifi::PmfConfiguration::NotCapable;
use esp_idf_svc::wifi::ScanMethod::FastScan;

const SSID: &str = "***REMOVED***";
const PASSWORD: &str = "***REMOVED***";

const HOST: &str = "192.168.24.1";
const PORT: &str = "2003";

const MEASURE_TIMEOUT_SEC: u64 = 600;
const SEND_TIMEOUT_SEC: u64 = 600;


type CO2Measurement = SensorData;
fn preamble() {
    esp_idf_svc::sys::link_patches();
    EspLogger::initialize_default();
}

fn get_sensor<'a, 'b>(
    i2c: RefCellDevice<'b, I2cDriver<'a>>
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
    return sensor;
}

fn main() -> anyhow::Result<()> {
    println!("Shieeeet");
    preamble();

    let mut peripherals = Peripherals::take()?;

    let config = I2cConfig::new().baudrate(100.kHz().into());
    let i2c = I2cDriver::new(peripherals.i2c0, peripherals.pins.gpio9, peripherals.pins.gpio8, &config).unwrap();
    let i2c_ref_cell = RefCell::new(i2c);

    let mut co2_sensor = get_sensor(RefCellDevice::new(&i2c_ref_cell));

    info!("Starting the first measurement... (5 sec)");

    match co2_sensor.start_periodic_measurement() {
        Ok(_) => {
            println!("Measurements started")
        }
        Err(e) => {
            panic!("Measurements cannot be started: {:?}", e)

        }
    };

    let user_loop = EspBackgroundEventLoop::new(&BackgroundLoopConfiguration::default())?;
    let sys_loop = EspSystemEventLoop::take()?;
    let nvs = EspDefaultNvsPartition::take()?;
    let wifi = BlockingWifi::wrap(
        EspWifi::new(&mut peripherals.modem, sys_loop.clone(), Some(nvs))?,
        sys_loop.clone(),
    )?;

    trace!("Calling run");

    run(wifi, co2_sensor, user_loop)?;
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
    trace!("connect_wifi");
    if wifi.is_started()? {
        if wifi.is_connected()? {
            trace!("wifi.disconnect");
            wifi.disconnect()?;
        }
        trace!("wifi.stop");
        wifi.stop()?;
    }

    let wifi_configuration: Configuration = Configuration::Client(ClientConfiguration {
        ssid: SSID.try_into().unwrap(),
        bssid: None,
        auth_method: AuthMethod::WPA2Personal,
        password: PASSWORD.try_into().unwrap(),
        channel: None,
        scan_method: FastScan,
        pmf_cfg: NotCapable,
    });

    trace!("wifi.set_configuration");
    wifi.set_configuration(&wifi_configuration)?;

    trace!("wifi.start");
    wifi.start()?;

    trace!("wifi.connect");
    wifi.connect()?;

    trace!("wifi.wait_netif_up");
    wifi.wait_netif_up()?;
    Ok(())
}

fn run(mut wifi: BlockingWifi<EspWifi>, mut co2_sensor: Scd4x<RefCellDevice<I2cDriver>, Delay>, mut event_loop: EspBackgroundEventLoop) -> Result<(), EspError> {
    println!("Making measurements vector");
    let mut measurements: Vec<CO2Measurement> = Vec::with_capacity(100);

    // Post events using a callback-based timer
    println!("Creating timer service");
    let timer_service = EspTaskTimerService::new()?;
    println!("Creating measure timer");
    let measure_timer = {
        let sys_loop = event_loop.clone();
        timer_service.timer(move || {
            trace!("Posting measure event");
            sys_loop
                .post::<UserEvent>(&UserEvent::Measure, delay::BLOCK)
                .unwrap();
        })?
    };


    let send_timer = {
        let sys_loop = event_loop.clone();
        timer_service.timer(move || {
            trace!("Posting send event");
            sys_loop
                .post::<UserEvent>(&UserEvent::Send, delay::BLOCK)
                .unwrap();
        })?
    };

    // Let it trigger every second
    println!("Posting first connect event");
    event_loop.post::<UserEvent>(&UserEvent::WifiReconnect, delay::BLOCK)?;
    println!("Setting up measure_timer delay");
    measure_timer.every(Duration::from_secs(MEASURE_TIMEOUT_SEC))?;
    send_timer.every(Duration::from_secs(SEND_TIMEOUT_SEC))?;


    println!("Starting event loop");
    esp_idf_svc::hal::task::block_on(pin!(async move {
        // Fetch posted events with an async subscription as well
        let mut subscription = event_loop.subscribe_async::<UserEvent>()?;

        loop {
            let event = subscription.recv().await?;
            debug!("[Subscribe async] Got event: {:?}", event);
            match event{
                UserEvent::Measure => {
                    trace!("Measuring");
                    measurements.push(co2_sensor.measurement().unwrap());
                }
                UserEvent::Send => {
                    trace!("Sending");
                    while !measurements.is_empty() {
                        let measurement = measurements.pop().unwrap();
                        info!("CO2: {}, Temperature: {} C, Humidity: {} RH", measurement.co2, measurement.temperature, measurement.humidity);
                        match send_data(&measurement) {
                            Ok(_) => {}
                            Err(err) => {
                                event_loop.post::<UserEvent>(&UserEvent::WifiReconnect, delay::BLOCK).unwrap();
                                measurements.push(measurement);
                                error!("Error while sending data: {:?}", err);
                                break;
                            }
                        }
                    }
                }
                UserEvent::WifiReconnect => {
                    println!("Wifi connecting");
                    match connect_wifi(&mut wifi) {
                        Ok(_) => {}
                        Err(error) => {
                            error!("Error while trying to connect to wifi: {:?}", error);
                            event_loop.post::<UserEvent>(&UserEvent::WifiReconnect, delay::BLOCK)?;
                        }
                    };
                }
                UserEvent::NtpSync => {}
            }
        }
    }))
}

#[allow(dead_code)]
#[derive(Copy, Clone, Debug)]
enum UserEvent {
    Measure,
    Send,
    WifiReconnect,
    NtpSync,
}

unsafe impl EspEventSource for UserEvent {
    fn source() -> Option<&'static CStr> {
        // String should be unique across the whole project and ESP IDF
        Some(CStr::from_bytes_with_nul(b"A\0").unwrap())
    }
}

impl EspEventSerializer for UserEvent {
    type Data<'a> = UserEvent;

    fn serialize<F, R>(event: &Self::Data<'_>, f: F) -> R
        where
            F: FnOnce(&EspEventPostData) -> R,
    {
        // Go the easy way since our payload implements Copy and is `'static`
        f(&unsafe { EspEventPostData::new(Self::source().unwrap(), Self::event_id(), event) })
    }
}

impl EspEventDeserializer for UserEvent {
    type Data<'a> = UserEvent;

    fn deserialize<'a>(data: &EspEvent<'a>) -> Self::Data<'a> {
        // Just as easy as serializing
        *unsafe { data.as_payload::<UserEvent>() }
    }
}
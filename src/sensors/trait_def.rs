use embedded_hal_bus::i2c::RcDevice;
use esp_idf_svc::hal::i2c::I2cDriver;

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

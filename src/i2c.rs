use feather_rp2040::hal::{
    gpio::{
        FunctionI2C,
        pin::bank0::{Gpio2, Gpio3},
        Pin,
    },
    i2c::I2C,
    pac,
};

pub type FeatherI2C = I2C<pac::I2C1, (Pin<Gpio2, FunctionI2C>, Pin<Gpio3, FunctionI2C>)>;

#[macro_export]
macro_rules! feather_i2c_init {
    ($i2c:expr, $sda:expr, $scl:expr, $i2cFreq:expr, $resets:expr, $sysFreq:expr) =>
        (I2C::i2c1($i2c, $sda, $scl, $i2cFreq, $resets, $sysFreq))
}

pub(crate) use feather_i2c_init;

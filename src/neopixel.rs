use feather_rp2040::hal::{
    gpio::pin::bank0::Gpio16,
    pac::PIO0,
    pio::SM0,
};

use crate::ws2812::Ws2812;

pub type FeatherNeopixel = Ws2812<PIO0, SM0, Gpio16>;

#[macro_export]
macro_rules! feather_neopixel_init {
    ($pin:expr, $pio:expr, $sm:expr, $freq:expr) => (Ws2812::new($pin, $pio, $sm, $freq))
}

pub(crate) use feather_neopixel_init;

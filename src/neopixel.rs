use core::iter::once;
use feather_rp2040::hal::{
    gpio::pin::bank0::Gpio16,
    pac::PIO0,
    pio::SM0,
};
use heapless::{String, Vec};
use smart_leds::{RGB, SmartLedsWrite};
use ufmt::uwrite;

use crate::console::status;
use crate::parser::{CommandResult, MAX_TOKENS, MSG_SIZE};
use crate::ws2812::Ws2812;

pub type FeatherNeopixel = Ws2812<PIO0, SM0, Gpio16>;

#[macro_export]
macro_rules! feather_neopixel_init {
    ($pin:expr, $pio:expr, $sm:expr, $freq:expr) => (Ws2812::new($pin, $pio, $sm, $freq))
}

#[derive(PartialEq, Eq)]
enum State {
    Idle,
    Start,
}

pub struct Neopixel {
    state: State,
    value: RGB<u8>,
}

impl Neopixel {
    pub fn new() -> Neopixel {
        Self { state: State::Start, value: RGB::new(0, 0, 0) }
    }

    pub fn process(&mut self, ws2812: &mut FeatherNeopixel, tokens: &Vec<&str, MAX_TOKENS>) -> CommandResult {
        if tokens.len() <= 0 || tokens[0] != "neo" {
            return CommandResult::NotHandled;
        }

        if tokens.len() == 2 && tokens[1] == "get" {
            let mut s: String<MSG_SIZE> = String::new();
            let _ = uwrite!(s, "neo: value: {} {} {}", self.value.r, self.value.g, self.value.b);
            return CommandResult::Result(s);
        } else if tokens.len() == 2 {
            match tokens[1] {
                "blue" => { self.value.r = 0; self.value.g = 0; self.value.b = 16; },
                "cyan" => { self.value.r = 0; self.value.g = 16; self.value.b = 16; },
                "green" => { self.value.r = 0; self.value.g = 16; self.value.b = 0; },
                "magenta" => { self.value.r = 16; self.value.g = 0; self.value.b = 16; },
                "off" => { self.value.r = 0; self.value.g = 0; self.value.b = 0; },
                "red" => { self.value.r = 16; self.value.g = 0; self.value.b = 0; },
                "white" => { self.value.r = 16; self.value.g = 16; self.value.b = 16; },
                "yellow" => { self.value.r = 16; self.value.g = 16; self.value.b = 0; },
                _ => {
                    return CommandResult::Error("unknown color");
                }
            }

            match ws2812.write(once(self.value)) {
                Ok(_) => {},
                Err(_) => return CommandResult::Error("failed setting value"),
            }
        } else if tokens.len() == 4 {
            if let (Ok(r), Ok(g), Ok(b)) = (
                u8::from_str_radix(tokens[1], 10),
                u8::from_str_radix(tokens[2], 10),
                u8::from_str_radix(tokens[3], 10)
            ) {
                self.value.r = r;
                self.value.g = g;
                self.value.b = b;

                match ws2812.write(once(self.value)) {
                    Ok(_) => {},
                    Err(_) => return CommandResult::Error("failed setting value"),
                }
            } else {
                return CommandResult::Error("invalid rgb value");
            }
        } else {
            return CommandResult::Error("invalid arguments");
        }

        return CommandResult::Handled;
    }

    pub fn service(&mut self, ws2812: &mut FeatherNeopixel) {
        match self.state {
            State::Start => {
                status("neo: initializing");

                match ws2812.write(once(self.value)) {
                    Ok(_) => {},
                    Err(_) => status("neo: failed writing initial value"),
                }

                self.state = State::Idle;
            },
            State::Idle => {},
        }
    }
}

pub(crate) use feather_neopixel_init;

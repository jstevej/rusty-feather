use core::iter::once;
use heapless::{String, Vec};
use smart_leds::{RGB, SmartLedsWrite};

use crate::neopixel::FeatherNeopixel;
use crate::usb_writer::UsbWriter;

const MAX_TOKENS: usize = 4;

pub struct CommandProcessor<const MSG_SIZE: usize> {
    resp: String<MSG_SIZE>,
}

impl<const MSG_SIZE: usize> CommandProcessor<MSG_SIZE> {
    pub fn new() -> CommandProcessor<MSG_SIZE> {
        let resp: String<MSG_SIZE> = String::new();
        Self { resp }
    }

    fn assemble(&mut self, tokens: Vec<&str, MAX_TOKENS>) -> &str {
        self.resp.clear();

        for t in tokens.iter().take(tokens.len() - 1) {
            let _ = self.resp.push_str(t);
            let _ = self.resp.push(' ');
        }

        let _ = self.resp.push_str(tokens[tokens.len() - 1]);
        self.resp.as_str()
    }

    pub fn process(
        &mut self,
        usb_writer: &mut UsbWriter,
        neopixel: &mut FeatherNeopixel,
        cmd: &String<MSG_SIZE>
    ) {
        let mut tokens: Vec<&str, MAX_TOKENS> = Vec::new();

        for token in cmd.split_ascii_whitespace() {
            let _ = tokens.push(token);
        }

        if tokens[0] == "echo" {
            // do nothing
        } else if tokens[0] == "neo" {
            if tokens.len() == 2 {
                match tokens[1] {
                    "blue" => { let _ = neopixel.write(once(RGB::from((0, 0, 16)))); },
                    "cyan" => { let _ = neopixel.write(once(RGB::from((0, 16, 16)))); },
                    "green" => { let _ = neopixel.write(once(RGB::from((0, 16, 0)))); },
                    "magenta" => { let _ = neopixel.write(once(RGB::from((16, 0, 16)))); },
                    "off" => { let _ = neopixel.write(once(RGB::from((0, 0, 0)))); },
                    "red" => { let _ = neopixel.write(once(RGB::from((16, 0, 0)))); },
                    "white" => { let _ = neopixel.write(once(RGB::from((16, 16, 16)))); },
                    "yellow" => { let _ = neopixel.write(once(RGB::from((16, 16, 0)))); },
                    _ => {
                        usb_writer.resp_err("unknown color");
                        return;
                    }
                }
            } else if tokens.len() == 4 {
                if let (Ok(r), Ok(g), Ok(b)) = (
                    u8::from_str_radix(tokens[1], 10),
                    u8::from_str_radix(tokens[2], 10),
                    u8::from_str_radix(tokens[3], 10)
                ) {
                    let value: RGB<u8> = RGB::new(r, g, b);
                    let _ = neopixel.write(once(value));
                } else {
                    usb_writer.resp_err("invalid rgb value");
                    return;
                }
            }
        } else if tokens[0] == "panic" {
            let x = [0, 1, 2];
            let i = x.len() + 1;
            let _y = x[i];
        } else {
            usb_writer.resp_err("unknown command");
            return;
        }

        usb_writer.resp_ack(self.assemble(tokens));
    }
}

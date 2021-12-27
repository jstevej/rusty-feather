use cortex_m::interrupt;
use feather_rp2040::hal::pac;
use heapless::spsc::{Producer};

pub enum MessageType {
    ACK,
    DEBUG,
    ERROR,
    INFO,
    STATUS,
}

pub const USB_TX_SIZE: usize = 1024;

pub struct Console<'a> {
    producer: Producer<'a, u8, USB_TX_SIZE>,
    term_bytes: &'a [u8],
}

impl<'a> Console<'a> {
    pub fn new(producer: Producer<'a, u8, USB_TX_SIZE>, term_bytes: &'a [u8]) -> Console<'a> {
        Self { producer, term_bytes }
    }

    pub fn writeln(&mut self, msg_type: MessageType, msg: &str) {
        let prefix = match msg_type {
            MessageType::ACK => "@ ",
            MessageType::DEBUG => "~ ",
            MessageType::ERROR => "! ",
            MessageType::INFO => "> ",
            MessageType::STATUS => "$ ",
        };

        interrupt::free(|_| {
            for b in prefix.bytes() {
                let _ = self.producer.enqueue(b);
            }

            for b in msg.bytes() {
                let _ = self.producer.enqueue(b);
            }

            for b in self.term_bytes {
                let _ = self.producer.enqueue(*b);
            }
        });

        pac::NVIC::pend(pac::Interrupt::USBCTRL_IRQ); // force usb interrupt to transmit new data
    }
}

unsafe impl<'a> Send for Console<'a> {}
unsafe impl<'a> Sync for Console<'a> {}

struct StaticConsole<'a> {
    inner: Option<Console<'a>>,
}

static mut CONSOLE: StaticConsole = StaticConsole {
    inner: None,
};

pub fn init_console(producer: Producer<'static, u8, USB_TX_SIZE>, term_bytes: &'static [u8]) {
    interrupt::free(|_| unsafe {
        CONSOLE.inner = Some(Console::new(producer, term_bytes));
    });
}

pub fn ack(msg: &str) {
    interrupt::free(|_| unsafe {
        match &mut CONSOLE.inner {
            Some(console) => console.writeln(MessageType::ACK, msg),
            None => {},
        }
    });
}

pub fn debug(msg: &str) {
    interrupt::free(|_| unsafe {
        match &mut CONSOLE.inner {
            Some(console) => console.writeln(MessageType::DEBUG, msg),
            None => {},
        }
    });
}

pub fn error(msg: &str) {
    interrupt::free(|_| unsafe {
        match &mut CONSOLE.inner {
            Some(console) => console.writeln(MessageType::ERROR, msg),
            None => {},
        }
    });
}

pub fn info(msg: &str) {
    interrupt::free(|_| unsafe {
        match &mut CONSOLE.inner {
            Some(console) => console.writeln(MessageType::INFO, msg),
            None => {},
        }
    });
}

pub fn status(msg: &str) {
    interrupt::free(|_| unsafe {
        match &mut CONSOLE.inner {
            Some(console) => console.writeln(MessageType::STATUS, msg),
            None => {},
        }
    });
}

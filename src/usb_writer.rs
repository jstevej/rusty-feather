use feather_rp2040::hal::pac;
use heapless::spsc::Producer;

pub struct UsbWriter<'a, const USB_TX_SIZE: usize> {
    producer: Producer<'a, u8, USB_TX_SIZE>,
    term_bytes: &'a [u8],
}

impl<'a, const USB_TX_SIZE: usize> UsbWriter<'a, USB_TX_SIZE> {
    pub fn new(producer: Producer<'a, u8, USB_TX_SIZE>, term_bytes: &'a [u8]) -> UsbWriter<'a, USB_TX_SIZE> {
        Self { producer, term_bytes }
    }

    pub fn ack(&mut self, msg: &str) {
        self.writeln(Some("> ".bytes()), msg.bytes());
    }

    pub fn err(&mut self, msg: &str) {
        self.writeln(Some("! ".bytes()), msg.bytes());
    }

    pub fn sts(&mut self, msg: &str) {
        self.writeln(Some("# ".bytes()), msg.bytes());
    }

    // usage examples for msg:
    //     "foo".bytes()
    //     msg.bytes() // msg: &String<N>
    //     msg.iter().cloned() // msg: Vec<u8, N>

    fn writeln<I, J>(&mut self, pre: Option<I>, msg: J)
    where
        I: Iterator<Item = u8>,
        J: Iterator<Item = u8>
    {
        if let Some(p) = pre {
            for b in p {
                let _ = self.producer.enqueue(b);
            }
        }

        for b in msg {
            let _ = self.producer.enqueue(b);
        }

        for b in self.term_bytes {
            let _ = self.producer.enqueue(*b);
        }

        // force usb interrupt
        // TODO: don't hard-code this?
        // TODO: use bsp function?

        pac::NVIC::pend(pac::Interrupt::USBCTRL_IRQ);
    }
}

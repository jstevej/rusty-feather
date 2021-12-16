#![no_std]
#![no_main]

use panic_halt as _;
use rtic::app;

#[link_section = ".boot2"] // second stage bootloader
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GD25Q64CS;

mod ws2812;

// Task Priorities:
//   * 4 (highest): high-priority tasks
//   * 3: reading/writing data from/to USB into queues
//   * 2: process_commands (processing commands received over usb)
//   * 1 (lowest): low-priority tasks

#[app(
    device = feather_rp2040::hal::pac,
    dispatchers = [ADC_IRQ_FIFO, UART1_IRQ],
    peripherals = true
)]
mod app {
    use core::iter::once;
    //use cortex_m_rt::entry;
    use embedded_hal::digital::v2::OutputPin;
    use embedded_time::{duration::*};
    use feather_rp2040::{
        hal::{
            clocks::{init_clocks_and_plls, Clock},
            gpio::{Pin, PushPullOutput},
            gpio::pin::bank0::Gpio13,
            pac,
            Sio,
            timer::{Alarm0, Alarm1},
            timer::Timer,
            usb::UsbBus as HalUsbBus,
            watchdog::Watchdog,
        },
        Pins, XOSC_CRYSTAL_FREQ,
    };
    use heapless::{String, Vec};
    use heapless::spsc::{Consumer, Producer, Queue};
    use smart_leds::{RGB};
    //use ufmt::{derive::uDebug, uwrite};
    use usb_device::{class_prelude::*, prelude::*};
    use usbd_serial::SerialPort;

    use crate::ws2812::Ws2812;

    const ALARM1_TICK: Microseconds = Microseconds(5_000_000);
    const HEARTBEAT_FAST: Microseconds = Microseconds(150_000);
    const HEARTBEAT_SLOW: Microseconds = Microseconds(800_000);
    const MAX_TOKENS: usize = 4;
    const MSG_SIZE: usize = 64;
    const TERM_BYTES: [u8; 2] = [b'\r', b'\n'];
    const TERM_STR: &str = "\r\n";
    const USB_RX_SIZE: usize = 128;
    const USB_TX_SIZE: usize = 1024;

    static mut USB_BUS: Option<UsbBusAllocator<HalUsbBus>> = None;

    #[shared]
    struct Shared {
        timer: Timer,
        usb_tx_p: Producer<'static, u8, USB_TX_SIZE>,
    }

    #[local]
    struct Local
    {
        alarm0: Alarm0,
        alarm1: Alarm1,
        neopixel: Ws2812,
        red_led: Pin<Gpio13, PushPullOutput>,
        usb_device: UsbDevice<'static, HalUsbBus>,
        usb_rx_c: Consumer<'static, u8, USB_RX_SIZE>,
        usb_rx_p: Producer<'static, u8, USB_RX_SIZE>,
        usb_serial: SerialPort<'static, HalUsbBus>,
        usb_tx_c: Consumer<'static, u8, USB_TX_SIZE>,
    }

    #[init(
        local = [
            usb_rx_q: Queue<u8, USB_RX_SIZE> = Queue::new(),
            usb_tx_q: Queue<u8, USB_TX_SIZE> = Queue::new(),
        ],
    )]
    fn init(context: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut resets = context.device.RESETS;
        let mut watchdog = Watchdog::new(context.device.WATCHDOG);

        // Initialize peripherals.

        let clocks = init_clocks_and_plls(
            XOSC_CRYSTAL_FREQ,
            context.device.XOSC,
            context.device.CLOCKS,
            context.device.PLL_SYS,
            context.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .ok()
        .unwrap();
        let sio = Sio::new(context.device.SIO); // single-cycle I/O block (gpio pins)
        let pins = Pins::new(
            context.device.IO_BANK0,
            context.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets
        );
        let mut timer = Timer::new(context.device.TIMER, &mut resets);

        // Initialize alarms.

        let mut alarm0 = timer.alarm_0().unwrap();
        let mut alarm1 = timer.alarm_1().unwrap();

        // Initialize the neopixel.

        let neopixel = Ws2812::new(
            pins.neopixel.into_mode(),
            context.device.PIO0,
            &mut resets,
            clocks.peripheral_clock.freq(),
        );

        // Initialize red LED.

        let red_led = pins.d13.into_push_pull_output();

        // Initialize USB.

        let usb_bus: UsbBusAllocator<HalUsbBus> = UsbBusAllocator::new(HalUsbBus::new(
            context.device.USBCTRL_REGS,
            context.device.USBCTRL_DPRAM,
            clocks.usb_clock,
            true,
            &mut resets,
        ));
        unsafe { USB_BUS = Some(usb_bus); }
        let bus_ref = unsafe { USB_BUS.as_ref().unwrap() };
        let usb_serial = SerialPort::new(bus_ref);
        let usb_device = UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("Acme, Inc.")
            .product("Rusty Feather")
            .serial_number("8675309")
            .device_class(2) // from https://www.usb.org/defined-class-codes
            .build();

        let (usb_rx_p, usb_rx_c) = context.local.usb_rx_q.split();
        let (usb_tx_p, usb_tx_c) = context.local.usb_tx_q.split();

        //let mut delay = cortex_m::delay::Delay::new(context.core.SYST, clocks.system_clock.freq().integer());

        // Enable interrupts.

        unsafe { pac::NVIC::unmask(pac::Interrupt::USBCTRL_IRQ); } // TODO: do via bsp function?
        alarm0.enable_interrupt(&mut timer);
        alarm1.enable_interrupt(&mut timer);

        // Schedule first timer alarms. TODO: do with rtic?

        alarm0.schedule(HEARTBEAT_FAST).unwrap();
        alarm1.schedule(ALARM1_TICK).unwrap();

        (
            Shared {
                timer,
                usb_tx_p,
            },
            Local {
                alarm0,
                alarm1,
                neopixel,
                red_led,
                usb_device,
                usb_rx_c,
                usb_rx_p,
                usb_serial,
                usb_tx_c,
            },
            init::Monotonics(),
        )
    }

    #[task(
        priority = 2,
        local = [
            cmd: String<MSG_SIZE> = String::new(),
            neopixel,
            resp: String<MSG_SIZE> = String::new(),
            usb_rx_c,
        ],
        shared = [usb_tx_p],
    )]
    fn process_commands(context: process_commands::Context) {
        let process_commands::LocalResources { cmd, neopixel, resp, usb_rx_c } = context.local;
        let process_commands::SharedResources { mut usb_tx_p } = context.shared;

        usb_tx_p.lock(|p| {
            loop {
                match usb_rx_c.dequeue() {
                    None => { break; }
                    Some(b) => {
                        if TERM_BYTES.contains(&b) {
                            if cmd.len() > 0 {
                                // TODO: figure out how to allocate tokens statically
                                let mut tokens: Vec<&str, MAX_TOKENS> = Vec::new();

                                for token in cmd.split_ascii_whitespace() {
                                    let _ = tokens.push(token);
                                }

                                if tokens[0] == "echo" {
                                    resp.clear();
                                    let _ = resp.push_str("echo:");

                                    for t in tokens.iter().rev().take(tokens.len() - 1).rev() {
                                        let _ = resp.push(' ');
                                        let _ = resp.push_str(t);
                                    }

                                    let _ = resp.push_str(TERM_STR);
                                    let _ = usb_write(p, resp.bytes());
                                } else if tokens[0] == "neo" {
                                    if tokens.len() == 2 {
                                        if tokens[1] == "red" {
                                            let value: RGB<u8> = RGB::new(16, 0, 0);
                                            let _ = neopixel.write(once(value));
                                        } else if tokens[1] == "green" {
                                            let value: RGB<u8> = RGB::new(0, 8, 0);
                                            let _ = neopixel.write(once(value));
                                        } else if tokens[1] == "blue" {
                                            let value: RGB<u8> = RGB::new(0, 0, 48);
                                            let _ = neopixel.write(once(value));
                                        } else if tokens[1] == "off" {
                                            let value: RGB<u8> = RGB::new(0, 0, 0);
                                            let _ = neopixel.write(once(value));
                                        } else {
                                            usb_writeln(p, "error: unknown color".bytes());
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
                                            usb_writeln(p, "error: invalid rgb value".bytes());
                                        }
                                    }
                                } else {
                                    usb_writeln(p, "error: unknown command".bytes());
                                }
                            }

                            cmd.clear();
                        } else {
                            let _ = cmd.push(b as char);
                        }
                    }
                }
            }
        });
    }

    #[task(
        binds = TIMER_IRQ_0,
        local = [
            alarm0,
            count: u32 = 0,
            led_state: bool = true,
            red_led,
        ],
        priority = 1,
        shared = [timer],
    )]
    fn timer_irq_0(context: timer_irq_0::Context) {
        let timer_irq_0::LocalResources { alarm0, count, led_state, red_led } = context.local;
        let timer_irq_0::SharedResources { mut timer } = context.shared;

        if *led_state {
            red_led.set_high().unwrap();
        } else {
            red_led.set_low().unwrap();
        }

        *led_state = !*led_state;

        timer.lock(|t| {
            alarm0.clear_interrupt(t);
            *count = *count + 1;
            if *count == 4 {
                *count = 0;
                let _ = alarm0.schedule(HEARTBEAT_SLOW);
            } else {
                let _ = alarm0.schedule(HEARTBEAT_FAST);
            }
        });
    }

    #[task(
        binds = TIMER_IRQ_1,
        local = [alarm1],
        priority = 1,
        shared = [timer, usb_tx_p],
    )]
    fn timer_irq_1(context: timer_irq_1::Context) {
        let timer_irq_1::LocalResources { alarm1 } = context.local;
        let timer_irq_1::SharedResources { timer, usb_tx_p } = context.shared;

        (timer, usb_tx_p).lock(|t, p| {
            usb_writeln(p, "tick".bytes());
            alarm1.clear_interrupt(t);
            let _ = alarm1.schedule(ALARM1_TICK);
        });

    }

    #[task(
        binds = USBCTRL_IRQ,
        local = [usb_device, usb_rx_p, usb_serial, usb_tx_c],
        priority = 3,
    )]
    fn usbctrl_irq(context: usbctrl_irq::Context) {
        let usbctrl_irq::LocalResources { usb_device, usb_rx_p, usb_serial, usb_tx_c } = context.local;

        // Read from USB into rx queue.

        let mut terminator_seen = false;

        if usb_device.poll(&mut [usb_serial]) {
            let mut buf = [0u8; 64];

            match usb_serial.read(&mut buf) {
                Err(_e) => {} // do nothing
                Ok(0) => {} // do nothing
                Ok(count) => {
                    buf.iter_mut().take(count).for_each(|b| {
                        if TERM_BYTES.contains(b) {
                            terminator_seen = true;
                        }

                        let _ = usb_rx_p.enqueue(*b);
                    });
                }
            }
        }

        if terminator_seen {
            let _ = process_commands::spawn();
        }

        // Write from tx queue to USB.

        loop {
            match usb_tx_c.dequeue() {
                None => { break; }
                Some(b) => {
                    let _ = usb_serial.write(&[b]).unwrap();
                }
            }
        }
    }
    // usage examples:
    //     usb_write(producer, "foo".bytes());
    //     usb_write(producer, msg.bytes()); // msg: &String<N>
    //     usb_write(producer, msg.iter().cloned()); // msg: Vec<u8, N>

    fn usb_write<I>(producer: &mut Producer<u8, USB_TX_SIZE>, msg: I)
    where
        I: Iterator<Item = u8>
    {

        for b in msg {
            let _ = producer.enqueue(b);
        }

        pac::NVIC::pend(pac::Interrupt::USBCTRL_IRQ); // force usb interrupt (TODO: do via bsp?)
    }

    fn usb_writeln<I>(producer: &mut Producer<u8, USB_TX_SIZE>, msg: I)
    where
        I: Iterator<Item = u8>
    {
        for b in msg {
            let _ = producer.enqueue(b);
        }

        for b in TERM_BYTES {
            let _ = producer.enqueue(b);
        }

        pac::NVIC::pend(pac::Interrupt::USBCTRL_IRQ); // force usb interrupt (TODO: do via bsp?)
    }
}

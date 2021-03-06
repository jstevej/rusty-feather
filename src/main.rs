#![no_std]
#![no_main]

use rtic::app;

#[link_section = ".boot2"] // second stage bootloader
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GD25Q64CS;

mod console;
mod fmt_fix;
mod i2c;
mod panic_led_halt;
mod parser;
mod neopixel;
mod scd41;
mod ws2812;

#[allow(unused_imports)]
use crate::panic_led_halt as _;

// Task Priorities:
//   * 4 (highest): none
//   * 3: timer_irq_0 (reading/writing data from/to USB into queues)
//   * 2: process_commands, timer_irq_1 (service sensors)
//   * 1 (lowest): timer_irq_0 (heartbeat)

#[app(
    device = feather_rp2040::hal::pac,
    dispatchers = [ADC_IRQ_FIFO, UART1_IRQ],
    peripherals = true
)]
mod app {
    use cortex_m::delay::Delay;
    use embedded_hal::digital::v2::OutputPin;
    use embedded_time::{duration::*, rate::*};
    use feather_rp2040::{
        hal::{
            clocks::{init_clocks_and_plls, Clock},
            gpio::{Pin, PushPullOutput},
            gpio::pin::bank0::{Gpio13},
            i2c::I2C,
            pac,
            pio::{PIOExt},
            Sio,
            timer::{Alarm0, Alarm1},
            timer::Timer,
            usb::UsbBus as HalUsbBus,
            watchdog::Watchdog,
        },
        Pins, XOSC_CRYSTAL_FREQ,
    };
    use heapless::String;
    use heapless::spsc::{Consumer, Producer, Queue};
    use usb_device::{class_prelude::*, prelude::*};
    use usbd_serial::SerialPort;

    use crate::console::{init_console, status, USB_TX_SIZE};
    use crate::neopixel::{feather_neopixel_init, FeatherNeopixel, Neopixel};
    use crate::parser::{MSG_SIZE, Parser};
    use crate::scd41::Scd41;
    use crate::ws2812::Ws2812;
    use crate::i2c as FeatherI2C;

    const ALARM1_TICK: Microseconds = Microseconds(5_000_000);
    const HEARTBEAT_FAST: Microseconds = Microseconds(150_000);
    const HEARTBEAT_SLOW: Microseconds = Microseconds(800_000);
    const TERM_BYTES: [u8; 2] = [b'\r', b'\n'];
    const USB_RX_SIZE: usize = 128;

    static mut USB_BUS: Option<UsbBusAllocator<HalUsbBus>> = None;

    #[shared]
    struct Shared {
        #[lock_free]
        delay: Delay,
        #[lock_free]
        i2c: FeatherI2C::FeatherI2C,
        #[lock_free]
        neopixel: Neopixel,
        #[lock_free]
        scd41: Scd41,
        timer: Timer,
        #[lock_free]
        ws2812: FeatherNeopixel,
    }

    #[local]
    struct Local
    {
        alarm0: Alarm0,
        alarm1: Alarm1,
        parser: Parser,
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

        // Initialize delay.

        let delay = Delay::new(context.core.SYST, clocks.system_clock.freq().integer());

        // Initialize I2C.

        let i2c = FeatherI2C::feather_i2c_init!(
            context.device.I2C1,
            pins.sda.into_mode(),
            pins.scl.into_mode(),
            100.kHz(),
            &mut resets,
            clocks.system_clock.freq()
        );

        // Initialize the neopixel.

        let (mut pio0, sm0, _, _, _) = context.device.PIO0.split(&mut resets);
        let ws2812 = feather_neopixel_init!(
            pins.neopixel.into_mode(),
            &mut pio0,
            sm0,
            clocks.peripheral_clock.freq()
        );
        let neopixel = Neopixel::new();

        // Initialize red LED.

        let red_led = pins.d13.into_push_pull_output();

        // Initialize SCD41.

        let scd41 = Scd41::new();

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
        init_console(usb_tx_p, &TERM_BYTES);

        // Create the command parser.

        let parser: Parser = Parser::new();

        // Enable interrupts.

        unsafe { pac::NVIC::unmask(pac::Interrupt::USBCTRL_IRQ); } // TODO: do via bsp function?
        alarm0.enable_interrupt(&mut timer);
        alarm1.enable_interrupt(&mut timer);

        // Schedule first timer alarms. TODO: do with rtic?

        alarm0.schedule(HEARTBEAT_FAST).unwrap();
        alarm1.schedule(ALARM1_TICK).unwrap();

        (
            Shared {
                delay,
                i2c,
                neopixel,
                scd41,
                timer,
                ws2812,
            },
            Local {
                alarm0,
                alarm1,
                parser,
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
        local = [
            cmd: String<MSG_SIZE> = String::new(),
            parser,
            usb_rx_c,
        ],
        priority = 2,
        shared = [delay, i2c, neopixel, scd41, ws2812]
    )]
    fn process_commands(context: process_commands::Context) {
        let process_commands::LocalResources { cmd, parser, usb_rx_c } = context.local;
        let process_commands::SharedResources { delay, i2c, neopixel, scd41, ws2812 } = context.shared;

        loop {
            match usb_rx_c.dequeue() {
                None => { break; }
                Some(b) => {
                    if TERM_BYTES.contains(&b) {
                        if let Some(tokens) = parser.tokenize(cmd) {
                            let _ = parser.handle_result(&tokens, scd41.process(delay, i2c, &tokens)) ||
                                parser.handle_result(&tokens, neopixel.process(ws2812, &tokens)) ||
                                parser.process(&tokens);
                        }

                        cmd.clear();
                    } else {
                        let _ = cmd.push(b as char);
                    }
                }
            }
        }
    }

    #[task(
        binds = TIMER_IRQ_0,
        local = [alarm0, count: u32 = 0, led_state: bool = true, red_led],
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
        local = [alarm1, firstTime: bool = true],
        priority = 2,
        shared = [delay, i2c, neopixel, scd41, timer, ws2812],
    )]
    fn timer_irq_1(context: timer_irq_1::Context) {
        let timer_irq_1::LocalResources { alarm1, firstTime } = context.local;
        let timer_irq_1::SharedResources { delay, i2c, neopixel, scd41, mut timer, ws2812 } = context.shared;

        if *firstTime {
            status("hello");
            *firstTime = false;
        }

        neopixel.service(ws2812);
        scd41.service(delay, i2c);

        timer.lock(|t| {
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
}

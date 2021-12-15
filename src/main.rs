#![no_std]
#![no_main]

use panic_halt as _;
use rtic::app;

#[link_section = ".boot2"] // second stage bootloader
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GD25Q64CS;

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
            //pac::interrupt,
            pio::PIOExt,
            //sio::Sio,
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
    use smart_leds::{RGB, SmartLedsWrite};
    use usb_device::{class_prelude::*, prelude::*};
    use usbd_serial::SerialPort;
    use ws2812_pio::Ws2812;

    const ALARM1_TICK: Microseconds = Microseconds(5_000_000);
    const HEARTBEAT_FAST: Microseconds = Microseconds(150_000);
    const HEARTBEAT_SLOW: Microseconds = Microseconds(800_000);
    const MSG_SIZE: usize = 64;
    const USB_RX_SIZE: usize = 128;
    const USB_TX_SIZE: usize = 1024;

    static mut USB_BUS: Option<UsbBusAllocator<HalUsbBus>> = None;
    #[shared]
    struct Shared {
        timer: Timer, // timer_irq0, timer_irq1
        usb_tx_p: Producer<'static, u8, USB_TX_SIZE>, // lots
    }

    #[local]
    struct Local {
        alarm0: Alarm0, // timer_irq0
        alarm1: Alarm1, // timer_irq1
        red_led: Pin<Gpio13, PushPullOutput>, // timer_irq0
        usb_device: UsbDevice<'static, HalUsbBus>, // usbctrl_irq
        usb_rx_c: Consumer<'static, u8, USB_RX_SIZE>, // usb_rx_service
        usb_rx_p: Producer<'static, u8, USB_RX_SIZE>, // usbctrl_irq
        usb_serial: SerialPort<'static, HalUsbBus>, // usbctrl_irq
        usb_tx_c: Consumer<'static, u8, USB_TX_SIZE>, // usbctrl_irq
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

        let (mut pio, sm0, _, _, _) = context.device.PIO0.split(&mut resets);
        let mut ws = Ws2812::new(
            pins.neopixel.into_mode(),
            &mut pio,
            sm0,
            clocks.peripheral_clock.freq(),
            timer.count_down()
        );
        let mut neopixel_value: RGB<u8> = RGB::default();
        neopixel_value.r = 16;
        neopixel_value.g = 8;
        neopixel_value.b = 48;
        ws.write(once(neopixel_value)).unwrap();


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

        unsafe { pac::NVIC::unmask(pac::Interrupt::USBCTRL_IRQ); } // TODO: do via function?
        alarm0.enable_interrupt(&mut timer);
        alarm1.enable_interrupt(&mut timer);

        // Schedule first timer alarm. TODO: do with rtic?

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

    // Priorities:
    //   - 4 (highest): high-priority tasks
    //   - 3: reading/writing data from/to USB into queues
    //   - 2: usb_rx_service (processing commands received over usb)
    //   - 1 (lowest): low-priority tasks

    #[task(
        binds = TIMER_IRQ_0,
        local = [alarm0, red_led, count: u32 = 0, led_state: bool = true],
        priority = 1,
        shared = [timer],
    )]
    fn timer_irq_0(context: timer_irq_0::Context) {
        let timer_irq_0::LocalResources { alarm0, red_led, count, led_state } = context.local;
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
            usb_print(p, &mut String::from("tick\r\n"));
            alarm1.clear_interrupt(t);
            let _ = alarm1.schedule(ALARM1_TICK);
        });

    }

    fn usb_print(producer: &mut Producer<u8, USB_TX_SIZE>, msg: &mut String<MSG_SIZE>) {
        let vec = unsafe { msg.as_mut_vec() };
        usb_write(producer, vec);
    }

    fn usb_write(producer: &mut Producer<u8, USB_TX_SIZE>, msg: &mut Vec<u8, MSG_SIZE>) {
        //let vec = unsafe { msg.as_mut_vec() };

        for b in msg.iter() {
            let _ = producer.enqueue(*b);
        }

        pac::NVIC::pend(pac::Interrupt::USBCTRL_IRQ); // force usb interrupt
    }

    #[task(
        priority = 2,
        local = [usb_rx_c, cmd: Vec<u8, MSG_SIZE> = Vec::new()],
        shared = [usb_tx_p],
    )]
    fn usb_rx_service(context: usb_rx_service::Context) {
        let usb_rx_service::LocalResources { usb_rx_c, cmd } = context.local;
        let usb_rx_service::SharedResources { mut usb_tx_p } = context.shared;

        usb_tx_p.lock(|p| {
            loop {
                match usb_rx_c.dequeue() {
                    None => { break; }
                    Some(b) => {
                        if b == b'\r' || b == b'\n' {
                            if cmd.len() > 0 {
                                let _ = usb_write(p, cmd);
                                let _ = usb_print(p, &mut String::from("\r\nrx\r\n"));
                                cmd.clear();
                            }
                        } else {
                            let _ = cmd.push(b);
                        }
                    }
                }
            }
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
                        if *b == b'\r' || *b == b'\n' {
                            terminator_seen = true;
                        }

                        let _ = usb_rx_p.enqueue(*b);
                    });
                }
            }
        }

        if terminator_seen {
            let _ = usb_rx_service::spawn();
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

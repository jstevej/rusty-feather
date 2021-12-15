#![no_std]
#![no_main]

use panic_halt as _;
use rtic::app;

#[link_section = ".boot2"] // second stage bootloader
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GD25Q64CS;

#[app(device = feather_rp2040::hal::pac, peripherals = true)]
mod app {
     static mut USB_BUS: Option<UsbBusAllocator<HalUsbBus>> = None;

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
            timer::Alarm0,
            timer::Timer,
            usb::UsbBus as HalUsbBus,
            watchdog::Watchdog,
        },
        Pins, XOSC_CRYSTAL_FREQ,
    };
    use smart_leds::{RGB, SmartLedsWrite};
    use usb_device::{class_prelude::*, prelude::*};
    use usbd_serial::SerialPort;
    use ws2812_pio::Ws2812;

    #[shared]
    struct Shared {
        alarm0: Alarm0,
        red_led: Pin<Gpio13, PushPullOutput>,
        timer: Timer,
        usb_device: UsbDevice<'static, HalUsbBus>,
        usb_serial: SerialPort<'static, HalUsbBus>,
    }

    #[local]
    struct Local {}

    #[init]
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

        // Initialize alarm 0.

        let mut alarm0 = timer.alarm_0().unwrap();

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


        //let mut delay = cortex_m::delay::Delay::new(context.core.SYST, clocks.system_clock.freq().integer());

        // Enable interrupts.

        unsafe { pac::NVIC::unmask(pac::Interrupt::USBCTRL_IRQ); } // TODO: do via function?
        alarm0.enable_interrupt(&mut timer);

        // Schedule first timer alarm. TODO: do with rtic?

        alarm0.schedule(150_000.microseconds()).unwrap();

        (Shared { alarm0, red_led, timer, usb_device, usb_serial }, Local {}, init::Monotonics())
    }

    #[task(
        binds = TIMER_IRQ_0,
        local = [count: u32 = 0, led_state: bool = true],
        shared = [alarm0, red_led, timer],
    )]
    fn timer_irq_0(mut context: timer_irq_0::Context) {
        let timer_irq_0::LocalResources { count, led_state } = context.local;
        let timer_irq_0::SharedResources { alarm0, ref mut red_led, timer } = context.shared;

        if *led_state {
            red_led.lock(|led| led.set_high().unwrap());
        } else {
            red_led.lock(|led| led.set_low().unwrap());
        }

        *led_state = !*led_state;

        (alarm0, timer).lock(|a, t| {
            a.clear_interrupt(t);
            *count = *count + 1;
            if *count == 4 {
                *count = 0;
                let _ = a.schedule(800_000.microseconds());
            } else {
                let _ = a.schedule(150_000.microseconds());
            }
        });
    }

    #[task(
        binds = USBCTRL_IRQ,
        shared = [usb_device, usb_serial],
    )]
    fn usbctrl_irq(context: usbctrl_irq::Context) {
        let usbctrl_irq::SharedResources { usb_device, usb_serial } = context.shared;

        (usb_device, usb_serial).lock(|d, s| {
            if d.poll(&mut [s]) {
                let mut buf = [0u8; 64];

                match s.read(&mut buf) {
                    Err(_e) => {
                        // do nothing
                    }
                    Ok(0) => {
                        // do nothing
                    }
                    Ok(count) => {
                        buf.iter_mut().take(count).for_each(|b| {
                            b.make_ascii_uppercase();
                        });

                        let mut wr_ptr = &buf[..count];

                        while !wr_ptr.is_empty() {
                            let _ = s.write(wr_ptr).map(|len| {
                                wr_ptr = &wr_ptr[len..];
                            });
                        }
                    }
                }
            }
        });
    }
}

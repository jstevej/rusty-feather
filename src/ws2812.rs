// This is a hacked version of [ws2812-pio-rs](https://github.com/ithinuel/ws2812-pio-rs), based on
// the source from commit 4f0d81e594ea9934f9c4c38ed9824ad0cce4ebb5 on December 11, 2021.
//
// This version removes the 60 microsecond wait before each write. This removes the dependency on
// the countdown timer, which resolves the following complaint from rtic.
//
// error[E0277]: `*const ()` cannot be shared between threads safely
//    --> src/main.rs:20:1
//     |
// 20  | / #[app(
// 21  | |     device = feather_rp2040::hal::pac,
// 22  | |     dispatchers = [ADC_IRQ_FIFO, UART1_IRQ],
// 23  | |     peripherals = true
// 24  | | )]
//     | |__^ `*const ()` cannot be shared between threads safely
//     |
//     = help: within `feather_rp2040::rp2040_hal::Timer`, the trait `Sync` is not implemented for `*const ()`
//     = note: required because it appears within the type `PhantomData<*const ()>`
//     = note: required because it appears within the type `TIMER`
//     = note: required because it appears within the type `feather_rp2040::rp2040_hal::Timer`
//     = note: required because of the requirements on the impl of `Send` for `&'static feather_rp2040::rp2040_hal::Timer`
//     = note: required because it appears within the type `feather_rp2040::rp2040_hal::timer::CountDown<'static>`
//     = note: required because it appears within the type `ws2812_pio::Ws2812<feather_rp2040::rp2040_pac::PIO0, feather_rp2040::rp2040_hal::pio::SM0, feather_rp2040::rp2040_hal::timer::CountDown<'static>, feather_rp2040::rp2040_hal::gpio::bank0::Gpio16>`
// note: required by a bound in `assert_send`
//    --> /Users/sjoiner/.cargo/registry/src/github.com-1ecc6299db9ec823/cortex-m-rtic-0.6.0-rc.4/src/export.rs:106:8
//     |
// 106 |     T: Send,
//     |        ^^^^ required by this bound in `assert_send`
//     = note: this error originates in the attribute macro `app` (in Nightly builds, run with -Z macro-backtrace for more info)
//
// Some errors have detailed explanations: E0277, E0412.
// For more information about an error, try `rustc --explain E0277`.


use cortex_m;
use embedded_time::{
    fixed_point::FixedPoint,
};
use feather_rp2040::hal::{
    gpio::{Function, FunctionConfig, Pin, PinId, ValidPinMode},
    pio::{PIOExt, StateMachineIndex, Tx, UninitStateMachine, PIO},
};
use smart_leds_trait::SmartLedsWrite;

/// Instance of WS2812 LED chain.
pub struct Ws2812<P, SM, I>
where
    I: PinId,
    P: PIOExt + FunctionConfig,
    Function<P>: ValidPinMode<I>,
    SM: StateMachineIndex,
{
    tx: Tx<(P, SM)>,
    _pin: Pin<I, Function<P>>,
}

impl<P, SM, I> Ws2812<P, SM, I>
where
    I: PinId,
    P: PIOExt + FunctionConfig,
    Function<P>: ValidPinMode<I>,
    SM: StateMachineIndex,
{
    /// Creates a new instance of this driver.
    pub fn new(
        pin: Pin<I, Function<P>>,
        pio: &mut PIO<P>,
        sm: UninitStateMachine<(P, SM)>,
        clock_freq: embedded_time::rate::Hertz,
    ) -> Ws2812<P, SM, I> {
        // prepare the PIO program
        let side_set = pio::SideSet::new(false, 1, false);
        let mut a = pio::Assembler::new_with_side_set(side_set);

        const T1: u8 = 2; // start bit
        const T2: u8 = 5; // data bit
        const T3: u8 = 3; // stop bit
        const CYCLES_PER_BIT: u32 = (T1 + T2 + T3) as u32;
        const FREQ: u32 = 800_000;

        let mut wrap_target = a.label();
        let mut wrap_source = a.label();
        let mut do_zero = a.label();
        a.bind(&mut wrap_target);
        // Do stop bit
        a.out_with_delay_and_side_set(pio::OutDestination::X, 1, T3 - 1, 0);
        // Do start bit
        a.jmp_with_delay_and_side_set(pio::JmpCondition::XIsZero, &mut do_zero, T1 - 1, 1);
        // Do data bit = 1
        a.jmp_with_delay_and_side_set(pio::JmpCondition::Always, &mut wrap_target, T2 - 1, 1);
        a.bind(&mut do_zero);
        // Do data bit = 0
        a.nop_with_delay_and_side_set(T2 - 2, 0);
        a.bind(&mut wrap_source);
        let program = a.assemble_with_wrap(wrap_source, wrap_target);

        // Install the program into PIO instruction memory.
        let installed = pio.install(&program).unwrap();

        // Configure the PIO state machine.
        let div = clock_freq.integer() as f32 / (FREQ as f32 * CYCLES_PER_BIT as f32);

        let (mut sm, _, tx) = feather_rp2040::hal::pio::PIOBuilder::from_program(installed)
            // only use TX FIFO
            .buffers(feather_rp2040::hal::pio::Buffers::OnlyTx)
            // Pin configuration
            .side_set_pin_base(I::DYN.num)
            // OSR config
            .out_shift_direction(feather_rp2040::hal::pio::ShiftDirection::Left)
            .autopull(true)
            .pull_threshold(24)
            .clock_divisor(div)
            .build(sm);

        // Prepare pin's direction.
        sm.set_pindirs([(I::DYN.num, feather_rp2040::hal::pio::PinDir::Output)]);

        sm.start();

        Self { tx, _pin: pin }
    }
}

impl<P, SM, I> SmartLedsWrite for Ws2812<P, SM, I>
where
    I: PinId,
    P: PIOExt + FunctionConfig,
    Function<P>: ValidPinMode<I>,
    SM: StateMachineIndex,
{
    type Color = smart_leds_trait::RGB8;
    type Error = ();
    fn write<T, J>(&mut self, iterator: T) -> Result<(), ()>
    where
        T: Iterator<Item = J>,
        J: Into<Self::Color>,
    {
        for item in iterator {
            let color: Self::Color = item.into();
            let word =
                (u32::from(color.g) << 24) | (u32::from(color.r) << 16) | (u32::from(color.b) << 8);

            while !self.tx.write(word) {
                cortex_m::asm::nop();
            }
        }
        Ok(())
    }
}

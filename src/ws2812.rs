// This is a hacked version of [ws2812-pio-rs](https://github.com/ithinuel/ws2812-pio-rs), based on
// the source from commit 4f0d81e594ea9934f9c4c38ed9824ad0cce4ebb5 on December 11, 2021.
//
// This version hard-codes the implementation to PIO0, which is the only way I could get the
// compiler to let me specify a type for Ws2812 that I could use in my rtic resources. Trying to
// specify the type of the original implementation involved a nasty mess of generics and dyn
// traits, and even when I got that right, the compiler complained that the size couldn't be
// determined at compile time. Solving that is beyond my meager Rust skills right now.
//
// I also removed the 60 microsecond wait before each write. That simplified things further, and
// seems to not cause any problems.

use cortex_m;
use embedded_time::{duration::*};
use feather_rp2040::{
    hal::{
        gpio::{FunctionPio0, Pin, PinId },
        gpio::pin::bank0::Gpio16,
        pac::{PIO0, RESETS},
        pio::{Buffers, PinDir, PIOBuilder, PIOExt, ShiftDirection, SM0, Tx},
    },
};
use smart_leds::{RGB8};

pub struct Ws2812 {
    tx: Tx<(PIO0, SM0)>,
    _pin: Pin<Gpio16, FunctionPio0>,
}

impl Ws2812 {
    pub fn new(
        pin: Pin<Gpio16, FunctionPio0>,
        pio0: PIO0,
        resets: &mut RESETS,
        clock_freq: embedded_time::rate::Hertz,
    ) -> Ws2812 {
        let (mut pio, sm0, _, _, _) = pio0.split(resets);

        // Prepare the PIO program.

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

        let (mut sm, _, tx) = PIOBuilder::from_program(installed)
            // only use TX FIFO
            .buffers(Buffers::OnlyTx)
            // Pin configuration
            .side_set_pin_base(Gpio16::DYN.num)
            // OSR config
            .out_shift_direction(ShiftDirection::Left)
            .autopull(true)
            .pull_threshold(24)
            .clock_divisor(div)
            .build(sm0);

        // Prepare pin's direction.

        sm.set_pindirs([(Gpio16::DYN.num, PinDir::Output)]);

        sm.start();

        Self { tx, _pin: pin }
    }

    pub fn write<I>(&mut self, iterator: I) -> Result<(), ()>
    where
        I: Iterator<Item = RGB8>
    {
        //self.cd.start(60.microseconds());
        //let _ = nb::block!(self.cd.wait());

        for item in iterator {
            let color: RGB8 = item.into();
            let word =
                (u32::from(color.g) << 24) | (u32::from(color.r) << 16) | (u32::from(color.b) << 8);

            while !self.tx.write(word) {
                cortex_m::asm::nop();
            }
        }
        Ok(())
    }
}

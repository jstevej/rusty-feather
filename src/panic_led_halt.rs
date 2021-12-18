// This is just like panic_halt, but it also turns on the red LED. It assumes we've already set up
// the red LED, so we just need to set it high.

use core::{
    panic::PanicInfo,
    sync::atomic::{self, Ordering},
};
use feather_rp2040::hal::pac;

#[inline(never)]
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    let  mask = 1 << 13;
    unsafe { (*pac::SIO::ptr()).gpio_out_set.write(|w| w.bits(mask)); }
    loop { atomic::compiler_fence(Ordering::SeqCst); }
}

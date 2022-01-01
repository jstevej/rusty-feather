use feather_rp2040::hal::rom_data::float_funcs;
use ufmt::{Formatter, uDisplay, uWrite, uwrite};

pub struct FixI16 {
    v: i16, // value
    q: i32, // number of binary decimal places, e.g. Q7.8 -> q = 8
    p: i32, // number of base 10 decimal places in output
}

impl FixI16 {
    pub fn new(v: i16, q: i32, p: i32) -> FixI16 {
        Self { v, q, p }
    }
}

impl uDisplay for FixI16 {
    fn fmt<W>(&self, f: &mut Formatter<'_, W>) -> Result<(), W::Error>
        where
            W: uWrite + ?Sized
    {
        fmt_i32(f, i32::from(self.v), self.q, self.p)
    }
}

//pub struct FixI32 {
//    v: i32, // value
//    q: i32, // number of binary decimal places, e.g. Q23.8 -> q = 8
//    p: i32, // number of base 10 decimal places in output
//}
//
//impl FixI32 {
//    pub fn new(v: i32, q: i32, p: i32) -> FixI32 {
//        Self { v, q, p }
//    }
//}
//
//impl uDisplay for FixI32 {
//    fn fmt<W>(&self, f: &mut Formatter<'_, W>) -> Result<(), W::Error>
//        where
//            W: uWrite + ?Sized
//    {
//        fmt_i32(f, self.v, self.q, self.p)
//    }
//}

//pub struct FixU16 {
//    v: u16, // value
//    q: i32, // number of binary decimal places, e.g. QU6.8 -> q = 8
//    p: i32, // number of base 10 decimal places in output
//}
//
//impl FixU16 {
//    pub fn new(v: u16, q: i32, p: i32) -> FixU16 {
//        Self { v, q, p }
//    }
//}
//
//impl uDisplay for FixU16 {
//    fn fmt<W>(&self, f: &mut Formatter<'_, W>) -> Result<(), W::Error>
//        where
//            W: uWrite + ?Sized
//    {
//        fmt_u32(f, u32::from(self.v), self.q, self.p)
//    }
//}

pub struct FixU32 {
    v: u32, // value
    q: i32, // number of binary decimal places, e.g. QU22.8 -> q = 8
    p: i32, // number of base 10 decimal places in output
}

impl FixU32 {
    pub fn new(v: u32, q: i32, p: i32) -> FixU32 {
        Self { v, q, p }
    }
}

impl uDisplay for FixU32 {
    fn fmt<W>(&self, f: &mut Formatter<'_, W>) -> Result<(), W::Error>
        where
            W: uWrite + ?Sized
    {
        fmt_u32(f, self.v, self.q, self.p)
    }
}

fn fmt_i32<W>(f: &mut Formatter<'_, W>, v: i32, q: i32, p: i32) -> Result<(), W::Error>
    where
        W: uWrite + ?Sized
{
    let fadd = float_funcs::fadd();
    let fcmp = float_funcs::fcmp();
    let fix_to_float = float_funcs::fix_to_float();
    let float_to_int = float_funcs::float_to_int();
    let fmul = float_funcs::fmul();
    let fsub = float_funcs::fsub();
    let int_to_float = float_funcs::int_to_float();
    let mut pow_int = 1;
    for _ in 0..p {
        pow_int *= 10;
    }
    let pow = int_to_float(pow_int);
    let v_abs = v.abs();
    let mut v_abs_ip = v_abs >> q;
    let v_abs_fp = fix_to_float(v_abs - (v_abs_ip << q), q);
    let mut v_abs_fp_digits = fadd(fmul(pow, v_abs_fp), 0.5);
    if fcmp(v_abs_fp_digits, pow) >= 1 {
        v_abs_fp_digits = fsub(v_abs_fp_digits, pow);
        v_abs_ip += 1;
    }
    let v_abs_fp_digits_int = float_to_int(v_abs_fp_digits);
    if v < 0 {
        f.write_char('-')?;
    }
    uwrite!(f, "{}.", v_abs_ip)?;
    pow_int = 1;
    for _ in 0..p {
        if v_abs_fp_digits_int < pow_int {
            f.write_char('0')?;
        }
        pow_int *= 10;
    }
    if v_abs_fp_digits_int > 0 {
        uwrite!(f, "{}", v_abs_fp_digits_int)?;
    }
    Ok(())
}

fn fmt_u32<W>(f: &mut Formatter<'_, W>, v: u32, q: i32, p: i32) -> Result<(), W::Error>
    where
        W: uWrite + ?Sized
{
    let fadd = float_funcs::fadd();
    let fcmp = float_funcs::fcmp();
    let float_to_int = float_funcs::float_to_int();
    let fmul = float_funcs::fmul();
    let fsub = float_funcs::fsub();
    let int_to_float = float_funcs::int_to_float();
    let ufix_to_float = float_funcs::ufix_to_float();
    let mut pow_int = 1;
    for _ in 0..p {
        pow_int *= 10;
    }
    let pow = int_to_float(pow_int);
    let v32 = u32::from(v);
    let mut v_abs_ip = v32 >> q;
    let v_abs_fp = ufix_to_float(v32 - (v_abs_ip << q), q);
    let mut v_abs_fp_digits = fadd(fmul(pow, v_abs_fp), 0.5);
    if fcmp(v_abs_fp_digits, pow) >= 1 {
        v_abs_fp_digits = fsub(v_abs_fp_digits, pow);
        v_abs_ip += 1;
    }
    let v_abs_fp_digits_int = float_to_int(v_abs_fp_digits);
    uwrite!(f, "{}.", v_abs_ip)?;
    pow_int = 1;
    for _ in 0..p {
        if v_abs_fp_digits_int < pow_int {
            f.write_char('0')?;
        }
        pow_int *= 10;
    }
    if v_abs_fp_digits_int > 0 {
        uwrite!(f, "{}", v_abs_fp_digits_int)?;
    }
    Ok(())
}

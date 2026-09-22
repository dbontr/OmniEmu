use crate::state::{StateReader, StateWriter};

const T: u32 = 1 << 0;
const S: u32 = 1 << 1;
const I_MASK: u32 = 0x0000_00f0;
const Q: u32 = 1 << 8;
const M: u32 = 1 << 9;
const SR_MASK: u32 = 0x0000_03f3;

pub trait Sh2Bus {
    fn read8(&mut self, address: u32) -> u8;
    fn write8(&mut self, address: u32, value: u8);

    fn read16(&mut self, address: u32) -> u16 {
        u16::from_be_bytes([self.read8(address), self.read8(address.wrapping_add(1))])
    }

    fn write16(&mut self, address: u32, value: u16) {
        let [high, low] = value.to_be_bytes();
        self.write8(address, high);
        self.write8(address.wrapping_add(1), low);
    }

    fn read32(&mut self, address: u32) -> u32 {
        (u32::from(self.read16(address)) << 16) | u32::from(self.read16(address.wrapping_add(2)))
    }

    fn write32(&mut self, address: u32, value: u32) {
        self.write16(address, (value >> 16) as u16);
        self.write16(address.wrapping_add(2), value as u16);
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct Sh2Intc {
    ipra: u16,
    iprb: u16,
    vcrwdt: u16,
    vcra: u16,
    vcrb: u16,
    vcrc: u16,
    vcrd: u16,
}

impl Sh2Intc {
    pub(crate) fn read8(&self, address: u32) -> Option<u8> {
        let value = match address & !1 {
            0xffff_fee2 => self.ipra,
            0xffff_fe60 => self.iprb,
            0xffff_fee4 => self.vcrwdt,
            0xffff_fe62 => self.vcra,
            0xffff_fe64 => self.vcrb,
            0xffff_fe66 => self.vcrc,
            0xffff_fe68 => self.vcrd,
            _ => return None,
        };
        Some(if address & 1 == 0 {
            (value >> 8) as u8
        } else {
            value as u8
        })
    }

    pub(crate) fn write8(&mut self, address: u32, value: u8) -> bool {
        let aligned = address & !1;
        let Some(old) = self.read16(aligned) else {
            return false;
        };
        let merged = if address & 1 == 0 {
            (old & 0x00ff) | (u16::from(value) << 8)
        } else {
            (old & 0xff00) | u16::from(value)
        };
        self.write16(aligned, merged)
    }

    pub(crate) fn read16(&self, address: u32) -> Option<u16> {
        match address & !1 {
            0xffff_fee2 => Some(self.ipra),
            0xffff_fe60 => Some(self.iprb),
            0xffff_fee4 => Some(self.vcrwdt),
            0xffff_fe62 => Some(self.vcra),
            0xffff_fe64 => Some(self.vcrb),
            0xffff_fe66 => Some(self.vcrc),
            0xffff_fe68 => Some(self.vcrd),
            _ => None,
        }
    }

    pub(crate) fn write16(&mut self, address: u32, value: u16) -> bool {
        match address & !1 {
            0xffff_fee2 => self.ipra = value,
            0xffff_fe60 => self.iprb = value,
            0xffff_fee4 => self.vcrwdt = value & 0x7f7f,
            0xffff_fe62 => self.vcra = value & 0x7f7f,
            0xffff_fe64 => self.vcrb = value & 0x7f7f,
            0xffff_fe66 => self.vcrc = value & 0x7f7f,
            0xffff_fe68 => self.vcrd = value & 0x7f00,
            _ => return false,
        }
        true
    }

    pub(crate) fn dmac_level(&self) -> u8 {
        ((self.ipra >> 8) & 0x0f) as u8
    }

    pub(crate) fn wdt_level(&self) -> u8 {
        ((self.ipra >> 4) & 0x0f) as u8
    }

    pub(crate) fn divu_level(&self) -> u8 {
        (self.ipra & 0x0f) as u8
    }

    pub(crate) fn sci_level(&self) -> u8 {
        ((self.iprb >> 12) & 0x0f) as u8
    }

    pub(crate) fn frt_level(&self) -> u8 {
        ((self.iprb >> 8) & 0x0f) as u8
    }

    pub(crate) fn sci_error_vector(&self) -> u8 {
        ((self.vcra >> 8) & 0x7f) as u8
    }

    pub(crate) fn sci_receive_vector(&self) -> u8 {
        (self.vcra & 0x7f) as u8
    }

    pub(crate) fn sci_transmit_vector(&self) -> u8 {
        ((self.vcrb >> 8) & 0x7f) as u8
    }

    pub(crate) fn sci_transmit_end_vector(&self) -> u8 {
        (self.vcrb & 0x7f) as u8
    }

    pub(crate) fn frt_capture_vector(&self) -> u8 {
        ((self.vcrc >> 8) & 0x7f) as u8
    }

    pub(crate) fn frt_compare_vector(&self) -> u8 {
        (self.vcrc & 0x7f) as u8
    }

    pub(crate) fn frt_overflow_vector(&self) -> u8 {
        ((self.vcrd >> 8) & 0x7f) as u8
    }

    pub(crate) fn wdt_vector(&self) -> u8 {
        ((self.vcrwdt >> 8) & 0x7f) as u8
    }

    pub(crate) fn save(&self, out: &mut StateWriter) {
        out.u16(self.ipra);
        out.u16(self.iprb);
        out.u16(self.vcrwdt);
        out.u16(self.vcra);
        out.u16(self.vcrb);
        out.u16(self.vcrc);
        out.u16(self.vcrd);
    }

    pub(crate) fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        self.ipra = input.u16()?;
        self.iprb = input.u16()?;
        self.vcrwdt = input.u16()? & 0x7f7f;
        self.vcra = input.u16()? & 0x7f7f;
        self.vcrb = input.u16()? & 0x7f7f;
        self.vcrc = input.u16()? & 0x7f7f;
        self.vcrd = input.u16()? & 0x7f00;
        Ok(())
    }
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct Sh2Wdt {
    wtcsr: u8,
    wtcnt: u8,
    rstcsr: u8,
    clock_phase: u32,
    ovf_read: bool,
    wovf_read: bool,
}

impl Default for Sh2Wdt {
    fn default() -> Self {
        Self {
            wtcsr: 0x18,
            wtcnt: 0,
            rstcsr: 0x1f,
            clock_phase: 0,
            ovf_read: false,
            wovf_read: false,
        }
    }
}

impl Sh2Wdt {
    pub(crate) fn read8(&mut self, address: u32) -> Option<u8> {
        match address {
            0xffff_fe80 => {
                self.ovf_read |= self.wtcsr & 0x80 != 0;
                Some(self.wtcsr)
            }
            0xffff_fe81 => Some(self.wtcnt),
            0xffff_fe83 => {
                self.wovf_read |= self.rstcsr & 0x80 != 0;
                Some(self.rstcsr)
            }
            _ => None,
        }
    }

    pub(crate) fn write8(&mut self, address: u32, _value: u8) -> bool {
        matches!(address, 0xffff_fe80..=0xffff_fe83)
    }

    pub(crate) fn write16(&mut self, address: u32, value: u16) -> bool {
        let [key, data] = value.to_be_bytes();
        match address & !1 {
            0xffff_fe80 => match key {
                0x5a => self.wtcnt = data,
                0xa5 => {
                    let overflow = if self.wtcsr & 0x80 != 0 && !(self.ovf_read && data & 0x80 == 0)
                    {
                        0x80
                    } else {
                        0
                    };
                    self.wtcsr = overflow | 0x18 | (data & 0x67);
                    if self.wtcsr & 0x80 == 0 {
                        self.ovf_read = false;
                    }
                    if self.wtcsr & 0x20 == 0 {
                        self.clock_phase = 0;
                    }
                }
                _ => {}
            },
            0xffff_fe82 => match key {
                0xa5 if data == 0 && self.wovf_read => {
                    self.rstcsr &= !0x80;
                    self.wovf_read = false;
                }
                0x5a => self.rstcsr = (self.rstcsr & 0x80) | 0x1f | (data & 0x60),
                _ => {}
            },
            _ => return false,
        }
        true
    }

    fn divisor(&self) -> u32 {
        [2, 64, 128, 256, 512, 1024, 4096, 8192][usize::from(self.wtcsr & 7)]
    }

    pub(crate) fn tick(&mut self, cycles: u32) {
        if self.wtcsr & 0x20 == 0 {
            return;
        }
        let divisor = self.divisor();
        self.clock_phase = self.clock_phase.saturating_add(cycles);
        let increments = self.clock_phase / divisor;
        self.clock_phase %= divisor;
        if increments == 0 {
            return;
        }
        let total = u64::from(self.wtcnt) + u64::from(increments);
        self.wtcnt = (total & 0xff) as u8;
        if total >= 0x100 {
            if self.wtcsr & 0x40 == 0 {
                self.wtcsr |= 0x80;
            } else {
                self.rstcsr |= 0x80;
            }
        }
    }

    pub(crate) fn interrupt(&self, intc: &Sh2Intc) -> Option<(u8, u8)> {
        let level = intc.wdt_level();
        if self.wtcsr & 0xc0 == 0x80 && level != 0 {
            Some((level, intc.wdt_vector()))
        } else {
            None
        }
    }

    pub(crate) fn save(&self, out: &mut StateWriter) {
        out.u8(self.wtcsr);
        out.u8(self.wtcnt);
        out.u8(self.rstcsr);
        out.u32(self.clock_phase);
        out.u8(u8::from(self.ovf_read));
        out.u8(u8::from(self.wovf_read));
    }

    pub(crate) fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        self.wtcsr = (input.u8()? & 0xe7) | 0x18;
        self.wtcnt = input.u8()?;
        self.rstcsr = (input.u8()? & 0xe0) | 0x1f;
        self.clock_phase = input.u32()? % self.divisor();
        self.ovf_read = input.u8()? != 0;
        self.wovf_read = input.u8()? != 0;
        Ok(())
    }
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct Sh2Sci {
    smr: u8,
    brr: u8,
    scr: u8,
    tdr: u8,
    ssr: u8,
    rdr: u8,
    ssr_read: u8,
    tx_data: u8,
    tx_mpb: bool,
    tx_cycles: u32,
    tx_busy: bool,
}

impl Default for Sh2Sci {
    fn default() -> Self {
        Self {
            smr: 0,
            brr: 0xff,
            scr: 0,
            tdr: 0xff,
            ssr: 0x84,
            rdr: 0,
            ssr_read: 0,
            tx_data: 0,
            tx_mpb: false,
            tx_cycles: 0,
            tx_busy: false,
        }
    }
}

impl Sh2Sci {
    const TDRE: u8 = 0x80;
    const RDRF: u8 = 0x40;
    const ORER: u8 = 0x20;
    const FER: u8 = 0x10;
    const PER: u8 = 0x08;
    const TEND: u8 = 0x04;
    const MPB: u8 = 0x02;
    const MPBT: u8 = 0x01;
    const TIE: u8 = 0x80;
    const RIE: u8 = 0x40;
    const TE: u8 = 0x20;
    const RE: u8 = 0x10;
    const TEIE: u8 = 0x04;

    pub(crate) fn read8(&mut self, address: u32) -> Option<u8> {
        match address {
            0xffff_fe00 => Some(self.smr),
            0xffff_fe01 => Some(self.brr),
            0xffff_fe02 => Some(self.scr),
            0xffff_fe03 => Some(self.tdr),
            0xffff_fe04 => {
                self.ssr_read |= self.ssr & 0xf8;
                Some(self.ssr)
            }
            0xffff_fe05 => Some(self.rdr),
            _ => None,
        }
    }

    pub(crate) fn write8(&mut self, address: u32, value: u8) -> bool {
        match address {
            0xffff_fe00 => self.smr = value,
            0xffff_fe01 => self.brr = value,
            0xffff_fe02 => {
                let old = self.scr;
                self.scr = value;
                if old & Self::TE != 0 && value & Self::TE == 0 {
                    self.tx_busy = false;
                    self.tx_cycles = 0;
                    self.ssr |= Self::TDRE | Self::TEND;
                }
                self.start_transmit_if_ready();
            }
            0xffff_fe03 => self.tdr = value,
            0xffff_fe04 => {
                let clearable = Self::TDRE | Self::RDRF | Self::ORER | Self::FER | Self::PER;
                let clear = self.ssr_read & clearable & !value;
                self.ssr &= !clear;
                self.ssr = (self.ssr & !Self::MPBT) | (value & Self::MPBT);
                self.ssr_read &= !clear;
                self.start_transmit_if_ready();
            }
            0xffff_fe05 => {}
            _ => return false,
        }
        true
    }

    fn character_cycles(&self) -> u32 {
        let clock_scale = 1u64 << (u32::from(self.smr & 3) * 2);
        let synchronous = self.smr & 0x80 != 0;
        let clocks_per_bit = if synchronous { 4u64 } else { 32u64 };
        let bits = if synchronous {
            8u64
        } else {
            let data = if self.smr & 0x40 != 0 { 7 } else { 8 };
            let parity = u64::from(self.smr & 0x20 != 0);
            let stop = if self.smr & 0x08 != 0 { 2 } else { 1 };
            1 + data + parity + stop
        };
        (clocks_per_bit
            .saturating_mul(u64::from(self.brr) + 1)
            .saturating_mul(clock_scale)
            .saturating_mul(bits))
        .clamp(1, u64::from(u32::MAX)) as u32
    }

    fn start_transmit_if_ready(&mut self) {
        if self.tx_busy || self.scr & Self::TE == 0 || self.ssr & Self::TDRE != 0 {
            return;
        }
        self.tx_data = self.tdr;
        self.tx_mpb = self.ssr & Self::MPBT != 0;
        self.tx_cycles = self.character_cycles();
        self.tx_busy = true;
        self.ssr &= !Self::TEND;
    }

    pub(crate) fn tick(&mut self, cycles: u32) -> Option<(u8, bool)> {
        self.start_transmit_if_ready();
        if !self.tx_busy {
            return None;
        }
        if cycles < self.tx_cycles {
            self.tx_cycles -= cycles;
            return None;
        }
        self.tx_cycles = 0;
        self.tx_busy = false;
        self.ssr |= Self::TDRE | Self::TEND;
        Some((self.tx_data, self.tx_mpb))
    }

    pub(crate) fn receive(&mut self, value: u8, multiprocessor: bool) {
        if self.scr & Self::RE == 0 {
            return;
        }
        if self.ssr & Self::RDRF != 0 {
            self.ssr |= Self::ORER;
            return;
        }
        self.rdr = value;
        self.ssr = (self.ssr & !Self::MPB) | if multiprocessor { Self::MPB } else { 0 };
        self.ssr |= Self::RDRF;
    }

    pub(crate) fn interrupt(&self, intc: &Sh2Intc) -> Option<(u8, u8)> {
        let level = intc.sci_level();
        if level == 0 {
            return None;
        }
        if self.scr & Self::RIE != 0 && self.ssr & (Self::ORER | Self::FER | Self::PER) != 0 {
            return Some((level, intc.sci_error_vector()));
        }
        if self.scr & Self::RIE != 0 && self.ssr & Self::RDRF != 0 {
            return Some((level, intc.sci_receive_vector()));
        }
        if self.scr & Self::TIE != 0 && self.ssr & Self::TDRE != 0 {
            return Some((level, intc.sci_transmit_vector()));
        }
        if self.scr & Self::TEIE != 0 && self.ssr & Self::TEND != 0 {
            return Some((level, intc.sci_transmit_end_vector()));
        }
        None
    }

    pub(crate) fn save(&self, out: &mut StateWriter) {
        out.u8(self.smr);
        out.u8(self.brr);
        out.u8(self.scr);
        out.u8(self.tdr);
        out.u8(self.ssr);
        out.u8(self.rdr);
        out.u8(self.ssr_read);
        out.u8(self.tx_data);
        out.u8(u8::from(self.tx_mpb));
        out.u32(self.tx_cycles);
        out.u8(u8::from(self.tx_busy));
    }

    pub(crate) fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        self.smr = input.u8()?;
        self.brr = input.u8()?;
        self.scr = input.u8()?;
        self.tdr = input.u8()?;
        self.ssr = input.u8()?;
        self.rdr = input.u8()?;
        self.ssr_read = input.u8()? & 0xf8;
        self.tx_data = input.u8()?;
        self.tx_mpb = input.u8()? != 0;
        self.tx_cycles = input.u32()?;
        self.tx_busy = input.u8()? != 0;
        if !self.tx_busy {
            self.tx_cycles = 0;
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct Sh2Divu {
    pub(crate) dvsr: u32,
    pub(crate) dvdnth: u32,
    pub(crate) dvdntl: u32,
    pub(crate) dvcr: u32,
    pub(crate) vcrdiv: u32,
}

impl Sh2Divu {
    fn set_overflow(&mut self, negative: bool) {
        self.dvcr |= 1;
        let saturated = if negative { 0x8000_0000 } else { 0x7fff_ffff };
        self.dvdnth = saturated;
        self.dvdntl = saturated;
    }

    pub(crate) fn divide32(&mut self, dividend: u32) {
        let a = i64::from(dividend as i32);
        let b = i64::from(self.dvsr as i32);
        if b == 0 {
            self.set_overflow(false);
            return;
        }
        let quotient = a / b;
        if quotient < i64::from(i32::MIN) || quotient > i64::from(i32::MAX) {
            self.set_overflow((a < 0) ^ (b < 0));
            return;
        }
        self.dvdntl = (quotient as i32) as u32;
        self.dvdnth = ((a % b) as i32) as u32;
    }

    fn divide64(&mut self, low: u32) {
        let dividend = ((i64::from(self.dvdnth as i32)) << 32) | i64::from(low);
        let divisor = i64::from(self.dvsr as i32);
        if divisor == 0 {
            self.set_overflow(false);
            return;
        }
        let quotient = dividend / divisor;
        if quotient < i64::from(i32::MIN) || quotient > i64::from(i32::MAX) {
            self.set_overflow((dividend < 0) ^ (divisor < 0));
            return;
        }
        self.dvdntl = (quotient as i32) as u32;
        self.dvdnth = ((dividend % divisor) as i32) as u32;
    }

    pub(crate) fn read32(&self, address: u32) -> Option<u32> {
        match address & !3 {
            0xffff_ff00 => Some(self.dvsr),
            0xffff_ff04 => Some(self.dvdntl),
            0xffff_ff08 => Some(self.dvcr & 3),
            0xffff_ff0c => Some(self.vcrdiv),
            0xffff_ff10 | 0xffff_ff18 => Some(self.dvdnth),
            0xffff_ff14 | 0xffff_ff1c => Some(self.dvdntl),
            _ => None,
        }
    }

    pub(crate) fn write32(&mut self, address: u32, value: u32) -> bool {
        match address & !3 {
            0xffff_ff00 => self.dvsr = value,
            0xffff_ff04 => self.divide32(value),
            0xffff_ff08 => self.dvcr = value & 3,
            0xffff_ff0c => self.vcrdiv = value & 0x7f,
            0xffff_ff10 => self.dvdnth = value,
            0xffff_ff14 => self.divide64(value),
            0xffff_ff18 | 0xffff_ff1c => return true,
            _ => return false,
        }
        true
    }

    pub(crate) fn interrupt(&self, intc: &Sh2Intc) -> Option<(u8, u8)> {
        let level = intc.divu_level();
        if self.dvcr & 3 == 3 && level != 0 {
            Some((level, self.vcrdiv as u8))
        } else {
            None
        }
    }

    pub(crate) fn save(&self, out: &mut StateWriter) {
        out.u32(self.dvsr);
        out.u32(self.dvdnth);
        out.u32(self.dvdntl);
        out.u32(self.dvcr);
        out.u32(self.vcrdiv);
    }

    pub(crate) fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        self.dvsr = input.u32()?;
        self.dvdnth = input.u32()?;
        self.dvdntl = input.u32()?;
        self.dvcr = input.u32()? & 3;
        self.vcrdiv = input.u32()? & 0x7f;
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct Sh2DmaChannel {
    pub(crate) sar: u32,
    pub(crate) dar: u32,
    pub(crate) tcr: u32,
    pub(crate) chcr: u32,
}

#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct Sh2Dmac {
    pub(crate) channels: [Sh2DmaChannel; 2],
    pub(crate) drcr: [u8; 2],
    pub(crate) vcrdma: [u32; 2],
    pub(crate) dmaor: u32,
}

impl Sh2Dmac {
    pub(crate) fn read32(&self, address: u32) -> Option<u32> {
        match address & !3 {
            0xffff_ff80 => Some(self.channels[0].sar),
            0xffff_ff84 => Some(self.channels[0].dar),
            0xffff_ff88 => Some(self.channels[0].tcr),
            0xffff_ff8c => Some(self.channels[0].chcr),
            0xffff_ff90 => Some(self.channels[1].sar),
            0xffff_ff94 => Some(self.channels[1].dar),
            0xffff_ff98 => Some(self.channels[1].tcr),
            0xffff_ff9c => Some(self.channels[1].chcr),
            0xffff_ffa0 => Some(self.vcrdma[0]),
            0xffff_ffa8 => Some(self.vcrdma[1]),
            0xffff_ffb0 => Some(self.dmaor),
            _ => None,
        }
    }

    pub(crate) fn write32(&mut self, address: u32, value: u32) -> bool {
        match address & !3 {
            0xffff_ff80 => self.channels[0].sar = value,
            0xffff_ff84 => self.channels[0].dar = value,
            0xffff_ff88 => self.channels[0].tcr = value & 0x00ff_ffff,
            0xffff_ff8c => self.channels[0].chcr = value & 0x0000_ffff,
            0xffff_ff90 => self.channels[1].sar = value,
            0xffff_ff94 => self.channels[1].dar = value,
            0xffff_ff98 => self.channels[1].tcr = value & 0x00ff_ffff,
            0xffff_ff9c => self.channels[1].chcr = value & 0x0000_ffff,
            0xffff_ffa0 => self.vcrdma[0] = value & 0x7f,
            0xffff_ffa8 => self.vcrdma[1] = value & 0x7f,
            0xffff_ffb0 => self.dmaor = value & 0x0000_0007,
            _ => return false,
        }
        true
    }

    pub(crate) fn read_drcr(&self, address: u32) -> Option<u8> {
        match address {
            0xffff_fe71 => Some(self.drcr[0]),
            0xffff_fe72 => Some(self.drcr[1]),
            _ => None,
        }
    }

    pub(crate) fn write_drcr(&mut self, address: u32, value: u8) -> bool {
        match address {
            0xffff_fe71 => self.drcr[0] = value & 3,
            0xffff_fe72 => self.drcr[1] = value & 3,
            _ => return false,
        }
        true
    }

    pub(crate) fn interrupt(&self, intc: &Sh2Intc) -> Option<(u8, u8)> {
        let level = intc.dmac_level();
        if level == 0 {
            return None;
        }
        for channel in 0..2 {
            if self.channels[channel].chcr & 0x0006 == 0x0006 {
                return Some((level, self.vcrdma[channel] as u8));
            }
        }
        None
    }

    pub(crate) fn auto_transfer_size(&self, channel: usize) -> Option<u32> {
        let channel = self.channels[channel];
        if self.dmaor & 7 != 1
            || channel.chcr & 0x03f8 != 0x02e0
            || channel.chcr & 0x0001 == 0
            || channel.chcr & 0x0002 != 0
            || ((channel.chcr >> 14) & 3) == 3
            || ((channel.chcr >> 12) & 3) == 3
        {
            return None;
        }
        Some(match (channel.chcr >> 10) & 3 {
            0 => 1,
            1 => 2,
            2 => 4,
            3 => 16,
            _ => unreachable!(),
        })
    }

    pub(crate) fn transfer_complete(&mut self, channel: usize) {
        self.channels[channel].tcr = 0;
        self.channels[channel].chcr |= 0x0002;
    }

    pub(crate) fn save(&self, out: &mut StateWriter) {
        for channel in self.channels {
            out.u32(channel.sar);
            out.u32(channel.dar);
            out.u32(channel.tcr);
            out.u32(channel.chcr);
        }
        out.u8(self.drcr[0]);
        out.u8(self.drcr[1]);
        out.u32(self.vcrdma[0]);
        out.u32(self.vcrdma[1]);
        out.u32(self.dmaor);
    }

    pub(crate) fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        for channel in &mut self.channels {
            channel.sar = input.u32()?;
            channel.dar = input.u32()?;
            channel.tcr = input.u32()? & 0x00ff_ffff;
            channel.chcr = input.u32()? & 0x0000_ffff;
        }
        self.drcr[0] = input.u8()? & 3;
        self.drcr[1] = input.u8()? & 3;
        self.vcrdma[0] = input.u32()? & 0x7f;
        self.vcrdma[1] = input.u32()? & 0x7f;
        self.dmaor = input.u32()? & 7;
        Ok(())
    }
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct Sh2Frt {
    tier: u8,
    ftcsr: u8,
    frc: u16,
    ocra: u16,
    ocrb: u16,
    tcr: u8,
    tocr: u8,
    icr: u16,
    clock_phase: u32,
}

impl Default for Sh2Frt {
    fn default() -> Self {
        Self {
            tier: 0x01,
            ftcsr: 0,
            frc: 0,
            ocra: 0xffff,
            ocrb: 0xffff,
            tcr: 0,
            tocr: 0xe0,
            icr: 0,
            clock_phase: 0,
        }
    }
}

impl Sh2Frt {
    fn selected_ocr(&self) -> u16 {
        if self.tocr & 0x10 != 0 {
            self.ocrb
        } else {
            self.ocra
        }
    }

    fn selected_ocr_mut(&mut self) -> &mut u16 {
        if self.tocr & 0x10 != 0 {
            &mut self.ocrb
        } else {
            &mut self.ocra
        }
    }

    pub(crate) fn read8(&self, address: u32) -> Option<u8> {
        match address {
            0xffff_fe10 => Some(self.tier),
            0xffff_fe11 => Some(self.ftcsr),
            0xffff_fe12 => Some((self.frc >> 8) as u8),
            0xffff_fe13 => Some(self.frc as u8),
            0xffff_fe14 => Some((self.selected_ocr() >> 8) as u8),
            0xffff_fe15 => Some(self.selected_ocr() as u8),
            0xffff_fe16 => Some(self.tcr),
            0xffff_fe17 => Some(self.tocr),
            0xffff_fe18 => Some((self.icr >> 8) as u8),
            0xffff_fe19 => Some(self.icr as u8),
            _ => None,
        }
    }

    pub(crate) fn write8(&mut self, address: u32, value: u8) -> bool {
        match address {
            0xffff_fe10 => self.tier = (value & 0x8e) | 0x01,
            0xffff_fe11 => self.ftcsr = (self.ftcsr & value & 0x8e) | (value & 0x01),
            0xffff_fe12 => self.frc = (u16::from(value) << 8) | (self.frc & 0x00ff),
            0xffff_fe13 => self.frc = (self.frc & 0xff00) | u16::from(value),
            0xffff_fe14 => {
                let old = self.selected_ocr();
                *self.selected_ocr_mut() = (u16::from(value) << 8) | (old & 0x00ff);
            }
            0xffff_fe15 => {
                let old = self.selected_ocr();
                *self.selected_ocr_mut() = (old & 0xff00) | u16::from(value);
            }
            0xffff_fe16 => self.tcr = value & 0x83,
            0xffff_fe17 => self.tocr = 0xe0 | (value & 0x13),
            0xffff_fe18 | 0xffff_fe19 => {}
            _ => return false,
        }
        true
    }

    fn increment(&mut self) {
        let next = self.frc.wrapping_add(1);
        let match_a = next == self.ocra;
        let match_b = next == self.ocrb;
        if match_a {
            self.ftcsr |= 0x08;
        }
        if match_b {
            self.ftcsr |= 0x04;
        }
        if next == 0 {
            self.ftcsr |= 0x02;
        }
        self.frc = if match_a && self.ftcsr & 0x01 != 0 {
            0
        } else {
            next
        };
    }

    pub(crate) fn tick(&mut self, cycles: u32) {
        let divisor = match self.tcr & 3 {
            0 => 8,
            1 => 32,
            2 => 128,
            _ => return,
        };
        self.clock_phase = self.clock_phase.saturating_add(cycles);
        while self.clock_phase >= divisor {
            self.clock_phase -= divisor;
            self.increment();
        }
    }

    pub(crate) fn capture(&mut self) {
        self.icr = self.frc;
        self.ftcsr |= 0x80;
    }

    pub(crate) fn interrupt(&self, intc: &Sh2Intc) -> Option<(u8, u8)> {
        let level = intc.frt_level();
        if level == 0 {
            return None;
        }
        if self.ftcsr & 0x80 != 0 && self.tier & 0x80 != 0 {
            return Some((level, intc.frt_capture_vector()));
        }
        if (self.ftcsr & 0x08 != 0 && self.tier & 0x08 != 0)
            || (self.ftcsr & 0x04 != 0 && self.tier & 0x04 != 0)
        {
            return Some((level, intc.frt_compare_vector()));
        }
        if self.ftcsr & 0x02 != 0 && self.tier & 0x02 != 0 {
            return Some((level, intc.frt_overflow_vector()));
        }
        None
    }

    pub(crate) fn save(&self, out: &mut StateWriter) {
        out.u8(self.tier);
        out.u8(self.ftcsr);
        out.u16(self.frc);
        out.u16(self.ocra);
        out.u16(self.ocrb);
        out.u8(self.tcr);
        out.u8(self.tocr);
        out.u16(self.icr);
        out.u32(self.clock_phase);
    }

    pub(crate) fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        self.tier = (input.u8()? & 0x8e) | 0x01;
        self.ftcsr = input.u8()? & 0x8f;
        self.frc = input.u16()?;
        self.ocra = input.u16()?;
        self.ocrb = input.u16()?;
        self.tcr = input.u8()? & 0x83;
        self.tocr = 0xe0 | (input.u8()? & 0x13);
        self.icr = input.u16()?;
        self.clock_phase = input.u32()? % 128;
        Ok(())
    }
}

#[derive(Clone, Debug, Default)]
pub struct Sh2 {
    pub r: [u32; 16],
    pub pc: u32,
    pub pr: u32,
    pub gbr: u32,
    pub vbr: u32,
    pub mach: u32,
    pub macl: u32,
    pub sr: u32,
    pub cycles: u64,
    sleeping: bool,
    pending_branch: Option<u32>,
}

impl Sh2 {
    pub fn reset(&mut self, pc: u32, vbr: u32, stack: u32) {
        *self = Self::default();
        self.pc = pc;
        self.vbr = vbr;
        self.r[15] = stack;
    }

    fn t(&self) -> bool {
        self.sr & T != 0
    }

    fn set_t(&mut self, value: bool) {
        if value {
            self.sr |= T
        } else {
            self.sr &= !T
        }
    }

    fn set_bit(&mut self, mask: u32, value: bool) {
        if value {
            self.sr |= mask
        } else {
            self.sr &= !mask
        }
    }

    fn mac_value(&self) -> i64 {
        ((u64::from(self.mach) << 32) | u64::from(self.macl)) as i64
    }

    fn set_mac_value(&mut self, value: i64) {
        let value = value as u64;
        self.mach = (value >> 32) as u32;
        self.macl = value as u32;
    }

    fn accumulate_long(&mut self, product: i64) {
        if self.sr & S == 0 {
            self.set_mac_value(self.mac_value().wrapping_add(product));
            return;
        }

        let raw = (u64::from(self.mach & 0xffff) << 32) | u64::from(self.macl);
        let accumulator = if raw & (1u64 << 47) != 0 {
            (raw | 0xffff_0000_0000_0000) as i64
        } else {
            raw as i64
        };
        let value = (i128::from(accumulator) + i128::from(product))
            .clamp(-(1i128 << 47), (1i128 << 47) - 1) as i64;
        self.set_mac_value(value);
    }

    fn accumulate_word(&mut self, product: i64) {
        if self.sr & S == 0 {
            self.set_mac_value(self.mac_value().wrapping_add(product));
            return;
        }

        let value = i64::from(self.macl as i32) + product;
        if value > i64::from(i32::MAX) {
            self.macl = i32::MAX as u32;
            self.mach |= 1;
        } else if value < i64::from(i32::MIN) {
            self.macl = i32::MIN as u32;
            self.mach |= 1;
        } else {
            self.macl = (value as i32) as u32;
        }
    }

    fn sign8(value: u8) -> u32 {
        i32::from(value as i8) as u32
    }

    fn sign12(value: u16) -> i32 {
        let raw = i32::from(value & 0x0fff);
        if raw & 0x0800 != 0 {
            raw | !0x0fff
        } else {
            raw
        }
    }

    fn branch_target8(pc_after_fetch: u32, disp: u8) -> u32 {
        let delta = i32::from(disp as i8) * 2;
        pc_after_fetch.wrapping_add(2).wrapping_add_signed(delta)
    }

    fn branch_target12(pc_after_fetch: u32, disp: u16) -> u32 {
        pc_after_fetch
            .wrapping_add(2)
            .wrapping_add_signed(Self::sign12(disp) * 2)
    }

    fn schedule_branch(&mut self, target: u32, in_delay_slot: bool) -> bool {
        if in_delay_slot {
            return false;
        }
        self.pending_branch = Some(target);
        true
    }

    fn push32<B: Sh2Bus>(&mut self, bus: &mut B, value: u32) {
        self.r[15] = self.r[15].wrapping_sub(4);
        bus.write32(self.r[15], value);
    }

    fn pop32<B: Sh2Bus>(&mut self, bus: &mut B) -> u32 {
        let value = bus.read32(self.r[15]);
        self.r[15] = self.r[15].wrapping_add(4);
        value
    }

    pub fn interrupt<B: Sh2Bus>(&mut self, bus: &mut B, level: u8, vector: u8) -> u32 {
        let current = ((self.sr & I_MASK) >> 4) as u8;
        if level <= current {
            return 0;
        }
        self.sleeping = false;
        self.pending_branch = None;
        self.push32(bus, self.sr);
        self.push32(bus, self.pc);
        self.sr = (self.sr & !I_MASK) | (u32::from(level.min(15)) << 4);
        self.pc = bus.read32(self.vbr.wrapping_add(u32::from(vector) * 4));
        self.cycles = self.cycles.wrapping_add(5);
        5
    }

    pub fn step<B: Sh2Bus>(&mut self, bus: &mut B) -> u32 {
        if self.sleeping {
            self.cycles = self.cycles.wrapping_add(1);
            return 1;
        }
        let delayed_target = self.pending_branch.take();
        let in_delay_slot = delayed_target.is_some();
        let opcode = bus.read16(self.pc);
        self.pc = self.pc.wrapping_add(2);
        let n = usize::from((opcode >> 8) & 0x0f);
        let m = usize::from((opcode >> 4) & 0x0f);
        let mut used = 1u32;
        let ok = match opcode >> 12 {
            0x0 => self.group0(bus, opcode, n, m, in_delay_slot, &mut used),
            0x1 => {
                let disp = u32::from(opcode & 0x0f) * 4;
                bus.write32(self.r[n].wrapping_add(disp), self.r[m]);
                true
            }
            0x2 => self.group2(bus, opcode, n, m, &mut used),
            0x3 => self.group3(opcode, n, m, &mut used),
            0x4 => self.group4(bus, opcode, n, m, in_delay_slot, &mut used),
            0x5 => {
                let disp = u32::from(opcode & 0x0f) * 4;
                self.r[n] = bus.read32(self.r[m].wrapping_add(disp));
                true
            }
            0x6 => self.group6(bus, opcode, n, m),
            0x7 => {
                self.r[n] = self.r[n].wrapping_add(Self::sign8(opcode as u8));
                true
            }
            0x8 => self.group8(bus, opcode, n, in_delay_slot, &mut used),
            0x9 => {
                let address = self
                    .pc
                    .wrapping_add(2)
                    .wrapping_add(u32::from(opcode as u8) * 2);
                self.r[n] = i32::from(bus.read16(address) as i16) as u32;
                true
            }
            0xa => {
                let target = Self::branch_target12(self.pc, opcode);
                used = 2;
                self.schedule_branch(target, in_delay_slot)
            }
            0xb => {
                self.pr = self.pc.wrapping_add(2);
                let target = Self::branch_target12(self.pc, opcode);
                used = 2;
                self.schedule_branch(target, in_delay_slot)
            }
            0xc => self.group_c(bus, opcode, &mut used),
            0xd => {
                let base = self.pc.wrapping_add(2) & !3;
                self.r[n] = bus.read32(base.wrapping_add(u32::from(opcode as u8) * 4));
                true
            }
            0xe => {
                self.r[n] = Self::sign8(opcode as u8);
                true
            }
            _ => false,
        };
        if !ok {
            return 0;
        }
        if let Some(target) = delayed_target {
            self.pc = target;
        }
        self.cycles = self.cycles.wrapping_add(u64::from(used));
        used
    }

    fn group0<B: Sh2Bus>(
        &mut self,
        bus: &mut B,
        opcode: u16,
        n: usize,
        m: usize,
        in_delay: bool,
        used: &mut u32,
    ) -> bool {
        if opcode == 0x0008 {
            self.set_t(false);
            return true;
        }
        if opcode == 0x0018 {
            self.set_t(true);
            return true;
        }
        if opcode == 0x0009 {
            return true;
        }
        if opcode == 0x0019 {
            self.sr &= !(M | Q | T);
            return true;
        }
        if opcode == 0x0028 {
            self.mach = 0;
            self.macl = 0;
            return true;
        }
        if opcode == 0x001b {
            self.sleeping = true;
            *used = 3;
            return true;
        }
        if opcode == 0x000b {
            *used = 2;
            return self.schedule_branch(self.pr, in_delay);
        }
        if opcode == 0x002b {
            if in_delay {
                return false;
            }
            let target = self.pop32(bus);
            self.sr = self.pop32(bus) & SR_MASK;
            *used = 4;
            return self.schedule_branch(target, false);
        }
        match opcode & 0x00ff {
            0x02 => self.r[n] = self.sr,
            0x12 => self.r[n] = self.gbr,
            0x22 => self.r[n] = self.vbr,
            0x0a => self.r[n] = self.mach,
            0x1a => self.r[n] = self.macl,
            0x2a => self.r[n] = self.pr,
            0x29 => self.r[n] = u32::from(self.t()),
            0x03 => {
                self.pr = self.pc.wrapping_add(2);
                *used = 2;
                return self
                    .schedule_branch(self.pc.wrapping_add(2).wrapping_add(self.r[n]), in_delay);
            }
            0x23 => {
                *used = 2;
                return self
                    .schedule_branch(self.pc.wrapping_add(2).wrapping_add(self.r[n]), in_delay);
            }
            _ => match opcode & 0x000f {
                0x4 => bus.write8(self.r[n].wrapping_add(self.r[0]), self.r[m] as u8),
                0x5 => bus.write16(self.r[n].wrapping_add(self.r[0]), self.r[m] as u16),
                0x6 => bus.write32(self.r[n].wrapping_add(self.r[0]), self.r[m]),
                0x7 => self.macl = self.r[n].wrapping_mul(self.r[m]),
                0xc => {
                    self.r[n] = i32::from(bus.read8(self.r[m].wrapping_add(self.r[0])) as i8) as u32
                }
                0xd => {
                    self.r[n] =
                        i32::from(bus.read16(self.r[m].wrapping_add(self.r[0])) as i16) as u32
                }
                0xe => self.r[n] = bus.read32(self.r[m].wrapping_add(self.r[0])),
                0xf => {
                    let n_value = bus.read32(self.r[n]) as i32 as i64;
                    self.r[n] = self.r[n].wrapping_add(4);
                    let m_value = bus.read32(self.r[m]) as i32 as i64;
                    self.r[m] = self.r[m].wrapping_add(4);
                    self.accumulate_long(n_value.wrapping_mul(m_value));
                    *used = 3;
                }
                _ => return false,
            },
        }
        true
    }

    fn group2<B: Sh2Bus>(
        &mut self,
        bus: &mut B,
        opcode: u16,
        n: usize,
        m: usize,
        used: &mut u32,
    ) -> bool {
        match opcode & 0x000f {
            0x0 => bus.write8(self.r[n], self.r[m] as u8),
            0x1 => bus.write16(self.r[n], self.r[m] as u16),
            0x2 => bus.write32(self.r[n], self.r[m]),
            0x4 => {
                self.r[n] = self.r[n].wrapping_sub(1);
                bus.write8(self.r[n], self.r[m] as u8);
            }
            0x5 => {
                self.r[n] = self.r[n].wrapping_sub(2);
                bus.write16(self.r[n], self.r[m] as u16);
            }
            0x6 => {
                self.r[n] = self.r[n].wrapping_sub(4);
                bus.write32(self.r[n], self.r[m]);
            }
            0x7 => {
                self.set_bit(Q, self.r[n] & 0x8000_0000 != 0);
                self.set_bit(M, self.r[m] & 0x8000_0000 != 0);
                self.set_t((self.sr & Q != 0) == (self.sr & M != 0));
            }
            0x8 => self.set_t(self.r[n] & self.r[m] == 0),
            0x9 => self.r[n] &= self.r[m],
            0xa => self.r[n] ^= self.r[m],
            0xb => self.r[n] |= self.r[m],
            0xc => {
                let x = self.r[n] ^ self.r[m];
                self.set_t((0..4).any(|shift| ((x >> (shift * 8)) & 0xff) == 0));
            }
            0xd => self.r[n] = (self.r[n] >> 16) | (self.r[m] << 16),
            0xe => self.macl = u32::from(self.r[n] as u16) * u32::from(self.r[m] as u16),
            0xf => {
                self.macl = ((self.r[n] as i16 as i32).wrapping_mul(self.r[m] as i16 as i32)) as u32
            }
            _ => return false,
        }
        *used = 1;
        true
    }

    fn group3(&mut self, opcode: u16, n: usize, m: usize, used: &mut u32) -> bool {
        match opcode & 0x000f {
            0x0 => self.set_t(self.r[n] == self.r[m]),
            0x2 => self.set_t(self.r[n] >= self.r[m]),
            0x3 => self.set_t((self.r[n] as i32) >= (self.r[m] as i32)),
            0x4 => self.div1(n, m),
            0x5 => {
                let value = u64::from(self.r[n]) * u64::from(self.r[m]);
                self.mach = (value >> 32) as u32;
                self.macl = value as u32;
                *used = 2;
            }
            0x6 => self.set_t(self.r[n] > self.r[m]),
            0x7 => self.set_t((self.r[n] as i32) > (self.r[m] as i32)),
            0x8 => self.r[n] = self.r[n].wrapping_sub(self.r[m]),
            0xa => {
                let borrow = u32::from(self.t());
                let (v1, b1) = self.r[n].overflowing_sub(self.r[m]);
                let (v2, b2) = v1.overflowing_sub(borrow);
                self.r[n] = v2;
                self.set_t(b1 || b2);
            }
            0xb => {
                let a = self.r[n] as i32;
                let b = self.r[m] as i32;
                let value = a.wrapping_sub(b);
                self.r[n] = value as u32;
                self.set_t(((a ^ b) & (a ^ value)) < 0);
            }
            0xc => self.r[n] = self.r[n].wrapping_add(self.r[m]),
            0xd => {
                let value = (self.r[n] as i32 as i64).wrapping_mul(self.r[m] as i32 as i64) as u64;
                self.mach = (value >> 32) as u32;
                self.macl = value as u32;
                *used = 2;
            }
            0xe => {
                let carry = u32::from(self.t());
                let (v1, c1) = self.r[n].overflowing_add(self.r[m]);
                let (v2, c2) = v1.overflowing_add(carry);
                self.r[n] = v2;
                self.set_t(c1 || c2);
            }
            0xf => {
                let a = self.r[n] as i32;
                let b = self.r[m] as i32;
                let value = a.wrapping_add(b);
                self.r[n] = value as u32;
                self.set_t(((a ^ value) & (b ^ value)) < 0);
            }
            _ => return false,
        }
        true
    }

    fn group4<B: Sh2Bus>(
        &mut self,
        bus: &mut B,
        opcode: u16,
        n: usize,
        m: usize,
        in_delay: bool,
        used: &mut u32,
    ) -> bool {
        if opcode & 0x000f == 0x000f {
            let n_value = bus.read16(self.r[n]) as i16 as i64;
            self.r[n] = self.r[n].wrapping_add(2);
            let m_value = bus.read16(self.r[m]) as i16 as i64;
            self.r[m] = self.r[m].wrapping_add(2);
            self.accumulate_word(n_value.wrapping_mul(m_value));
            *used = 3;
            return true;
        }
        match opcode & 0x00ff {
            0x00 | 0x20 => {
                let carry = self.r[n] & 0x8000_0000 != 0;
                self.r[n] <<= 1;
                self.set_t(carry);
            }
            0x01 => {
                let carry = self.r[n] & 1 != 0;
                self.r[n] >>= 1;
                self.set_t(carry);
            }
            0x02 => {
                self.r[n] = self.r[n].wrapping_sub(4);
                bus.write32(self.r[n], self.mach);
            }
            0x03 => {
                self.r[n] = self.r[n].wrapping_sub(4);
                bus.write32(self.r[n], self.sr);
            }
            0x04 => {
                let carry = self.r[n] & 0x8000_0000 != 0;
                self.r[n] = self.r[n].rotate_left(1);
                self.set_t(carry);
            }
            0x05 => {
                let carry = self.r[n] & 1 != 0;
                self.r[n] = self.r[n].rotate_right(1);
                self.set_t(carry);
            }
            0x06 => {
                self.mach = bus.read32(self.r[n]);
                self.r[n] = self.r[n].wrapping_add(4);
            }
            0x07 => {
                self.sr = bus.read32(self.r[n]) & SR_MASK;
                self.r[n] = self.r[n].wrapping_add(4);
            }
            0x08 => self.r[n] <<= 2,
            0x09 => self.r[n] >>= 2,
            0x0a => self.mach = self.r[n],
            0x0b => {
                self.pr = self.pc.wrapping_add(2);
                *used = 2;
                return self.schedule_branch(self.r[n], in_delay);
            }
            0x0e => self.sr = self.r[n] & SR_MASK,
            0x10 => {
                self.r[n] = self.r[n].wrapping_sub(1);
                self.set_t(self.r[n] == 0);
            }
            0x11 => self.set_t((self.r[n] as i32) >= 0),
            0x12 => {
                self.r[n] = self.r[n].wrapping_sub(4);
                bus.write32(self.r[n], self.macl);
            }
            0x13 => {
                self.r[n] = self.r[n].wrapping_sub(4);
                bus.write32(self.r[n], self.gbr);
            }
            0x15 => self.set_t((self.r[n] as i32) > 0),
            0x16 => {
                self.macl = bus.read32(self.r[n]);
                self.r[n] = self.r[n].wrapping_add(4);
            }
            0x17 => {
                self.gbr = bus.read32(self.r[n]);
                self.r[n] = self.r[n].wrapping_add(4);
            }
            0x18 => self.r[n] <<= 8,
            0x19 => self.r[n] >>= 8,
            0x1a => self.macl = self.r[n],
            0x1b => {
                let value = bus.read8(self.r[n]);
                self.set_t(value == 0);
                bus.write8(self.r[n], value | 0x80);
                *used = 4;
            }
            0x1e => self.gbr = self.r[n],
            0x21 => {
                let carry = self.r[n] & 1 != 0;
                self.r[n] = ((self.r[n] as i32) >> 1) as u32;
                self.set_t(carry);
            }
            0x22 => {
                self.r[n] = self.r[n].wrapping_sub(4);
                bus.write32(self.r[n], self.pr);
            }
            0x23 => {
                self.r[n] = self.r[n].wrapping_sub(4);
                bus.write32(self.r[n], self.vbr);
            }
            0x24 => {
                let old_t = u32::from(self.t());
                let carry = self.r[n] & 0x8000_0000 != 0;
                self.r[n] = (self.r[n] << 1) | old_t;
                self.set_t(carry);
            }
            0x25 => {
                let old_t = u32::from(self.t());
                let carry = self.r[n] & 1 != 0;
                self.r[n] = (self.r[n] >> 1) | (old_t << 31);
                self.set_t(carry);
            }
            0x26 => {
                self.pr = bus.read32(self.r[n]);
                self.r[n] = self.r[n].wrapping_add(4);
            }
            0x27 => {
                self.vbr = bus.read32(self.r[n]);
                self.r[n] = self.r[n].wrapping_add(4);
            }
            0x28 => self.r[n] <<= 16,
            0x29 => self.r[n] >>= 16,
            0x2a => self.pr = self.r[n],
            0x2b => {
                *used = 2;
                return self.schedule_branch(self.r[n], in_delay);
            }
            0x2e => self.vbr = self.r[n],
            _ => return false,
        }
        true
    }

    fn group6<B: Sh2Bus>(&mut self, bus: &mut B, opcode: u16, n: usize, m: usize) -> bool {
        match opcode & 0x000f {
            0x0 => self.r[n] = i32::from(bus.read8(self.r[m]) as i8) as u32,
            0x1 => self.r[n] = i32::from(bus.read16(self.r[m]) as i16) as u32,
            0x2 => self.r[n] = bus.read32(self.r[m]),
            0x3 => self.r[n] = self.r[m],
            0x4 => {
                let value = i32::from(bus.read8(self.r[m]) as i8) as u32;
                if n != m {
                    self.r[m] = self.r[m].wrapping_add(1);
                }
                self.r[n] = value;
            }
            0x5 => {
                let value = i32::from(bus.read16(self.r[m]) as i16) as u32;
                if n != m {
                    self.r[m] = self.r[m].wrapping_add(2);
                }
                self.r[n] = value;
            }
            0x6 => {
                let value = bus.read32(self.r[m]);
                if n != m {
                    self.r[m] = self.r[m].wrapping_add(4);
                }
                self.r[n] = value;
            }
            0x7 => self.r[n] = !self.r[m],
            0x8 => {
                self.r[n] = (self.r[m] & 0xffff_0000)
                    | ((self.r[m] & 0xff) << 8)
                    | ((self.r[m] >> 8) & 0xff)
            }
            0x9 => self.r[n] = self.r[m].rotate_left(16),
            0xa => {
                let carry = u32::from(self.t());
                let (v1, b1) = 0u32.overflowing_sub(self.r[m]);
                let (v2, b2) = v1.overflowing_sub(carry);
                self.r[n] = v2;
                self.set_t(b1 || b2);
            }
            0xb => self.r[n] = 0u32.wrapping_sub(self.r[m]),
            0xc => self.r[n] = self.r[m] & 0xff,
            0xd => self.r[n] = self.r[m] & 0xffff,
            0xe => self.r[n] = i32::from(self.r[m] as u8 as i8) as u32,
            0xf => self.r[n] = i32::from(self.r[m] as u16 as i16) as u32,
            _ => return false,
        }
        true
    }

    fn group8<B: Sh2Bus>(
        &mut self,
        bus: &mut B,
        opcode: u16,
        _n: usize,
        in_delay: bool,
        used: &mut u32,
    ) -> bool {
        let reg = usize::from((opcode >> 4) & 0x0f);
        let disp4 = u32::from(opcode & 0x0f);
        match opcode >> 8 {
            0x80 => bus.write8(self.r[reg].wrapping_add(disp4), self.r[0] as u8),
            0x81 => bus.write16(self.r[reg].wrapping_add(disp4 * 2), self.r[0] as u16),
            0x84 => self.r[0] = i32::from(bus.read8(self.r[reg].wrapping_add(disp4)) as i8) as u32,
            0x85 => {
                self.r[0] = i32::from(bus.read16(self.r[reg].wrapping_add(disp4 * 2)) as i16) as u32
            }
            0x88 => self.set_t(self.r[0] == Self::sign8(opcode as u8)),
            0x89 => {
                if self.t() {
                    self.pc = Self::branch_target8(self.pc, opcode as u8);
                    *used = 3;
                }
            }
            0x8b => {
                if !self.t() {
                    self.pc = Self::branch_target8(self.pc, opcode as u8);
                    *used = 3;
                }
            }
            0x8d => {
                if self.t() {
                    *used = 2;
                    return self
                        .schedule_branch(Self::branch_target8(self.pc, opcode as u8), in_delay);
                }
            }
            0x8f => {
                if !self.t() {
                    *used = 2;
                    return self
                        .schedule_branch(Self::branch_target8(self.pc, opcode as u8), in_delay);
                }
            }
            _ => return false,
        }
        true
    }

    fn group_c<B: Sh2Bus>(&mut self, bus: &mut B, opcode: u16, used: &mut u32) -> bool {
        let imm = opcode as u8;
        let disp = u32::from(imm);
        match opcode >> 8 {
            0xc0 => bus.write8(self.gbr.wrapping_add(disp), self.r[0] as u8),
            0xc1 => bus.write16(self.gbr.wrapping_add(disp * 2), self.r[0] as u16),
            0xc2 => bus.write32(self.gbr.wrapping_add(disp * 4), self.r[0]),
            0xc3 => {
                self.push32(bus, self.sr);
                self.push32(bus, self.pc);
                self.pc = bus.read32(self.vbr.wrapping_add(disp * 4));
                self.pending_branch = None;
                *used = 8;
            }
            0xc4 => self.r[0] = i32::from(bus.read8(self.gbr.wrapping_add(disp)) as i8) as u32,
            0xc5 => {
                self.r[0] = i32::from(bus.read16(self.gbr.wrapping_add(disp * 2)) as i16) as u32
            }
            0xc6 => self.r[0] = bus.read32(self.gbr.wrapping_add(disp * 4)),
            0xc7 => self.r[0] = (self.pc.wrapping_add(2) & !3).wrapping_add(disp * 4),
            0xc8 => self.set_t(self.r[0] & u32::from(imm) == 0),
            0xc9 => self.r[0] &= u32::from(imm),
            0xca => self.r[0] ^= u32::from(imm),
            0xcb => self.r[0] |= u32::from(imm),
            0xcc..=0xcf => {
                let address = self.gbr.wrapping_add(self.r[0]);
                let value = bus.read8(address);
                match opcode >> 8 {
                    0xcc => self.set_t(value & imm == 0),
                    0xcd => bus.write8(address, value & imm),
                    0xce => bus.write8(address, value ^ imm),
                    _ => bus.write8(address, value | imm),
                }
            }
            _ => return false,
        }
        true
    }

    fn div1(&mut self, n: usize, m: usize) {
        let old_q = self.sr & Q != 0;
        let q = self.r[n] & 0x8000_0000 != 0;
        self.set_bit(Q, q);
        self.r[n] = (self.r[n] << 1) | u32::from(self.t());
        let m_bit = self.sr & M != 0;
        let before = self.r[n];
        let q_after = match (old_q, m_bit) {
            (false, false) => {
                self.r[n] = self.r[n].wrapping_sub(self.r[m]);
                let carry = self.r[n] > before;
                if !q {
                    carry
                } else {
                    !carry
                }
            }
            (false, true) => {
                self.r[n] = self.r[n].wrapping_add(self.r[m]);
                let carry = self.r[n] < before;
                if !q {
                    !carry
                } else {
                    carry
                }
            }
            (true, false) => {
                self.r[n] = self.r[n].wrapping_add(self.r[m]);
                let carry = self.r[n] < before;
                if !q {
                    carry
                } else {
                    !carry
                }
            }
            (true, true) => {
                self.r[n] = self.r[n].wrapping_sub(self.r[m]);
                let carry = self.r[n] > before;
                if !q {
                    !carry
                } else {
                    carry
                }
            }
        };
        self.set_bit(Q, q_after);
        self.set_t(q_after == m_bit);
    }

    pub fn save(&self, out: &mut StateWriter) {
        for value in self.r {
            out.u32(value);
        }
        out.u32(self.pc);
        out.u32(self.pr);
        out.u32(self.gbr);
        out.u32(self.vbr);
        out.u32(self.mach);
        out.u32(self.macl);
        out.u32(self.sr);
        out.u64(self.cycles);
        out.u8(u8::from(self.sleeping));
        out.u8(u8::from(self.pending_branch.is_some()));
        out.u32(self.pending_branch.unwrap_or(0));
    }

    pub fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        for value in &mut self.r {
            *value = input.u32()?;
        }
        self.pc = input.u32()?;
        self.pr = input.u32()?;
        self.gbr = input.u32()?;
        self.vbr = input.u32()?;
        self.mach = input.u32()?;
        self.macl = input.u32()?;
        self.sr = input.u32()? & SR_MASK;
        self.cycles = input.u64()?;
        self.sleeping = input.u8()? != 0;
        let pending = input.u8()? != 0;
        let target = input.u32()?;
        self.pending_branch = pending.then_some(target);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::platform::PlatformId;

    struct TestBus {
        data: Vec<u8>,
    }

    impl TestBus {
        fn new() -> Self {
            Self {
                data: vec![0; 0x2000],
            }
        }
        fn word(&mut self, address: usize, value: u16) {
            self.data[address..address + 2].copy_from_slice(&value.to_be_bytes());
        }
        fn long(&mut self, address: usize, value: u32) {
            self.data[address..address + 4].copy_from_slice(&value.to_be_bytes());
        }
    }

    impl Sh2Bus for TestBus {
        fn read8(&mut self, address: u32) -> u8 {
            self.data.get(address as usize).copied().unwrap_or(0xff)
        }
        fn write8(&mut self, address: u32, value: u8) {
            if let Some(slot) = self.data.get_mut(address as usize) {
                *slot = value;
            }
        }
    }

    #[test]
    fn frt_counts_compares_captures_and_routes_interrupts() {
        let mut frt = Sh2Frt::default();
        let mut intc = Sh2Intc::default();
        assert_eq!(frt.read8(0xffff_fe10), Some(0x01));
        assert!(frt.write8(0xffff_fe14, 0x00));
        assert!(frt.write8(0xffff_fe15, 0x03));
        assert!(frt.write8(0xffff_fe11, 0x01));
        assert!(frt.write8(0xffff_fe10, 0x08));
        assert!(intc.write8(0xffff_fe60, 0x05));
        assert!(intc.write8(0xffff_fe67, 0x60));

        frt.tick(24);
        assert_eq!(frt.read8(0xffff_fe12), Some(0));
        assert_eq!(frt.read8(0xffff_fe13), Some(0));
        assert_eq!(frt.read8(0xffff_fe11).unwrap() & 0x08, 0x08);
        assert_eq!(frt.interrupt(&intc), Some((5, 0x60)));

        assert!(frt.write8(0xffff_fe11, 0x01));
        assert_eq!(frt.read8(0xffff_fe11).unwrap() & 0x08, 0);
        assert_eq!(frt.interrupt(&intc), None);

        assert!(frt.write8(0xffff_fe12, 0x12));
        assert!(frt.write8(0xffff_fe13, 0x34));
        assert!(frt.write8(0xffff_fe10, 0x80));
        assert!(intc.write8(0xffff_fe60, 0x0a));
        assert!(intc.write8(0xffff_fe66, 0x64));
        frt.capture();
        assert_eq!(frt.read8(0xffff_fe18), Some(0x12));
        assert_eq!(frt.read8(0xffff_fe19), Some(0x34));
        assert_eq!(frt.interrupt(&intc), Some((10, 0x64)));

        assert!(frt.write8(0xffff_fe16, 0x03));
        frt.tick(10_000);
        assert_eq!(frt.read8(0xffff_fe12), Some(0x12));
        assert_eq!(frt.read8(0xffff_fe13), Some(0x34));
    }

    #[test]
    fn wdt_keyed_access_interval_interrupt_and_watchdog_overflow() {
        let mut wdt = Sh2Wdt::default();
        let mut intc = Sh2Intc::default();
        assert_eq!(wdt.read8(0xffff_fe80), Some(0x18));
        assert!(intc.write16(0xffff_fee2, 0x0050));
        assert!(intc.write16(0xffff_fee4, 0x6300));
        assert!(wdt.write16(0xffff_fe80, 0x5aff));
        assert!(wdt.write16(0xffff_fe80, 0xa53e));
        wdt.tick(4095);
        assert_eq!(wdt.interrupt(&intc), None);
        wdt.tick(1);
        assert_eq!(wdt.read8(0xffff_fe81), Some(0));
        assert_eq!(wdt.read8(0xffff_fe80).unwrap() & 0x80, 0x80);
        assert_eq!(wdt.interrupt(&intc), Some((5, 0x63)));
        assert!(wdt.write16(0xffff_fe80, 0xa53e));
        assert_eq!(wdt.interrupt(&intc), None);
        assert!(wdt.write16(0xffff_fe80, 0x5aff));
        assert!(wdt.write16(0xffff_fe80, 0xa57e));
        wdt.tick(4096);
        assert_eq!(wdt.read8(0xffff_fe83).unwrap() & 0x80, 0x80);
        assert_eq!(wdt.interrupt(&intc), None);
        assert!(wdt.write16(0xffff_fe82, 0xa500));
        assert_eq!(wdt.read8(0xffff_fe83).unwrap() & 0x80, 0);
        assert!(wdt.write16(0xffff_fe82, 0x5a60));
        assert_eq!(wdt.read8(0xffff_fe83), Some(0x7f));
    }

    #[test]
    fn arithmetic_memory_and_big_endian_bus_execute() {
        let mut bus = TestBus::new();
        bus.word(0, 0xe105);
        bus.word(2, 0xe207);
        bus.word(4, 0x321c);
        bus.word(6, 0x2322);
        let mut cpu = Sh2::default();
        cpu.r[3] = 0x100;
        for _ in 0..4 {
            assert_ne!(cpu.step(&mut bus), 0);
        }
        assert_eq!(cpu.r[2], 12);
        assert_eq!(&bus.data[0x100..0x104], &[0, 0, 0, 12]);
    }

    #[test]
    fn delayed_branch_executes_slot_once() {
        let mut bus = TestBus::new();
        bus.word(0, 0xe101);
        bus.word(2, 0xa001);
        bus.word(4, 0xe202);
        bus.word(6, 0xe303);
        bus.word(8, 0xe404);
        let mut cpu = Sh2::default();
        assert_ne!(cpu.step(&mut bus), 0);
        assert_ne!(cpu.step(&mut bus), 0);
        assert_eq!(cpu.pc, 4);
        assert_ne!(cpu.step(&mut bus), 0);
        assert_eq!(cpu.pc, 8);
        assert_eq!(cpu.r[2], 2);
        assert_eq!(cpu.r[3], 0);
        assert_ne!(cpu.step(&mut bus), 0);
        assert_eq!(cpu.r[4], 4);
    }

    #[test]
    fn trap_and_rte_preserve_pc_and_status() {
        let mut bus = TestBus::new();
        bus.word(0, 0xc320);
        bus.long(0x180, 0x200);
        bus.word(0x200, 0x002b);
        bus.word(0x202, 0x0009);
        let mut cpu = Sh2 {
            vbr: 0x100,
            sr: 0x51,
            ..Default::default()
        };
        cpu.r[15] = 0x800;
        assert_ne!(cpu.step(&mut bus), 0);
        assert_eq!(cpu.pc, 0x200);
        assert_eq!(cpu.r[15], 0x7f8);
        assert_eq!(bus.read32(0x7f8), 2);
        assert_eq!(bus.read32(0x7fc), 0x51);
        assert_ne!(cpu.step(&mut bus), 0);
        assert_ne!(cpu.step(&mut bus), 0);
        assert_eq!(cpu.pc, 2);
        assert_eq!(cpu.sr, 0x51);
    }

    #[test]
    fn mac_postincrement_alias_reads_consecutive_operands() {
        let mut bus = TestBus::new();
        bus.word(0, 0x011f);
        bus.word(2, 0x422f);
        bus.long(0x100, 3);
        bus.long(0x104, 4);
        bus.word(0x120, 5);
        bus.word(0x122, 6);
        let mut cpu = Sh2::default();
        cpu.r[1] = 0x100;
        cpu.r[2] = 0x120;

        assert_ne!(cpu.step(&mut bus), 0);
        assert_eq!(cpu.r[1], 0x108);
        assert_eq!(cpu.mach, 0);
        assert_eq!(cpu.macl, 12);

        cpu.mach = 0;
        cpu.macl = 0;
        assert_ne!(cpu.step(&mut bus), 0);
        assert_eq!(cpu.r[2], 0x124);
        assert_eq!(cpu.mach, 0);
        assert_eq!(cpu.macl, 30);
    }

    #[test]
    fn mac_saturation_obeys_sr_s_for_long_and_word_operations() {
        let mut bus = TestBus::new();
        bus.word(0, 0x032f);
        bus.long(0x100, 2);
        bus.long(0x104, 16);
        let mut cpu = Sh2 {
            sr: S,
            mach: 0x0000_7fff,
            macl: 0xffff_fff0,
            ..Default::default()
        };
        cpu.r[3] = 0x100;
        cpu.r[2] = 0x104;
        assert_ne!(cpu.step(&mut bus), 0);
        assert_eq!(cpu.mach, 0x0000_7fff);
        assert_eq!(cpu.macl, 0xffff_ffff);

        let mut bus = TestBus::new();
        bus.word(0, 0x032f);
        bus.long(0x100, (-2i32) as u32);
        bus.long(0x104, 16);
        let mut cpu = Sh2 {
            sr: S,
            mach: 0xffff_8000,
            macl: 0x0000_0010,
            ..Default::default()
        };
        cpu.r[3] = 0x100;
        cpu.r[2] = 0x104;
        assert_ne!(cpu.step(&mut bus), 0);
        assert_eq!(cpu.mach, 0xffff_8000);
        assert_eq!(cpu.macl, 0x0000_0000);

        let mut bus = TestBus::new();
        bus.word(0, 0x432f);
        bus.word(0x100, 4);
        bus.word(0x102, 8);
        let mut cpu = Sh2 {
            sr: S,
            mach: 0x1234_5678,
            macl: 0x7fff_fff0,
            ..Default::default()
        };
        cpu.r[3] = 0x100;
        cpu.r[2] = 0x102;
        assert_ne!(cpu.step(&mut bus), 0);
        assert_eq!(cpu.macl, 0x7fff_ffff);
        assert_eq!(cpu.mach, 0x1234_5679);

        cpu.macl = 0;
        cpu.pc = 0;
        bus.word(0x100, 1);
        bus.word(0x102, 1);
        cpu.r[3] = 0x100;
        cpu.r[2] = 0x102;
        assert_ne!(cpu.step(&mut bus), 0);
        assert_eq!(cpu.macl, 1);
        assert_eq!(cpu.mach, 0x1234_5679);
    }

    #[test]
    fn cpu_state_round_trip_is_deterministic() {
        let mut cpu = Sh2::default();
        cpu.r[2] = 0x1234_5678;
        cpu.pc = 0x0600_0100;
        cpu.vbr = 0x0600_0000;
        cpu.pr = 0x0600_0200;
        cpu.sr = 0x121;
        cpu.cycles = 99;
        cpu.pending_branch = Some(0x0600_0300);
        let mut out = StateWriter::new(PlatformId::Sega32x, 1);
        cpu.save(&mut out);
        let bytes = out.finish();
        let mut input = StateReader::new(&bytes, PlatformId::Sega32x, 1).unwrap();
        let mut restored = Sh2::default();
        restored.load(&mut input).unwrap();
        input.finish().unwrap();
        assert_eq!(restored.r[2], cpu.r[2]);
        assert_eq!(restored.pc, cpu.pc);
        assert_eq!(restored.vbr, cpu.vbr);
        assert_eq!(restored.pr, cpu.pr);
        assert_eq!(restored.sr, cpu.sr);
        assert_eq!(restored.cycles, cpu.cycles);
        assert_eq!(restored.pending_branch, cpu.pending_branch);
    }
}

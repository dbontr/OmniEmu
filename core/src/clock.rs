#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ClockRate {
    pub numerator: u64,
    pub denominator: u64,
}

impl ClockRate {
    pub const fn hz(hz: u64) -> Self {
        Self {
            numerator: hz,
            denominator: 1,
        }
    }
    pub const fn rational(numerator: u64, denominator: u64) -> Self {
        Self {
            numerator,
            denominator,
        }
    }
}

#[derive(Debug, Clone)]
pub struct ClockDomain {
    ratio_num: u128,
    ratio_den: u128,
    phase: u128,
    cycles: u64,
}

impl ClockDomain {
    pub fn new(master: ClockRate, device: ClockRate) -> Self {
        assert!(master.numerator != 0 && master.denominator != 0);
        assert!(device.numerator != 0 && device.denominator != 0);
        let numerator = device.numerator as u128 * master.denominator as u128;
        let denominator = device.denominator as u128 * master.numerator as u128;
        let divisor = gcd_u128(numerator, denominator);
        Self {
            ratio_num: numerator / divisor,
            ratio_den: denominator / divisor,
            phase: 0,
            cycles: 0,
        }
    }
    pub fn advance(&mut self, master_ticks: u64) -> u64 {
        self.phase += master_ticks as u128 * self.ratio_num;
        let produced = self.phase / self.ratio_den;
        self.phase %= self.ratio_den;
        let produced = produced.min(u64::MAX as u128) as u64;
        self.cycles = self.cycles.saturating_add(produced);
        produced
    }

    pub fn cycles(&self) -> u64 {
        self.cycles
    }
    pub fn phase(&self) -> (u128, u128) {
        (self.phase, self.ratio_den)
    }
}

fn gcd_u128(mut a: u128, mut b: u128) -> u128 {
    while b != 0 {
        let remainder = a % b;
        a = b;
        b = remainder;
    }
    a.max(1)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exact_ratio_accumulates_without_float_drift() {
        let mut ppu = ClockDomain::new(ClockRate::hz(1_789_773), ClockRate::hz(5_369_319));
        assert_eq!(ppu.advance(1000), 3000);
        assert_eq!(ppu.advance(1), 3);
        assert_eq!(ppu.cycles(), 3003);
    }

    #[test]
    fn fractional_domains_preserve_remainder_across_calls() {
        let mut half = ClockDomain::new(ClockRate::hz(2), ClockRate::hz(1));
        assert_eq!(half.advance(1), 0);
        assert_eq!(half.advance(1), 1);
        assert_eq!(half.advance(3), 1);
        assert_eq!(half.advance(1), 1);
        assert_eq!(half.cycles(), 3);
    }
}

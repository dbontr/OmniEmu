use crate::state::{StateReader, StateWriter};

const RTC_BYTES: usize = 20;
const CPU_HZ: u64 = 3_579_545;
const MONTH_DAYS: [u16; 12] = [31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Mode {
    Read,
    Command,
    Write,
    Ready,
}

pub(crate) struct Srtc {
    regs: [u8; RTC_BYTES],
    mode: Mode,
    index: i8,
    cycle_phase: u64,
}

impl Default for Srtc {
    fn default() -> Self {
        let mut regs = [0; RTC_BYTES];
        regs[6] = 1;
        regs[8] = 1;
        regs[9] = 0;
        regs[10] = 0;
        regs[11] = 9;
        regs[12] = 1;
        Self {
            regs,
            mode: Mode::Read,
            index: -1,
            cycle_phase: 0,
        }
    }
}

impl Srtc {
    pub(crate) fn reset(&mut self) {
        self.mode = Mode::Read;
        self.index = -1;
    }

    fn leap_year(year: u16) -> bool {
        year.is_multiple_of(400) || (year.is_multiple_of(4) && !year.is_multiple_of(100))
    }

    fn days_in_month(year: u16, month: u16) -> u16 {
        if month == 2 && Self::leap_year(year) {
            29
        } else {
            MONTH_DAYS[usize::from(month.clamp(1, 12) - 1)]
        }
    }

    fn weekday(year: u16, month: u16, day: u16) -> u8 {
        let year = year.max(1900);
        let month = month.clamp(1, 12);
        let mut days = 0u32;
        for y in 1900..year {
            days += if Self::leap_year(y) { 366 } else { 365 };
        }
        for m in 1..month {
            days += u32::from(Self::days_in_month(year, m));
        }
        days += u32::from(day.clamp(1, Self::days_in_month(year, month)) - 1);
        ((days + 1) % 7) as u8
    }

    fn calendar(&self) -> (u16, u16, u16, u16, u16, u16) {
        let second = u16::from(self.regs[0]) + u16::from(self.regs[1]) * 10;
        let minute = u16::from(self.regs[2]) + u16::from(self.regs[3]) * 10;
        let hour = u16::from(self.regs[4]) + u16::from(self.regs[5]) * 10;
        let day = u16::from(self.regs[6]) + u16::from(self.regs[7]) * 10;
        let month = u16::from(self.regs[8]);
        let year = 1000
            + u16::from(self.regs[9])
            + u16::from(self.regs[10]) * 10
            + u16::from(self.regs[11]) * 100;
        (second, minute, hour, day, month, year)
    }

    fn store_calendar(
        &mut self,
        second: u16,
        minute: u16,
        hour: u16,
        day: u16,
        month: u16,
        year: u16,
    ) {
        let encoded_year = year.saturating_sub(1000);
        self.regs[0] = (second % 10) as u8;
        self.regs[1] = (second / 10) as u8;
        self.regs[2] = (minute % 10) as u8;
        self.regs[3] = (minute / 10) as u8;
        self.regs[4] = (hour % 10) as u8;
        self.regs[5] = (hour / 10) as u8;
        self.regs[6] = (day % 10) as u8;
        self.regs[7] = (day / 10) as u8;
        self.regs[8] = month as u8;
        self.regs[9] = (encoded_year % 10) as u8;
        self.regs[10] = ((encoded_year / 10) % 10) as u8;
        self.regs[11] = (encoded_year / 100) as u8;
        self.regs[12] = Self::weekday(year, month, day);
    }

    fn advance_second(&mut self) {
        let (mut second, mut minute, mut hour, mut day, mut month, mut year) = self.calendar();
        second = second.min(59) + 1;
        minute = minute.min(59);
        hour = hour.min(23);
        month = month.clamp(1, 12);
        day = day.clamp(1, Self::days_in_month(year, month));

        if second == 60 {
            second = 0;
            minute += 1;
            if minute == 60 {
                minute = 0;
                hour += 1;
                if hour == 24 {
                    hour = 0;
                    day += 1;
                    if day > Self::days_in_month(year, month) {
                        day = 1;
                        month += 1;
                        if month == 13 {
                            month = 1;
                            year = year.wrapping_add(1);
                        }
                    }
                }
            }
        }
        self.store_calendar(second, minute, hour, day, month, year);
    }

    pub(crate) fn tick(&mut self, cycles: u32) {
        self.cycle_phase = self.cycle_phase.wrapping_add(u64::from(cycles));
        while self.cycle_phase >= CPU_HZ {
            self.cycle_phase -= CPU_HZ;
            self.advance_second();
        }
    }

    pub(crate) fn read(&mut self) -> u8 {
        if self.mode != Mode::Read {
            return 0;
        }
        if self.index < 0 {
            self.index = 0;
            return 0x0f;
        }
        if self.index > 12 {
            self.index = -1;
            return 0x0f;
        }
        let value = self.regs[self.index as usize];
        self.index += 1;
        value
    }

    pub(crate) fn write(&mut self, value: u8) {
        let value = value & 0x0f;
        match value {
            0x0d => {
                self.mode = Mode::Read;
                self.index = -1;
                return;
            }
            0x0e => {
                self.mode = Mode::Command;
                return;
            }
            0x0f => return,
            _ => {}
        }

        match self.mode {
            Mode::Write if (0..12).contains(&self.index) => {
                self.regs[self.index as usize] = value;
                self.index += 1;
                if self.index == 12 {
                    let (_, _, _, day, month, year) = self.calendar();
                    self.regs[12] = Self::weekday(year, month, day);
                    self.index = 13;
                }
            }
            Mode::Command if value == 0 => {
                self.mode = Mode::Write;
                self.index = 0;
            }
            Mode::Command if value == 4 => {
                self.mode = Mode::Ready;
                self.index = -1;
                self.regs[..13].fill(0);
            }
            Mode::Command => {
                self.mode = Mode::Ready;
                self.index = -1;
            }
            _ => {}
        }
    }

    pub(crate) fn persistent(&self) -> &[u8] {
        &self.regs
    }
    pub(crate) fn set_persistent(&mut self, data: &[u8]) -> Result<(), String> {
        if data.len() != self.regs.len() {
            return Err("SNES S-RTC persistent data must contain 20 bytes".into());
        }
        self.regs.copy_from_slice(data);
        Ok(())
    }

    pub(crate) fn save(&self, out: &mut StateWriter) {
        out.blob(&self.regs);
        out.u8(match self.mode {
            Mode::Read => 0,
            Mode::Command => 1,
            Mode::Write => 2,
            Mode::Ready => 3,
        });
        out.u8((self.index + 1) as u8);
        out.u64(self.cycle_phase);
    }

    pub(crate) fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        let regs = input.blob()?;
        if regs.len() != self.regs.len() {
            return Err("invalid SNES S-RTC register state length".into());
        }
        self.regs.copy_from_slice(regs);
        self.mode = match input.u8()? {
            0 => Mode::Read,
            1 => Mode::Command,
            2 => Mode::Write,
            3 => Mode::Ready,
            _ => return Err("invalid SNES S-RTC mode".into()),
        };
        let index = input.u8()?;
        if index > 14 {
            return Err("invalid SNES S-RTC index".into());
        }
        self.index = index as i8 - 1;
        self.cycle_phase = input.u64()? % CPU_HZ;
        Ok(())
    }
}

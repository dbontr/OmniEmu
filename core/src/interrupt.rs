const WORDS: usize = 4;
const LINES: usize = WORDS * 64;

#[derive(Debug, Clone)]
pub struct InterruptController {
    pending: [u64; WORDS],
    enabled: [u64; WORDS],
    priorities: [u8; LINES],
}

impl Default for InterruptController {
    fn default() -> Self {
        Self {
            pending: [0; WORDS],
            enabled: [u64::MAX; WORDS],
            priorities: [0; LINES],
        }
    }
}

impl InterruptController {
    pub const fn line_count() -> usize {
        LINES
    }

    pub fn set_enabled(&mut self, line: usize, enabled: bool) -> Result<(), String> {
        let (word, bit) = Self::index(line)?;
        if enabled {
            self.enabled[word] |= bit;
        } else {
            self.enabled[word] &= !bit;
        }
        Ok(())
    }

    pub fn set_priority(&mut self, line: usize, priority: u8) -> Result<(), String> {
        if line >= LINES {
            return Err("interrupt line is out of range".into());
        }
        self.priorities[line] = priority;
        Ok(())
    }
    pub fn raise(&mut self, line: usize) -> Result<(), String> {
        let (word, bit) = Self::index(line)?;
        self.pending[word] |= bit;
        Ok(())
    }

    pub fn clear(&mut self, line: usize) -> Result<(), String> {
        let (word, bit) = Self::index(line)?;
        self.pending[word] &= !bit;
        Ok(())
    }

    pub fn is_pending(&self, line: usize) -> Result<bool, String> {
        let (word, bit) = Self::index(line)?;
        Ok(self.pending[word] & self.enabled[word] & bit != 0)
    }

    pub fn next(&self) -> Option<usize> {
        let mut best: Option<(u8, usize)> = None;
        for line in 0..LINES {
            let word = line / 64;
            let bit = 1u64 << (line % 64);
            if self.pending[word] & self.enabled[word] & bit == 0 {
                continue;
            }
            let candidate = (self.priorities[line], line);
            if best.is_none_or(|current| candidate > current) {
                best = Some(candidate);
            }
        }
        best.map(|(_, line)| line)
    }

    fn index(line: usize) -> Result<(usize, u64), String> {
        if line >= LINES {
            return Err("interrupt line is out of range".into());
        }
        Ok((line / 64, 1u64 << (line % 64)))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn prioritizes_enabled_pending_lines() {
        let mut irq = InterruptController::default();
        irq.set_priority(3, 1).unwrap();
        irq.set_priority(130, 9).unwrap();
        irq.raise(3).unwrap();
        irq.raise(130).unwrap();
        assert_eq!(irq.next(), Some(130));
        irq.set_enabled(130, false).unwrap();
        assert_eq!(irq.next(), Some(3));
        irq.clear(3).unwrap();
        assert_eq!(irq.next(), None);
    }

    #[test]
    fn supports_256_interrupt_sources() {
        let mut irq = InterruptController::default();
        irq.raise(255).unwrap();
        assert!(irq.is_pending(255).unwrap());
        assert!(irq.raise(256).is_err());
    }
}

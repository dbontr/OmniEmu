pub trait Bus8 {
    fn read8(&mut self, address: u16) -> u8;
    fn write8(&mut self, address: u16, value: u8);
    fn read16(&mut self, address: u16) -> u16 {
        let lo = self.read8(address) as u16;
        let hi = self.read8(address.wrapping_add(1)) as u16;
        lo | (hi << 8)
    }
}

pub struct Ram64k {
    bytes: Box<[u8; 65536]>,
}

impl Ram64k {
    pub fn new() -> Self {
        Self {
            bytes: Box::new([0; 65536]),
        }
    }
    pub fn load(&mut self, start: u16, data: &[u8]) {
        let start = start as usize;
        let end = start.saturating_add(data.len()).min(self.bytes.len());
        self.bytes[start..end].copy_from_slice(&data[..end - start]);
    }
    pub fn bytes(&self) -> &[u8; 65536] {
        &self.bytes
    }
    pub fn bytes_mut(&mut self) -> &mut [u8; 65536] {
        &mut self.bytes
    }
}

impl Default for Ram64k {
    fn default() -> Self {
        Self::new()
    }
}
impl Bus8 for Ram64k {
    fn read8(&mut self, address: u16) -> u8 {
        self.bytes[address as usize]
    }
    fn write8(&mut self, address: u16, value: u8) {
        self.bytes[address as usize] = value;
    }
}

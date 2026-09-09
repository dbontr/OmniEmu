use crate::platform::PlatformId;

const MAGIC: &[u8; 8] = b"OMNISTAT";
const FORMAT_VERSION: u32 = 1;
const HEADER_LEN: usize = 20;

pub struct StateWriter {
    bytes: Vec<u8>,
}

impl StateWriter {
    pub fn new(platform: PlatformId, machine_version: u32) -> Self {
        let mut bytes = Vec::with_capacity(256);
        bytes.extend_from_slice(MAGIC);
        bytes.extend_from_slice(&FORMAT_VERSION.to_le_bytes());
        bytes.extend_from_slice(&(platform as u32).to_le_bytes());
        bytes.extend_from_slice(&machine_version.to_le_bytes());
        Self { bytes }
    }

    pub fn u8(&mut self, value: u8) {
        self.bytes.push(value);
    }
    pub fn u16(&mut self, value: u16) {
        self.bytes.extend_from_slice(&value.to_le_bytes());
    }
    pub fn u32(&mut self, value: u32) {
        self.bytes.extend_from_slice(&value.to_le_bytes());
    }
    pub fn u64(&mut self, value: u64) {
        self.bytes.extend_from_slice(&value.to_le_bytes());
    }
    pub fn f32(&mut self, value: f32) {
        self.u32(value.to_bits());
    }
    pub fn f64(&mut self, value: f64) {
        self.u64(value.to_bits());
    }

    pub fn blob(&mut self, value: &[u8]) {
        self.u32(value.len() as u32);
        self.bytes.extend_from_slice(value);
    }

    pub fn finish(self) -> Vec<u8> {
        self.bytes
    }
}

pub struct StateReader<'a> {
    bytes: &'a [u8],
    cursor: usize,
}

impl<'a> StateReader<'a> {
    pub fn new(
        bytes: &'a [u8],
        platform: PlatformId,
        machine_version: u32,
    ) -> Result<Self, String> {
        if bytes.len() < HEADER_LEN {
            return Err("save state is truncated".into());
        }
        if &bytes[..8] != MAGIC {
            return Err("save state magic does not match OmniCore".into());
        }
        let format = u32::from_le_bytes(bytes[8..12].try_into().unwrap());
        if format != FORMAT_VERSION {
            return Err(format!("unsupported save-state format {format}"));
        }
        let stored_platform = u32::from_le_bytes(bytes[12..16].try_into().unwrap());
        if stored_platform != platform as u32 {
            return Err("save state belongs to another console".into());
        }
        let stored_machine = u32::from_le_bytes(bytes[16..20].try_into().unwrap());
        if stored_machine != machine_version {
            return Err(format!(
                "unsupported machine-state version {stored_machine}"
            ));
        }
        Ok(Self {
            bytes,
            cursor: HEADER_LEN,
        })
    }

    fn take<const N: usize>(&mut self) -> Result<[u8; N], String> {
        let end = self
            .cursor
            .checked_add(N)
            .ok_or_else(|| "save-state cursor overflow".to_string())?;
        if end > self.bytes.len() {
            return Err("save state is truncated".into());
        }
        let value = self.bytes[self.cursor..end].try_into().unwrap();
        self.cursor = end;
        Ok(value)
    }
    pub fn u8(&mut self) -> Result<u8, String> {
        Ok(self.take::<1>()?[0])
    }
    pub fn u16(&mut self) -> Result<u16, String> {
        Ok(u16::from_le_bytes(self.take()?))
    }
    pub fn u32(&mut self) -> Result<u32, String> {
        Ok(u32::from_le_bytes(self.take()?))
    }
    pub fn u64(&mut self) -> Result<u64, String> {
        Ok(u64::from_le_bytes(self.take()?))
    }
    pub fn f32(&mut self) -> Result<f32, String> {
        Ok(f32::from_bits(self.u32()?))
    }
    pub fn f64(&mut self) -> Result<f64, String> {
        Ok(f64::from_bits(self.u64()?))
    }

    pub fn blob(&mut self) -> Result<&'a [u8], String> {
        let len = self.u32()? as usize;
        let end = self
            .cursor
            .checked_add(len)
            .ok_or_else(|| "save-state blob overflow".to_string())?;
        if end > self.bytes.len() {
            return Err("save-state blob is truncated".into());
        }
        let value = &self.bytes[self.cursor..end];
        self.cursor = end;
        Ok(value)
    }

    pub fn finish(self) -> Result<(), String> {
        if self.cursor == self.bytes.len() {
            Ok(())
        } else {
            Err("save state has trailing bytes".into())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn state_codec_is_versioned_and_endian_stable() {
        let mut writer = StateWriter::new(PlatformId::HomePong, 7);
        writer.u16(0x1234);
        writer.f32(1.25);
        writer.blob(&[1, 2, 3]);
        let bytes = writer.finish();
        assert_eq!(&bytes[..8], b"OMNISTAT");
        assert_eq!(&bytes[20..22], &[0x34, 0x12]);
        let mut reader = StateReader::new(&bytes, PlatformId::HomePong, 7).unwrap();
        assert_eq!(reader.u16().unwrap(), 0x1234);
        assert_eq!(reader.f32().unwrap(), 1.25);
        assert_eq!(reader.blob().unwrap(), &[1, 2, 3]);
        reader.finish().unwrap();
    }

    #[test]
    fn state_codec_rejects_cross_console_restore() {
        let bytes = StateWriter::new(PlatformId::HomePong, 1).finish();
        assert!(StateReader::new(&bytes, PlatformId::Nes, 1).is_err());
    }
}

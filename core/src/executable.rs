use crate::resources::ResourceBlob;
use crate::sparse_memory::SparseMemory;

pub const EM_PPC: u16 = 20;
pub const EM_PPC64: u16 = 21;
pub const EM_AARCH64: u16 = 183;
const PT_LOAD: u32 = 1;

fn be16(bytes: &[u8], offset: usize) -> Result<u16, String> {
    let value = bytes
        .get(offset..offset + 2)
        .ok_or_else(|| "ELF field is truncated".to_string())?;
    Ok(u16::from_be_bytes(value.try_into().unwrap()))
}
fn be32(bytes: &[u8], offset: usize) -> Result<u32, String> {
    let value = bytes
        .get(offset..offset + 4)
        .ok_or_else(|| "ELF field is truncated".to_string())?;
    Ok(u32::from_be_bytes(value.try_into().unwrap()))
}
fn be64(bytes: &[u8], offset: usize) -> Result<u64, String> {
    let value = bytes
        .get(offset..offset + 8)
        .ok_or_else(|| "ELF field is truncated".to_string())?;
    Ok(u64::from_be_bytes(value.try_into().unwrap()))
}
fn validate_ident(header: &[u8], class: u8, endian: u8, machine: u16) -> Result<(), String> {
    if header.get(..4) != Some(b"\x7fELF") {
        return Err("executable is not an ELF image".into());
    }
    if header[4] != class || header[5] != endian {
        return Err("ELF class or byte order is unsupported".into());
    }
    if be16(header, 18)? != machine {
        return Err("ELF machine does not match the selected platform".into());
    }
    Ok(())
}
fn load_segment<F>(
    image: &ResourceBlob,
    memory: &mut SparseMemory,
    file_offset: u64,
    virtual_address: u64,
    file_size: u64,
    memory_size: u64,
    map: F,
) -> Result<(), String>
where
    F: Fn(u64) -> Option<u64>,
{
    if file_size > memory_size {
        return Err("ELF load segment has filesz larger than memsz".into());
    }
    let image_end = file_offset
        .checked_add(file_size)
        .filter(|end| *end <= image.len())
        .ok_or_else(|| "ELF load segment exceeds the image".to_string())?;
    let _ = image_end;
    let memory_address = map(virtual_address)
        .ok_or_else(|| format!("ELF virtual address {virtual_address:#x} is not mapped"))?;
    if memory_size != 0 {
        let guest_last = virtual_address
            .checked_add(memory_size - 1)
            .ok_or_else(|| "ELF virtual range overflow".to_string())?;
        let mapped_last = map(guest_last)
            .ok_or_else(|| "ELF load segment crosses an unmapped guest range".to_string())?;
        if mapped_last != memory_address + memory_size - 1 {
            return Err("ELF load segment crosses a non-contiguous guest mapping".into());
        }
    }
    memory_address
        .checked_add(memory_size)
        .filter(|end| *end <= memory.len())
        .ok_or_else(|| "ELF load segment exceeds guest memory".to_string())?;
    let file_len =
        usize::try_from(file_size).map_err(|_| "ELF segment is too large".to_string())?;
    memory.zero(memory_address, memory_size)?;
    if file_len != 0 {
        let mut bytes = vec![0; file_len];
        image.read(file_offset, &mut bytes)?;
        memory.write(memory_address, &bytes)?;
    }
    Ok(())
}
pub fn load_elf32_be<F>(
    image: &ResourceBlob,
    memory: &mut SparseMemory,
    expected_machine: u16,
    map: F,
) -> Result<u64, String>
where
    F: Fn(u64) -> Option<u64> + Copy,
{
    let mut header = [0; 52];
    image.read(0, &mut header)?;
    validate_ident(&header, 1, 2, expected_machine)?;
    let entry = u64::from(be32(&header, 24)?);
    let phoff = u64::from(be32(&header, 28)?);
    let phentsize = usize::from(be16(&header, 42)?);
    let phnum = usize::from(be16(&header, 44)?);
    if phentsize < 32 || phnum > 4096 {
        return Err("ELF32 program header table is invalid".into());
    }
    for index in 0..phnum {
        let offset = phoff
            .checked_add((index * phentsize) as u64)
            .ok_or_else(|| "ELF32 program header offset overflow".to_string())?;
        let mut ph = vec![0; phentsize];
        image.read(offset, &mut ph)?;
        if be32(&ph, 0)? != PT_LOAD {
            continue;
        }
        load_segment(
            image,
            memory,
            u64::from(be32(&ph, 4)?),
            u64::from(be32(&ph, 8)?),
            u64::from(be32(&ph, 16)?),
            u64::from(be32(&ph, 20)?),
            map,
        )?;
    }
    map(entry).ok_or_else(|| "ELF32 entry point is not mapped".to_string())?;
    Ok(entry)
}

pub fn load_elf64_be<F>(
    image: &ResourceBlob,
    memory: &mut SparseMemory,
    expected_machine: u16,
    map: F,
) -> Result<u64, String>
where
    F: Fn(u64) -> Option<u64> + Copy,
{
    let mut header = [0; 64];
    image.read(0, &mut header)?;
    validate_ident(&header, 2, 2, expected_machine)?;
    let entry = be64(&header, 24)?;
    let phoff = be64(&header, 32)?;
    let phentsize = usize::from(be16(&header, 54)?);
    let phnum = usize::from(be16(&header, 56)?);
    if phentsize < 56 || phnum > 4096 {
        return Err("ELF64 program header table is invalid".into());
    }
    for index in 0..phnum {
        let offset = phoff
            .checked_add((index * phentsize) as u64)
            .ok_or_else(|| "ELF64 program header offset overflow".to_string())?;
        let mut ph = vec![0; phentsize];
        image.read(offset, &mut ph)?;
        if be32(&ph, 0)? != PT_LOAD {
            continue;
        }
        load_segment(
            image,
            memory,
            be64(&ph, 8)?,
            be64(&ph, 16)?,
            be64(&ph, 32)?,
            be64(&ph, 40)?,
            map,
        )?;
    }
    map(entry).ok_or_else(|| "ELF64 entry point is not mapped".to_string())?;
    Ok(entry)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn elf64(program: &[u8], machine: u16) -> ResourceBlob {
        let mut bytes = vec![0; 0x100 + program.len()];
        bytes[..4].copy_from_slice(b"\x7fELF");
        bytes[4] = 2;
        bytes[5] = 2;
        bytes[6] = 1;
        bytes[16..18].copy_from_slice(&2u16.to_be_bytes());
        bytes[18..20].copy_from_slice(&machine.to_be_bytes());
        bytes[20..24].copy_from_slice(&1u32.to_be_bytes());
        bytes[24..32].copy_from_slice(&0x1000u64.to_be_bytes());
        bytes[32..40].copy_from_slice(&64u64.to_be_bytes());
        bytes[52..54].copy_from_slice(&64u16.to_be_bytes());
        bytes[54..56].copy_from_slice(&56u16.to_be_bytes());
        bytes[56..58].copy_from_slice(&1u16.to_be_bytes());
        bytes[64..68].copy_from_slice(&PT_LOAD.to_be_bytes());
        bytes[68..72].copy_from_slice(&5u32.to_be_bytes());
        bytes[72..80].copy_from_slice(&0x100u64.to_be_bytes());
        bytes[80..88].copy_from_slice(&0x1000u64.to_be_bytes());
        bytes[88..96].copy_from_slice(&0x1000u64.to_be_bytes());
        bytes[96..104].copy_from_slice(&(program.len() as u64).to_be_bytes());
        bytes[104..112].copy_from_slice(&(0x100u64).to_be_bytes());
        bytes[112..120].copy_from_slice(&0x1000u64.to_be_bytes());
        bytes[0x100..].copy_from_slice(program);
        ResourceBlob::from_bytes(&bytes)
    }

    #[test]
    fn elf64_big_endian_loads_segment_and_bss() {
        let image = elf64(&[1, 2, 3, 4], EM_PPC64);
        let mut memory = SparseMemory::new(0x4000).unwrap();
        memory.write(0x1000, &[0xff; 16]).unwrap();
        let entry = load_elf64_be(&image, &mut memory, EM_PPC64, Some).unwrap();
        assert_eq!(entry, 0x1000);
        let mut output = [0; 8];
        memory.read(0x1000, &mut output).unwrap();
        assert_eq!(output, [1, 2, 3, 4, 0, 0, 0, 0]);
    }

    #[test]
    fn elf_loader_rejects_wrong_machine() {
        let image = elf64(&[0; 4], EM_PPC64);
        let mut memory = SparseMemory::new(0x4000).unwrap();
        assert!(load_elf64_be(&image, &mut memory, EM_AARCH64, Some).is_err());
    }
}

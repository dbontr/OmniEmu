const CMIF_IN_MAGIC: u32 = 0x4943_4653;
const CMIF_OUT_MAGIC: u32 = 0x4f43_4653;
const CMIF_COMMAND_CLOSE: u16 = 2;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BufferDescriptor {
    pub address: u64,
    pub size: u64,
    pub mode: u8,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Request {
    pub command_type: u16,
    pub command_id: u32,
    pub domain_object: Option<u32>,
    pub payload: Vec<u8>,
    pub send_buffers: Vec<BufferDescriptor>,
    pub recv_buffers: Vec<BufferDescriptor>,
    pub exch_buffers: Vec<BufferDescriptor>,
    pub send_pid: bool,
    pub close: bool,
}

pub struct Response<'a> {
    pub result: u32,
    pub data: &'a [u8],
    pub copy_handles: &'a [u32],
    pub move_handles: &'a [u32],
    pub domain_objects: &'a [u32],
}

fn read_u32(bytes: &[u8], offset: usize) -> Result<u32, String> {
    let value = bytes
        .get(offset..offset + 4)
        .ok_or_else(|| "HIPC message is truncated".to_string())?;
    Ok(u32::from_le_bytes(value.try_into().unwrap()))
}

fn read_buffer_descriptor(bytes: &[u8], offset: usize) -> Result<BufferDescriptor, String> {
    let size_low = u64::from(read_u32(bytes, offset)?);
    let address_low = u64::from(read_u32(bytes, offset + 4)?);
    let packed = read_u32(bytes, offset + 8)?;
    let mode = (packed & 0x3) as u8;
    let address_high = u64::from((packed >> 2) & 0x3f_ffff);
    let size_high = u64::from((packed >> 24) & 0xf);
    let address_mid = u64::from((packed >> 28) & 0xf);
    Ok(BufferDescriptor {
        address: address_low | (address_mid << 32) | (address_high << 36),
        size: size_low | (size_high << 32),
        mode,
    })
}

fn align16(value: usize) -> Result<usize, String> {
    value
        .checked_add(15)
        .map(|value| value & !15)
        .ok_or_else(|| "HIPC alignment overflow".to_string())
}

pub fn parse_request(tls: &[u8]) -> Result<Request, String> {
    if tls.len() < 8 {
        return Err("HIPC header is truncated".into());
    }
    let word0 = read_u32(tls, 0)?;
    let word1 = read_u32(tls, 4)?;
    let command_type = (word0 & 0xffff) as u16;
    if command_type == CMIF_COMMAND_CLOSE {
        return Ok(Request {
            command_type,
            command_id: 0,
            domain_object: None,
            payload: Vec::new(),
            send_buffers: Vec::new(),
            recv_buffers: Vec::new(),
            exch_buffers: Vec::new(),
            send_pid: false,
            close: true,
        });
    }

    let send_statics = ((word0 >> 16) & 0xf) as usize;
    let send_buffers = ((word0 >> 20) & 0xf) as usize;
    let recv_buffers = ((word0 >> 24) & 0xf) as usize;
    let exch_buffers = ((word0 >> 28) & 0xf) as usize;
    let data_words = (word1 & 0x3ff) as usize;
    let mut offset = 8usize;
    let mut send_pid = false;
    if word1 >> 31 != 0 {
        let special = read_u32(tls, offset)?;
        offset += 4;
        send_pid = special & 1 != 0;
        let copy_handles = ((special >> 1) & 0xf) as usize;
        let move_handles = ((special >> 5) & 0xf) as usize;
        if send_pid {
            offset = offset.checked_add(8).ok_or("HIPC PID overflow")?;
        }
        offset = offset
            .checked_add(4 * (copy_handles + move_handles))
            .ok_or("HIPC handle overflow")?;
    }
    offset = offset
        .checked_add(send_statics * 8)
        .ok_or_else(|| "HIPC static-descriptor layout overflows".to_string())?;
    let mut parsed_send_buffers = Vec::with_capacity(send_buffers);
    for _ in 0..send_buffers {
        parsed_send_buffers.push(read_buffer_descriptor(tls, offset)?);
        offset = offset
            .checked_add(12)
            .ok_or_else(|| "HIPC send-buffer layout overflows".to_string())?;
    }
    let mut parsed_recv_buffers = Vec::with_capacity(recv_buffers);
    for _ in 0..recv_buffers {
        parsed_recv_buffers.push(read_buffer_descriptor(tls, offset)?);
        offset = offset
            .checked_add(12)
            .ok_or_else(|| "HIPC receive-buffer layout overflows".to_string())?;
    }
    let mut parsed_exch_buffers = Vec::with_capacity(exch_buffers);
    for _ in 0..exch_buffers {
        parsed_exch_buffers.push(read_buffer_descriptor(tls, offset)?);
        offset = offset
            .checked_add(12)
            .ok_or_else(|| "HIPC exchange-buffer layout overflows".to_string())?;
    }
    let data_start = offset;
    let data_end = data_start
        .checked_add(data_words * 4)
        .filter(|end| *end <= tls.len())
        .ok_or_else(|| "HIPC data words exceed TLS".to_string())?;
    let cmif_start = align16(data_start)?;
    if cmif_start >= data_end {
        return Err("HIPC request contains no CMIF data".into());
    }

    let first = read_u32(tls, cmif_start)?;
    let (domain_object, cmif_header, close) = if first == CMIF_IN_MAGIC {
        (None, cmif_start, false)
    } else {
        let domain_type = *tls
            .get(cmif_start)
            .ok_or_else(|| "CMIF domain header is truncated".to_string())?;
        let object = read_u32(tls, cmif_start + 4)?;
        if domain_type == 2 {
            return Ok(Request {
                command_type,
                command_id: 0,
                domain_object: Some(object),
                payload: Vec::new(),
                send_buffers: parsed_send_buffers,
                recv_buffers: parsed_recv_buffers,
                exch_buffers: parsed_exch_buffers,
                send_pid,
                close: true,
            });
        }
        if domain_type != 1 {
            return Err("CMIF domain request type is invalid".into());
        }
        (Some(object), cmif_start + 16, false)
    };
    if read_u32(tls, cmif_header)? != CMIF_IN_MAGIC {
        return Err("CMIF input magic is invalid".into());
    }
    let command_id = read_u32(tls, cmif_header + 8)?;
    let payload_start = cmif_header + 16;
    let payload = if payload_start <= data_end {
        tls[payload_start..data_end].to_vec()
    } else {
        Vec::new()
    };
    Ok(Request {
        command_type,
        command_id,
        domain_object,
        payload,
        send_buffers: parsed_send_buffers,
        recv_buffers: parsed_recv_buffers,
        exch_buffers: parsed_exch_buffers,
        send_pid,
        close,
    })
}

pub fn encode_response(response: Response<'_>, domain: bool) -> Result<Vec<u8>, String> {
    let has_special = !response.copy_handles.is_empty() || !response.move_handles.is_empty();
    if response.copy_handles.len() > 15 || response.move_handles.len() > 15 {
        return Err("HIPC response has too many handles".into());
    }
    let mut bytes = vec![0u8; 0x100];
    let mut data_start = 8usize;
    if has_special {
        let special = ((response.copy_handles.len() as u32) << 1)
            | ((response.move_handles.len() as u32) << 5);
        bytes[8..12].copy_from_slice(&special.to_le_bytes());
        data_start = 12;
    }
    for handle in response.copy_handles {
        bytes[data_start..data_start + 4].copy_from_slice(&handle.to_le_bytes());
        data_start += 4;
    }
    for handle in response.move_handles {
        bytes[data_start..data_start + 4].copy_from_slice(&handle.to_le_bytes());
        data_start += 4;
    }
    let cmif_start = align16(data_start)?;
    let mut cursor = cmif_start;
    if domain {
        bytes[cursor..cursor + 4]
            .copy_from_slice(&(response.domain_objects.len() as u32).to_le_bytes());
        cursor += 16;
    }
    bytes[cursor..cursor + 4].copy_from_slice(&CMIF_OUT_MAGIC.to_le_bytes());
    bytes[cursor + 4..cursor + 8].copy_from_slice(&0u32.to_le_bytes());
    bytes[cursor + 8..cursor + 12].copy_from_slice(&response.result.to_le_bytes());
    bytes[cursor + 12..cursor + 16].copy_from_slice(&0u32.to_le_bytes());
    cursor += 16;
    let data_end = cursor
        .checked_add(response.data.len())
        .ok_or_else(|| "CMIF response data overflows".to_string())?;
    if data_end > bytes.len() {
        return Err("CMIF response exceeds TLS buffer".into());
    }
    bytes[cursor..data_end].copy_from_slice(response.data);
    cursor = data_end;
    for object in response.domain_objects {
        let end = cursor + 4;
        if end > bytes.len() {
            return Err("CMIF domain objects exceed TLS buffer".into());
        }
        bytes[cursor..end].copy_from_slice(&object.to_le_bytes());
        cursor = end;
    }
    let data_words = (cursor - data_start).div_ceil(4);
    if data_words > 0x3ff {
        return Err("HIPC response data-word count exceeds encoding".into());
    }
    let word1 = data_words as u32 | if has_special { 1u32 << 31 } else { 0 };
    bytes[4..8].copy_from_slice(&word1.to_le_bytes());
    Ok(bytes)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn simple_request(command_type: u16, command_id: u32, payload: &[u8]) -> Vec<u8> {
        let mut bytes = vec![0u8; 0x100];
        let cmif_start = 16usize;
        let end = cmif_start + 16 + payload.len();
        let words = (end - 8).div_ceil(4);
        bytes[0..4].copy_from_slice(&u32::from(command_type).to_le_bytes());
        bytes[4..8].copy_from_slice(&(words as u32).to_le_bytes());
        bytes[cmif_start..cmif_start + 4].copy_from_slice(&CMIF_IN_MAGIC.to_le_bytes());
        bytes[cmif_start + 8..cmif_start + 12].copy_from_slice(&command_id.to_le_bytes());
        bytes[cmif_start + 16..end].copy_from_slice(payload);
        bytes
    }

    #[test]
    fn parses_non_domain_cmif_request() {
        let bytes = simple_request(4, 1, b"time:u\0\0");
        let request = parse_request(&bytes).unwrap();
        assert_eq!(request.command_type, 4);
        assert_eq!(request.command_id, 1);
        assert_eq!(&request.payload[..8], b"time:u\0\0");
        assert_eq!(request.domain_object, None);
        assert!(!request.close);
    }

    #[test]
    fn parses_mapped_receive_buffer_descriptor() {
        let mut bytes = vec![0u8; 0x100];
        let address = 0x71_2345_0000u64;
        let size = 0x1_0000_0100u64;
        let packed = (((address >> 36) as u32 & 0x3f_ffff) << 2)
            | (((size >> 32) as u32 & 0xf) << 24)
            | (((address >> 32) as u32 & 0xf) << 28);
        bytes[0..4].copy_from_slice(&(4u32 | (1 << 24)).to_le_bytes());
        bytes[8..12].copy_from_slice(&(size as u32).to_le_bytes());
        bytes[12..16].copy_from_slice(&(address as u32).to_le_bytes());
        bytes[16..20].copy_from_slice(&packed.to_le_bytes());
        let cmif_start = 32usize;
        let end = cmif_start + 16;
        bytes[4..8].copy_from_slice(&(((end - 20) / 4) as u32).to_le_bytes());
        bytes[cmif_start..cmif_start + 4].copy_from_slice(&CMIF_IN_MAGIC.to_le_bytes());
        bytes[cmif_start + 8..cmif_start + 12].copy_from_slice(&2020u32.to_le_bytes());

        let request = parse_request(&bytes).unwrap();
        assert!(request.send_buffers.is_empty());
        assert!(request.exch_buffers.is_empty());
        assert_eq!(
            request.recv_buffers,
            vec![BufferDescriptor {
                address,
                size,
                mode: 0,
            }]
        );
    }

    #[test]
    fn response_exposes_move_handle_and_cmif_payload() {
        let bytes = encode_response(
            Response {
                result: 0,
                data: &0x1234u32.to_le_bytes(),
                copy_handles: &[],
                move_handles: &[0x200],
                domain_objects: &[],
            },
            false,
        )
        .unwrap();
        assert_ne!(read_u32(&bytes, 4).unwrap() >> 31, 0);
        assert_eq!(read_u32(&bytes, 12).unwrap(), 0x200);
        assert_eq!(read_u32(&bytes, 16).unwrap(), CMIF_OUT_MAGIC);
        assert_eq!(read_u32(&bytes, 24).unwrap(), 0);
        assert_eq!(read_u32(&bytes, 32).unwrap(), 0x1234);
    }

    #[test]
    fn rejects_truncated_data_words() {
        let mut bytes = simple_request(4, 1, &[]);
        bytes[4..8].copy_from_slice(&0x3ffu32.to_le_bytes());
        assert!(parse_request(&bytes).is_err());
    }
}

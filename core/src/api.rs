use std::slice;
use std::sync::{Mutex, OnceLock};

use crate::kernel::InputState;
use crate::machine::{create_machine, Machine};
use crate::platform::{PlatformId, SupportLevel};

#[derive(Default)]
struct CoreState {
    machine: Option<Box<dyn Machine>>,
    input: InputState,
    last_error: String,
}

fn core() -> &'static Mutex<CoreState> {
    static CORE: OnceLock<Mutex<CoreState>> = OnceLock::new();
    CORE.get_or_init(|| Mutex::new(CoreState::default()))
}

fn fail(state: &mut CoreState, message: impl Into<String>) -> i32 {
    state.last_error = message.into();
    -1
}
#[no_mangle]
pub extern "C" fn omni_core_version() -> u32 {
    1
}

#[no_mangle]
pub extern "C" fn omni_can_launch(platform: u32) -> u32 {
    u32::from(PlatformId::from_u32(platform).is_some_and(PlatformId::is_launchable))
}

#[no_mangle]
pub extern "C" fn omni_support_level(platform: u32) -> u32 {
    PlatformId::from_u32(platform)
        .map(|id| id.support_level() as u32)
        .unwrap_or(SupportLevel::Planned as u32)
}

#[no_mangle]
pub extern "C" fn omni_alloc(len: usize) -> *mut u8 {
    if len == 0 {
        return std::ptr::null_mut();
    }
    let mut bytes = Vec::<u8>::with_capacity(len);
    let ptr = bytes.as_mut_ptr();
    std::mem::forget(bytes);
    ptr
}

#[no_mangle]
pub unsafe extern "C" fn omni_free(ptr: *mut u8, len: usize) {
    if !ptr.is_null() && len > 0 {
        drop(Vec::from_raw_parts(ptr, 0, len));
    }
}
#[no_mangle]
pub unsafe extern "C" fn omni_load(
    platform: u32,
    rom_ptr: *const u8,
    rom_len: usize,
    bios_ptr: *const u8,
    bios_len: usize,
) -> i32 {
    let mut state = core().lock().unwrap();
    state.machine = None;
    state.last_error.clear();
    let Some(platform) = PlatformId::from_u32(platform) else {
        return fail(&mut state, "unknown platform id");
    };
    let rom = if rom_len == 0 {
        &[]
    } else if rom_ptr.is_null() {
        return fail(&mut state, "ROM pointer is null");
    } else {
        slice::from_raw_parts(rom_ptr, rom_len)
    };
    let bios = if bios_len == 0 {
        &[]
    } else if bios_ptr.is_null() {
        return fail(&mut state, "BIOS pointer is null");
    } else {
        slice::from_raw_parts(bios_ptr, bios_len)
    };
    match create_machine(platform, rom, bios) {
        Ok(machine) => {
            state.machine = Some(machine);
            0
        }
        Err(error) => fail(&mut state, error),
    }
}

#[no_mangle]
pub extern "C" fn omni_unload() {
    core().lock().unwrap().machine = None;
}
#[no_mangle]
pub extern "C" fn omni_reset() -> i32 {
    let mut state = core().lock().unwrap();
    match state.machine.as_mut() {
        Some(machine) => {
            machine.reset();
            0
        }
        None => fail(&mut state, "no machine is loaded"),
    }
}

#[no_mangle]
pub extern "C" fn omni_set_input(player: u32, buttons: u64) -> i32 {
    let mut state = core().lock().unwrap();
    let Some(slot) = state.input.buttons.get_mut(player as usize) else {
        return fail(&mut state, "player index is out of range");
    };
    *slot = buttons;
    0
}

#[no_mangle]
pub extern "C" fn omni_run_frame() -> i32 {
    let mut state = core().lock().unwrap();
    let input = state.input.clone();
    match state.machine.as_mut() {
        Some(machine) => {
            machine.run_frame(&input);
            0
        }
        None => fail(&mut state, "no machine is loaded"),
    }
}
#[no_mangle]
pub extern "C" fn omni_video_ptr() -> *const u8 {
    core()
        .lock()
        .unwrap()
        .machine
        .as_ref()
        .map(|machine| machine.video().pixels().as_ptr())
        .unwrap_or(std::ptr::null())
}
#[no_mangle]
pub extern "C" fn omni_video_len() -> usize {
    core()
        .lock()
        .unwrap()
        .machine
        .as_ref()
        .map(|machine| machine.video().pixels().len())
        .unwrap_or(0)
}
#[no_mangle]
pub extern "C" fn omni_video_width() -> u32 {
    core()
        .lock()
        .unwrap()
        .machine
        .as_ref()
        .map(|machine| machine.video().width())
        .unwrap_or(0)
}
#[no_mangle]
pub extern "C" fn omni_video_height() -> u32 {
    core()
        .lock()
        .unwrap()
        .machine
        .as_ref()
        .map(|machine| machine.video().height())
        .unwrap_or(0)
}

#[no_mangle]
pub extern "C" fn omni_audio_ptr() -> *const f32 {
    core()
        .lock()
        .unwrap()
        .machine
        .as_ref()
        .map(|machine| machine.audio().samples().as_ptr())
        .unwrap_or(std::ptr::null())
}
#[no_mangle]
pub extern "C" fn omni_audio_len() -> usize {
    core()
        .lock()
        .unwrap()
        .machine
        .as_ref()
        .map(|machine| machine.audio().samples().len())
        .unwrap_or(0)
}
#[no_mangle]
pub extern "C" fn omni_audio_rate() -> u32 {
    core()
        .lock()
        .unwrap()
        .machine
        .as_ref()
        .map(|machine| machine.audio().sample_rate())
        .unwrap_or(0)
}
#[no_mangle]
pub extern "C" fn omni_audio_channels() -> u32 {
    core()
        .lock()
        .unwrap()
        .machine
        .as_ref()
        .map(|machine| machine.audio().channels())
        .unwrap_or(0)
}

#[no_mangle]
pub extern "C" fn omni_last_error_ptr() -> *const u8 {
    core().lock().unwrap().last_error.as_ptr()
}
#[no_mangle]
pub extern "C" fn omni_last_error_len() -> usize {
    core().lock().unwrap().last_error.len()
}
#[no_mangle]
pub unsafe extern "C" fn omni_save_state(out_ptr: *mut u8, capacity: usize) -> usize {
    let mut state = core().lock().unwrap();
    let Some(machine) = state.machine.as_ref() else {
        fail(&mut state, "no machine is loaded");
        return 0;
    };
    let bytes = match machine.save_state() {
        Ok(bytes) => bytes,
        Err(error) => {
            fail(&mut state, error);
            return 0;
        }
    };
    if capacity < bytes.len() || out_ptr.is_null() {
        if capacity != 0 {
            fail(&mut state, "save-state output buffer is too small");
        }
        return bytes.len();
    }
    std::ptr::copy_nonoverlapping(bytes.as_ptr(), out_ptr, bytes.len());
    bytes.len()
}

#[no_mangle]
pub unsafe extern "C" fn omni_load_state(ptr: *const u8, len: usize) -> i32 {
    let mut state = core().lock().unwrap();
    if ptr.is_null() || len == 0 {
        return fail(&mut state, "save state is empty");
    }
    let bytes = slice::from_raw_parts(ptr, len);
    let Some(machine) = state.machine.as_mut() else {
        return fail(&mut state, "no machine is loaded");
    };
    match machine.load_state(bytes) {
        Ok(()) => 0,
        Err(error) => fail(&mut state, error),
    }
}
#[no_mangle]
pub extern "C" fn omni_frame_rate() -> f64 {
    core()
        .lock()
        .unwrap()
        .machine
        .as_ref()
        .map(|machine| machine.frame_rate())
        .unwrap_or(60.0)
}

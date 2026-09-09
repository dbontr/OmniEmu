export type ControlId =
  | 'b' | 'y' | 'select' | 'start' | 'up' | 'down' | 'left' | 'right'
  | 'a' | 'x' | 'l' | 'r' | 'l2' | 'r2' | 'l3' | 'r3'
  | 'lsRight' | 'lsLeft' | 'lsDown' | 'lsUp'
  | 'rsRight' | 'rsLeft' | 'rsDown' | 'rsUp'
  | 'quickSave' | 'quickLoad' | 'stateSlot' | 'fastForward' | 'rewind' | 'slowMotion'

export interface Binding {
  id: ControlId
  label: string
  index: number
  keyboard: string
  gamepad: string
}

export interface InputProfile {
  version: 1
  players: Record<number, Binding[]>
}

const controls: Omit<Binding, 'keyboard' | 'gamepad'>[] = [
  { id: 'b', label: 'B / East', index: 0 },
  { id: 'y', label: 'Y / North', index: 1 },
  { id: 'select', label: 'Select / Back', index: 2 },
  { id: 'start', label: 'Start', index: 3 },
  { id: 'up', label: 'D-pad Up', index: 4 },
  { id: 'down', label: 'D-pad Down', index: 5 },
  { id: 'left', label: 'D-pad Left', index: 6 },
  { id: 'right', label: 'D-pad Right', index: 7 },
  { id: 'a', label: 'A / South', index: 8 },
  { id: 'x', label: 'X / West', index: 9 },
  { id: 'l', label: 'Left Shoulder', index: 10 },
  { id: 'r', label: 'Right Shoulder', index: 11 },
  { id: 'l2', label: 'Left Trigger', index: 12 },
  { id: 'r2', label: 'Right Trigger', index: 13 },
  { id: 'l3', label: 'Left Stick Press', index: 14 },
  { id: 'r3', label: 'Right Stick Press', index: 15 },
]
const defaults: Record<ControlId, [string, string]> = {
  b: ['z', 'BUTTON_2'], y: ['a', 'BUTTON_4'], select: ['shift', 'SELECT'], start: ['enter', 'START'],
  up: ['arrowup', 'DPAD_UP'], down: ['arrowdown', 'DPAD_DOWN'], left: ['arrowleft', 'DPAD_LEFT'], right: ['arrowright', 'DPAD_RIGHT'],
  a: ['x', 'BUTTON_1'], x: ['s', 'BUTTON_3'], l: ['q', 'LEFT_TOP_SHOULDER'], r: ['w', 'RIGHT_TOP_SHOULDER'],
  l2: ['tab', 'LEFT_BOTTOM_SHOULDER'], r2: ['r', 'RIGHT_BOTTOM_SHOULDER'], l3: ['', 'LEFT_STICK'], r3: ['', 'RIGHT_STICK'],
  lsRight: ['h', 'LEFT_STICK_X:+1'], lsLeft: ['f', 'LEFT_STICK_X:-1'], lsDown: ['g', 'LEFT_STICK_Y:+1'], lsUp: ['t', 'LEFT_STICK_Y:-1'],
  rsRight: ['l', 'RIGHT_STICK_X:+1'], rsLeft: ['j', 'RIGHT_STICK_X:-1'], rsDown: ['k', 'RIGHT_STICK_Y:+1'], rsUp: ['i', 'RIGHT_STICK_Y:-1'],
  quickSave: ['1', ''], quickLoad: ['2', ''], stateSlot: ['3', ''], fastForward: ['+', ''], rewind: [' ', ''], slowMotion: ['-', ''],
}

const advanced: Omit<Binding, 'keyboard' | 'gamepad'>[] = [
  { id: 'lsRight', label: 'Left Stick Right', index: 16 }, { id: 'lsLeft', label: 'Left Stick Left', index: 17 },
  { id: 'lsDown', label: 'Left Stick Down', index: 18 }, { id: 'lsUp', label: 'Left Stick Up', index: 19 },
  { id: 'rsRight', label: 'Right Stick Right', index: 20 }, { id: 'rsLeft', label: 'Right Stick Left', index: 21 },
  { id: 'rsDown', label: 'Right Stick Down', index: 22 }, { id: 'rsUp', label: 'Right Stick Up', index: 23 },
  { id: 'quickSave', label: 'Quick Save', index: 24 }, { id: 'quickLoad', label: 'Quick Load', index: 25 },
  { id: 'stateSlot', label: 'Change State Slot', index: 26 }, { id: 'fastForward', label: 'Fast Forward', index: 27 },
  { id: 'rewind', label: 'Rewind', index: 28 }, { id: 'slowMotion', label: 'Slow Motion', index: 29 },
]

const makeBindings = (keyboardEnabled = true) => [...controls, ...advanced].map((control) => ({
  ...control,
  keyboard: keyboardEnabled ? defaults[control.id][0] : '',
  gamepad: defaults[control.id][1],
}))

export const defaultProfile = (): InputProfile => ({
  version: 1,
  players: { 0: makeBindings(), 1: makeBindings(false), 2: makeBindings(false), 3: makeBindings(false) },
})
const STORAGE_KEY = 'omniemu.input-profile.v1'

export function loadInputProfile(): InputProfile {
  try {
    const parsed = JSON.parse(localStorage.getItem(STORAGE_KEY) ?? 'null') as InputProfile | null
    if (parsed?.version === 1 && parsed.players) return parsed
  } catch {
    // Invalid local state should never prevent a game from launching.
  }
  return defaultProfile()
}

export function saveInputProfile(profile: InputProfile) {
  localStorage.setItem(STORAGE_KEY, JSON.stringify(profile))
}

export const primaryBindings = (profile: InputProfile, player = 0) =>
  (profile.players[player] ?? []).filter((binding) => binding.index <= 15)

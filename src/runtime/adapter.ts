import type { Platform } from '../catalog'
import type { InputProfile } from '../input/mapping'
import { createEmulatorSession, type EmulatorSession } from './emulatorjs'

export interface RuntimeLaunchRequest {
  platform: Platform
  game: File
  bios?: File | null
  inputProfile: InputProfile
}

export interface RuntimeAdapter {
  id: string
  supports(platform: Platform): boolean
  launch(host: HTMLElement, request: RuntimeLaunchRequest): EmulatorSession
}

const emulatorJsAdapter: RuntimeAdapter = {
  id: 'libretro-wasm',
  supports: (platform) => Boolean(platform.core),
  launch: (host, request) => createEmulatorSession(host, request),
}

const adapters: RuntimeAdapter[] = [emulatorJsAdapter]

export const adapterFor = (platform: Platform) => adapters.find((adapter) => adapter.supports(platform)) ?? null
export const runtimeAvailable = (platform: Platform) => adapterFor(platform) !== null

export function launchRuntime(host: HTMLElement, request: RuntimeLaunchRequest) {
  const adapter = adapterFor(request.platform)
  if (!adapter) throw new Error(`${request.platform.name} does not have a connected browser runtime yet.`)
  return adapter.launch(host, request)
}

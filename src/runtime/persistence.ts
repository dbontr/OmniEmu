const DB_NAME = 'omniemu-persistence'
const DB_VERSION = 1
const STORE = 'resources'
const HASH_EDGE_BYTES = 1024 * 1024

let dbPromise: Promise<IDBDatabase> | null = null

function openDb(): Promise<IDBDatabase> {
  dbPromise ??= new Promise((resolve, reject) => {
    const request = indexedDB.open(DB_NAME, DB_VERSION)
    request.onupgradeneeded = () => {
      const db = request.result
      if (!db.objectStoreNames.contains(STORE)) db.createObjectStore(STORE)
    }
    request.onsuccess = () => resolve(request.result)
    request.onerror = () => reject(request.error ?? new Error('Unable to open persistence database.'))
  })
  return dbPromise
}

export async function loadPersistentResource(key: string): Promise<Uint8Array | null> {
  const db = await openDb()
  return new Promise((resolve, reject) => {
    const transaction = db.transaction(STORE, 'readonly')
    const request = transaction.objectStore(STORE).get(key)
    request.onsuccess = () => {
      const value = request.result
      resolve(value instanceof ArrayBuffer ? new Uint8Array(value) : null)
    }
    request.onerror = () => reject(request.error ?? new Error('Unable to read persistent resource.'))
  })
}

export async function savePersistentResource(key: string, bytes: Uint8Array): Promise<void> {
  const db = await openDb()
  const copy = bytes.slice().buffer
  await new Promise<void>((resolve, reject) => {
    const transaction = db.transaction(STORE, 'readwrite')
    transaction.objectStore(STORE).put(copy, key)
    transaction.oncomplete = () => resolve()
    transaction.onerror = () => reject(transaction.error ?? new Error('Unable to write persistent resource.'))
    transaction.onabort = () => reject(transaction.error ?? new Error('Persistence transaction aborted.'))
  })
}

export async function fingerprintFile(file: File): Promise<string> {
  const size = file.size
  const firstEnd = Math.min(size, HASH_EDGE_BYTES)
  const lastStart = Math.max(firstEnd, size - HASH_EDGE_BYTES)
  const first = new Uint8Array(await file.slice(0, firstEnd).arrayBuffer())
  const last = lastStart < size
    ? new Uint8Array(await file.slice(lastStart, size).arrayBuffer())
    : new Uint8Array()
  const metadata = new TextEncoder().encode(`${size}:`)
  const material = new Uint8Array(metadata.length + first.length + last.length)
  material.set(metadata, 0)
  material.set(first, metadata.length)
  material.set(last, metadata.length + first.length)
  const digest = new Uint8Array(await crypto.subtle.digest('SHA-256', material))
  return [...digest].map((byte) => byte.toString(16).padStart(2, '0')).join('')
}

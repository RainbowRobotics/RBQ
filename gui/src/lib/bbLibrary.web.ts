import { isDesktop } from '@/lib/desktopBridge';
import { serialKey, toEntries, type LibEntry } from './bbLibraryCommon';
import { isNoSpaceError, StorageFullError } from './storageCommon';
import { t } from './i18n';

export const libSupported = typeof window !== 'undefined';

const q = (serial: string, name: string) => `/bb-lib?serial=${encodeURIComponent(serial)}&name=${encodeURIComponent(name)}`;

async function ok(res: Response, what: string): Promise<Response> {
  if (res.ok) return res;
  const body = await res.text().catch(() => '');
  if (res.status === 507 || /ENOSPC/.test(body)) throw new StorageFullError(null);
  throw new Error(`${what} — HTTP ${res.status}${body ? ` ${body}` : ''}`);
}

const DB = 'rbq-blackbox';
const STORE = 'zips';
type Rec = { id: string; serial: string; name: string; size: number; blob: Blob };

function db(): Promise<IDBDatabase> {
  return new Promise((resolve, reject) => {
    const r = indexedDB.open(DB, 1);
    r.onupgradeneeded = () => r.result.createObjectStore(STORE, { keyPath: 'id' });
    r.onsuccess = () => resolve(r.result);
    r.onerror = () => reject(r.error);
  });
}

async function tx<T>(mode: IDBTransactionMode, run: (s: IDBObjectStore) => IDBRequest<T>): Promise<T> {
  const d = await db();
  try {
    return await new Promise<T>((resolve, reject) => {
      const tr = d.transaction(STORE, mode);
      const req = run(tr.objectStore(STORE));
      tr.oncomplete = () => resolve(req.result);
      tr.onabort = tr.onerror = () => reject(tr.error ?? req.error);
    });
  } finally { d.close(); }
}

export async function libList(): Promise<LibEntry[]> {
  if (isDesktop()) {
    const res = await ok(await fetch('/bb-lib'), 'bb-lib list');
    return toEntries(await res.json());
  }
  const all = await tx<Rec[]>('readonly', (s) => s.getAll() as IDBRequest<Rec[]>);
  return toEntries(all.map(({ serial, name, size }) => ({ serial, name, size })));
}

export async function libPut(serial: string, name: string, src: { uri: string } | { blob: Blob }): Promise<void> {
  if (!('blob' in src)) throw new Error('bbLibrary(web): blob 만 받는다');
  const sn = serialKey(serial);
  if (isDesktop()) {
    await ok(await fetch(q(sn, name), { method: 'PUT', body: src.blob }), 'bb-lib put');
    return;
  }
  try {
    await tx('readwrite', (s) => s.put({ id: `${sn}/${name}`, serial: sn, name, size: src.blob.size, blob: src.blob } satisfies Rec));
  } catch (e) {
    if ((e as DOMException)?.name === 'QuotaExceededError' || isNoSpaceError(e)) throw new StorageFullError(null);
    throw e;
  }
}

export async function libRead(e: LibEntry): Promise<Uint8Array> {
  if (isDesktop()) {
    const res = await ok(await fetch(q(e.serial, e.name)), 'bb-lib read');
    return new Uint8Array(await res.arrayBuffer());
  }
  const rec = await tx<Rec | undefined>('readonly', (s) => s.get(`${e.serial}/${e.name}`) as IDBRequest<Rec | undefined>);
  if (!rec) throw new Error(`${e.name} — ${t('보관함에 없습니다')}`);
  return new Uint8Array(await rec.blob.arrayBuffer());
}

export async function libRemove(e: LibEntry): Promise<void> {
  if (isDesktop()) {
    await ok(await fetch(q(e.serial, e.name), { method: 'DELETE' }), 'bb-lib delete');
    return;
  }
  await tx('readwrite', (s) => s.delete(`${e.serial}/${e.name}`));
}


export type RegisterDraft = {
  serial: string;
  name: string;
  lanIp: string;
  rendezvousUrl: string;
};

export type RegisterCheck = { ok: true } | { ok: false; reason: RegisterBlock };
export type RegisterBlock = 'serial_empty' | 'serial_charset' | 'name_empty' | 'lan_empty' | 'rv_scheme';

export function validateDraft(d: RegisterDraft): RegisterCheck {
  const serial = d.serial.trim();
  if (!serial) return { ok: false, reason: 'serial_empty' };
  if (/[\s/\\]/.test(serial)) return { ok: false, reason: 'serial_charset' };
  if (!d.name.trim()) return { ok: false, reason: 'name_empty' };
  if (!d.lanIp.trim()) return { ok: false, reason: 'lan_empty' };
  const rv = d.rendezvousUrl.trim();
  if (rv && !/^wss?:\/\//.test(rv)) return { ok: false, reason: 'rv_scheme' };
  return { ok: true };
}

export function registerValues(d: RegisterDraft) {
  const serial = d.serial.trim();
  return {
    serial,
    name: d.name.trim(),
    lanIp: d.lanIp.trim(),
    rendezvousUrl: d.rendezvousUrl.trim(),
    robotId: serial,
  };
}

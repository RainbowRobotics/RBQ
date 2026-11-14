
export type LibEntry = {
  serial: string;
  name: string;
  date: string;
  session: string;
  size: number;
};

export const NO_SERIAL = '@nosn';

export function serialKey(serial: string): string {
  const sn = serial.trim().replace(/[^A-Za-z0-9._-]/g, '_');
  return sn && !/^\.+$/.test(sn) ? sn : NO_SERIAL;
}

export function libName(serial: string, date: string, session: string): string {
  const sn = serialKey(serial);
  return sn === NO_SERIAL ? `blackbox-${date}_${session}.zip` : `blackbox-${sn}-${date}_${session}.zip`;
}

export function parseLibName(name: string): { date: string; session: string } | null {
  const m = /^blackbox-(?:.+-)?(\d{8})_(\d{2}_\d{2}_\d{2})\.zip$/.exec(name);
  return m ? { date: m[1], session: m[2] } : null;
}

export function toEntries(files: { serial: string; name: string; size: number }[]): LibEntry[] {
  const out: LibEntry[] = [];
  for (const f of files) {
    const p = parseLibName(f.name);
    if (p) out.push({ serial: f.serial, name: f.name, size: f.size, ...p });
  }
  return out;
}

export function libSerials(entries: LibEntry[]): string[] {
  return [...new Set(entries.map((e) => e.serial))]
    .sort((a, b) => (a === NO_SERIAL ? 1 : b === NO_SERIAL ? -1 : a.localeCompare(b)));
}

export function libDates(entries: LibEntry[], serial: string): string[] {
  return [...new Set(entries.filter((e) => e.serial === serial).map((e) => e.date))].sort().reverse();
}

export function libSessions(entries: LibEntry[], serial: string, date: string): string[] {
  return entries.filter((e) => e.serial === serial && e.date === date).map((e) => e.session).sort().reverse();
}

export type Axis = 'x' | 'y' | 'z';
export type PayloadCom = { x_m: number; y_m: number; z_m: number };

export type PayloadSlot = {
  id: number; name: string; label?: string; is_custom?: boolean;
  mass_kg: number; center_of_mass: PayloadCom;
  default_mass_kg?: number; default_center_of_mass?: PayloadCom;
};

export type PayloadRow = {
  id: number; name: string; label: string; isCustom: boolean; enabled: boolean;
  mass: number; x: number; y: number; z: number;
  defaultMass: number; defaultX: number; defaultY: number; defaultZ: number;
};

export type PayloadLimits = {
  mass_min_kg?: number; mass_max_kg?: number;
  mass_total_min_kg?: number; mass_total_max_kg?: number;
  com_x_max_m?: number; com_y_max_m?: number; com_z_max_m?: number;
};

const FALLBACK_LIMITS = {
  massMin: 0, massMax: 20, massTotalMin: 0, massTotalMax: 20,
  xMax: 0.5, yMax: 0.3, zMax: 0.4,
};

export const PAYLOAD_LIMITS = { ...FALLBACK_LIMITS };

export function applyServerLimits(l?: PayloadLimits) {
  Object.assign(PAYLOAD_LIMITS, FALLBACK_LIMITS);
  if (!l) return;
  const put = (dst: keyof typeof PAYLOAD_LIMITS, v?: number) => {
    if (typeof v === 'number' && Number.isFinite(v)) PAYLOAD_LIMITS[dst] = v;
  };
  put('massMin', l.mass_min_kg);            put('massMax', l.mass_max_kg);
  put('massTotalMin', l.mass_total_min_kg); put('massTotalMax', l.mass_total_max_kg);
  put('xMax', l.com_x_max_m);               put('yMax', l.com_y_max_m);
  put('zMax', l.com_z_max_m);
}

export const limitOf = (k: Axis) =>
  k === 'x' ? PAYLOAD_LIMITS.xMax : k === 'y' ? PAYLOAD_LIMITS.yMax : PAYLOAD_LIMITS.zMax;

export const clampAxis = (v: number, k: Axis) =>
  Number.isFinite(v) ? Math.max(-limitOf(k), Math.min(limitOf(k), v)) : 0;

export const clampMass = (v: number) =>
  Number.isFinite(v) ? Math.max(PAYLOAD_LIMITS.massMin, Math.min(PAYLOAD_LIMITS.massMax, v)) : 0;

export function toRows(slots: PayloadSlot[], keep?: (slotId: number) => boolean): PayloadRow[] {
  const rows: PayloadRow[] = slots
    .filter((s) => !keep || keep(s.id))
    .map((s) => ({
      id: s.id,
      name: s.name ?? String(s.id),
      label: s.label ?? s.name ?? `Slot ${s.id}`,
      isCustom: s.is_custom ?? s.name?.startsWith('CUSTOM') ?? false,
      enabled: Math.abs(s.mass_kg) > 1e-6,
      mass: s.mass_kg,
      x: s.center_of_mass.x_m, y: s.center_of_mass.y_m, z: s.center_of_mass.z_m,
      defaultMass: s.default_mass_kg ?? 0,
      defaultX: s.default_center_of_mass?.x_m ?? 0,
      defaultY: s.default_center_of_mass?.y_m ?? 0,
      defaultZ: s.default_center_of_mass?.z_m ?? 0,
    }));
  return [...rows.filter((r) => !r.isCustom), ...rows.filter((r) => r.isCustom)];
}

export function patchRow(rows: PayloadRow[], id: number, p: Partial<PayloadRow>): PayloadRow[] {
  return rows.map((r) => (r.id === id ? { ...r, ...p } : r));
}

export function toggleRow(rows: PayloadRow[], id: number): PayloadRow[] {
  const r = rows.find((row) => row.id === id);
  if (!r) return rows;
  if (r.enabled) return patchRow(rows, id, { enabled: false });
  const blank = Math.abs(r.mass) < 1e-6
    && Math.abs(r.x) < 1e-9 && Math.abs(r.y) < 1e-9 && Math.abs(r.z) < 1e-9;
  return patchRow(rows, id, blank
    ? { enabled: true, mass: r.defaultMass, x: r.defaultX, y: r.defaultY, z: r.defaultZ }
    : { enabled: true });
}

export function nudgeRow(rows: PayloadRow[], id: number, k: Axis | 'mass', delta: number): PayloadRow[] {
  const r = rows.find((row) => row.id === id);
  if (!r) return rows;
  return k === 'mass'
    ? patchRow(rows, id, { mass: clampMass(r.mass + delta) })
    : patchRow(rows, id, { [k]: clampAxis(r[k] + delta, k) });
}

export const exceedsTotal = (mass: number) => mass > PAYLOAD_LIMITS.massTotalMax + 1e-6;

export function combine(rows: PayloadRow[]) {
  let mass = 0, wx = 0, wy = 0, wz = 0;
  for (const r of rows) {
    if (!r.enabled) continue;
    mass += r.mass; wx += r.mass * r.x; wy += r.mass * r.y; wz += r.mass * r.z;
  }
  if (Math.abs(mass) < 1e-3) return { mass, x: 0, y: 0, z: 0 };
  return { mass, x: wx / mass, y: wy / mass, z: wz / mass };
}

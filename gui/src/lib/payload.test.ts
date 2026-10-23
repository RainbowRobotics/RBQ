import { describe, it, expect, afterEach } from 'vitest';
import { toRows, toggleRow, nudgeRow, patchRow, combine, clampMass, clampAxis, exceedsTotal,
         applyServerLimits, PAYLOAD_LIMITS } from './payload';
import type { PayloadSlot } from './rest';

const NAMES = [
  'CUSTOM1', 'CUSTOM2', 'CUSTOM3', 'CUSTOM4', 'CUSTOM5',
  'PTZ_CAM', 'LIDAR_LIVOX1', 'LIDAR_LIVOX2', 'LIDAR_OUSTER',
  'SOUND_CAM', 'LTE_5G', 'SWITCH_HUB', 'UPC_OUTER',
];
const LABELS = [
  'Custom 1', 'Custom 2', 'Custom 3', 'Custom 4', 'Custom 5',
  'PTZ Camera', 'Livox LiDAR Front', 'Livox LiDAR Rear', 'Ouster LiDAR',
  'Sound Camera', 'LTE / 5G', 'Switch Hub', 'External UPC',
];
const DEF: Record<number, { m: number; c: [number, number, number] }> = {
  5: { m: 5.3, c: [0.065, 0, 0.19] },
  8: { m: 0.65, c: [-0.312, 0, 0.132] },
  10: { m: 0.7, c: [-0.275, 0, 0.085] },
};

function serverSlots(): PayloadSlot[] {
  return NAMES.map((name, id) => ({
    id, name, label: LABELS[id], is_custom: id <= 4,
    mass_kg: 0, center_of_mass: { x_m: 0, y_m: 0, z_m: 0 },
    default_mass_kg: DEF[id]?.m ?? 0,
    default_center_of_mass: {
      x_m: DEF[id]?.c[0] ?? 0, y_m: DEF[id]?.c[1] ?? 0, z_m: DEF[id]?.c[2] ?? 0,
    },
  }));
}

describe('applyServerLimits', () => {
  const snapshot = { ...PAYLOAD_LIMITS };
  afterEach(() => Object.assign(PAYLOAD_LIMITS, snapshot));

  it('로봇이 준 범위로 클램프가 바뀐다', () => {
    applyServerLimits({ mass_max_kg: 8, com_x_max_m: 0.2 });
    expect(clampMass(12)).toBe(8);
    expect(clampAxis(0.4, 'x')).toBe(0.2);
  });

  it('구버전 데몬(limits 없음)이면 폴백으로 되돌아간다 — 이전 로봇 값이 남지 않는다', () => {
    applyServerLimits({ mass_max_kg: 8 });
    applyServerLimits(undefined);
    expect(PAYLOAD_LIMITS.massMax).toBe(snapshot.massMax);
  });

  it('수가 아닌 값은 무시한다', () => {
    applyServerLimits({ mass_max_kg: NaN, com_y_max_m: undefined });
    expect(PAYLOAD_LIMITS.massMax).toBe(snapshot.massMax);
    expect(PAYLOAD_LIMITS.yMax).toBe(snapshot.yMax);
  });
});

describe('nudgeRow(mass)', () => {
  const one = () => toRows(serverSlots()).map((r) => (r.id === 0 ? { ...r, mass: 1 } : r));

  it('mass 도 스텝만큼 움직인다', () => {
    expect(nudgeRow(one(), 0, 'mass', 0.05).find((r) => r.id === 0)?.mass).toBeCloseTo(1.05, 6);
    expect(nudgeRow(one(), 0, 'mass', -0.05).find((r) => r.id === 0)?.mass).toBeCloseTo(0.95, 6);
  });

  it('mass 클램프는 0~20 — 축처럼 ±대칭으로 자르면 안 된다', () => {
    const hi = one().map((r) => (r.id === 0 ? { ...r, mass: 19.99 } : r));
    expect(nudgeRow(hi, 0, 'mass', 0.05).find((r) => r.id === 0)?.mass).toBe(20);
    const lo = one().map((r) => (r.id === 0 ? { ...r, mass: 0.01 } : r));
    expect(nudgeRow(lo, 0, 'mass', -0.05).find((r) => r.id === 0)?.mass).toBe(0);
  });
});

describe('toRows', () => {
  it('고정 payload 먼저, 커스텀은 아래로 재배열', () => {
    const rows = toRows(serverSlots());
    expect(rows.map((r) => r.label).slice(0, 3)).toEqual(['PTZ Camera', 'Livox LiDAR Front', 'Livox LiDAR Rear']);
    expect(rows.slice(-5).every((r) => r.isCustom)).toBe(true);
  });

  it('재배열해도 id↔label 이 어긋나지 않는다 (인덱스 혼동 회귀 방지)', () => {
    for (const r of toRows(serverSlots())) {
      expect(r.label).toBe(LABELS[r.id]);
      expect(r.name).toBe(NAMES[r.id]);
    }
  });

  it('슬롯 필터가 있으면 그 칸만 연다 (고정 장비 이중 계상 방지)', () => {
    const rows = toRows(serverSlots(), (id) => id === 0);
    expect(rows).toHaveLength(1);
    expect(rows[0].id).toBe(0);
    expect(rows[0].name).toBe('CUSTOM1');
  });

  it('필터가 없으면 13슬롯 그대로', () => {
    expect(toRows(serverSlots(), undefined)).toHaveLength(13);
    expect(toRows(serverSlots())).toHaveLength(13);
  });

  it('배열 인덱스는 슬롯 id 와 다르다 — 인덱스로 지목하면 안 되는 이유', () => {
    const rows = toRows(serverSlots());
    expect(rows[0].id).toBe(5);
    expect(rows[3].id).toBe(8);
  });

  it('mass≠0 인 슬롯만 장착으로 본다', () => {
    const slots = serverSlots();
    slots[8].mass_kg = 0.65;
    const rows = toRows(slots);
    expect(rows.filter((r) => r.enabled).map((r) => r.id)).toEqual([8]);
  });

  it('카탈로그 필드 없는 구버전 응답도 깨지지 않는다', () => {
    const bare = [{ id: 5, name: 'PTZ_CAM', mass_kg: 0, center_of_mass: { x_m: 0, y_m: 0, z_m: 0 } }] as unknown as PayloadSlot[];
    const rows = toRows(bare);
    expect(rows[0].label).toBe('PTZ_CAM');
    expect(rows[0].isCustom).toBe(false);
    expect(rows[0].defaultMass).toBe(0);
  });
});

describe('toggleRow', () => {
  it('PTZ 를 켜면 PTZ 만 켜지고 PTZ 기본값이 채워진다 (ptz→ouster 회귀 방지)', () => {
    const rows = toggleRow(toRows(serverSlots()), 5);
    const ptz = rows.find((r) => r.id === 5)!;
    expect(ptz.label).toBe('PTZ Camera');
    expect(ptz.enabled).toBe(true);
    expect(ptz.mass).toBeCloseTo(5.3);
    expect(ptz.x).toBeCloseTo(0.065);
    expect(ptz.z).toBeCloseTo(0.19);
    expect(rows.filter((r) => r.enabled).map((r) => r.id)).toEqual([5]);
    expect(rows.find((r) => r.id === 8)!.mass).toBe(0);
  });

  it('Ouster 를 켜면 Ouster 기본값', () => {
    const ouster = toggleRow(toRows(serverSlots()), 8).find((r) => r.id === 8)!;
    expect(ouster.mass).toBeCloseTo(0.65);
    expect(ouster.x).toBeCloseTo(-0.312);
  });

  it('이미 값이 있으면 기본값으로 덮지 않는다', () => {
    let rows = patchRow(toRows(serverSlots()), 5, { mass: 3, x: 0.2 });
    rows = toggleRow(rows, 5);
    const ptz = rows.find((r) => r.id === 5)!;
    expect(ptz.mass).toBe(3);
    expect(ptz.x).toBeCloseTo(0.2);
  });

  it('끄면 값은 남기고 enabled 만 내린다', () => {
    let rows = toggleRow(toRows(serverSlots()), 5);
    rows = toggleRow(rows, 5);
    const ptz = rows.find((r) => r.id === 5)!;
    expect(ptz.enabled).toBe(false);
    expect(ptz.mass).toBeCloseTo(5.3);
  });
});

describe('nudgeRow / clamp', () => {
  it('지목한 슬롯의 그 축만 움직인다', () => {
    const rows = nudgeRow(toRows(serverSlots()), 8, 'x', 0.01);
    expect(rows.find((r) => r.id === 8)!.x).toBeCloseTo(0.01);
    expect(rows.find((r) => r.id === 5)!.x).toBe(0);
  });
  it('축 한계를 넘지 않는다', () => {
    const rows = nudgeRow(toRows(serverSlots()), 5, 'y', 99);
    expect(rows.find((r) => r.id === 5)!.y).toBeCloseTo(0.3);
  });
  it('clamp — 로봇 422 거절 범위를 앱에서 먼저 막는다', () => {
    expect(clampMass(999)).toBe(20);
    expect(clampMass(-999)).toBe(0);
    expect(clampMass(NaN)).toBe(0);
    expect(clampAxis(-9, 'z')).toBeCloseTo(-0.4);
  });
});

describe('exceedsTotal', () => {
  it('상한 이하는 통과, 넘으면 걸린다 — 슬롯별로는 다 합법이어도 합계로 거절될 조합', () => {
    applyServerLimits(undefined);
    expect(exceedsTotal(20)).toBe(false);
    expect(exceedsTotal(20.0000005)).toBe(false);
    expect(exceedsTotal(20.01)).toBe(true);
    expect(exceedsTotal(0)).toBe(false);
  });
  it('로봇이 준 상한을 따른다 — 고하중 기체에서 합법 조작을 앱이 막으면 안 된다', () => {
    applyServerLimits({ mass_total_max_kg: 40 });
    expect(exceedsTotal(35)).toBe(false);
    applyServerLimits(undefined);
    expect(exceedsTotal(35)).toBe(true);
  });
});

describe('combine', () => {
  it('장착 슬롯만 질량가중 평균', () => {
    let rows = toRows(serverSlots());
    rows = patchRow(rows, 5, { enabled: true, mass: 3, x: 0.2, y: 0, z: 0 });
    rows = patchRow(rows, 8, { enabled: true, mass: 1, x: -0.2, y: 0, z: 0 });
    rows = patchRow(rows, 0, { enabled: false, mass: 100, x: 0.5, y: 0, z: 0 });
    const t = combine(rows);
    expect(t.mass).toBeCloseTo(4);
    expect(t.x).toBeCloseTo((3 * 0.2 + 1 * -0.2) / 4);
  });
  it('질량 0 이면 CoM 은 0', () => {
    expect(combine(toRows(serverSlots()))).toEqual({ mass: 0, x: 0, y: 0, z: 0 });
  });
});

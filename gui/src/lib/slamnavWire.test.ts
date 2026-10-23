import { describe, it, expect } from 'vitest';
import {
  CLOUD_HEADER, CLOUD_MAGIC, CloudAssembler, MAP_MAGIC, MAP_POINT_BUDGET, deriveSlamUi, intensityColors,
  mapStride, parseCloudFrame, parseMapCloud, parseSlamnavState, utf8,
} from './slamnavWire';

const SAMPLE = {
  v: 1, t: 1760000000.123, mode: 'DRIVE', boot: 'AUTO_BOOT_COMPLETE', busy: false,
  flags: { connected: true, mapLoaded: true, slam: false, loc: true, sim: false, recording: false, quickAnnotation: false,
    quickNoise: false, tasking: false, scheduleRunning: false, recovery: false, autoInit: true },
  map: { dir: '/home/rbq/maps_3d/2026_x', name: '2026_x', version: 3, topoVersion: 7, pathVersion: 2, points: 812345 },
  pose: { x: 1, y: 2, z: 0.4, roll: 0, pitch: 0, yaw: 1.5 },
  loc: { state: 'good', ie: 0.05, ir: 0.9 },
  robot: { link: 'good', battery: 'good', gait: 2, extJoy: true, docking: 0, percentage: 80, voltage: 52,
    vel: [0.1, 0, 0], cmd: [0, 0, 0], size: { x: [-0.65, 0.35], y: [-0.25, 0.25], z: [-0.47, 0.2] } },
  ctrl: { state: 'stop', fsm: 'AUTO_FSM_COMPLETE', goal: '', obs: 'none', zone: '', localPath: [[1, 2, 0, 0.5]],
    targets: { nn: [0, 0, 0] } },
  task: { phase: 'PHASE_READY', fsm: 'TASK_READY', nodes: ['N_1'], odo: 12.3, files: [] },
  schedule: { entries: ['08:30 task_a.json'], fsm: '' },
  info: { slam: '…' },
};

describe('parseSlamnavState', () => {
  it('JSON 문자열과 UTF-8 바이트(slam-state DC) 모두 읽는다', () => {
    const json = JSON.stringify(SAMPLE);
    const a = parseSlamnavState(json)!;
    const b = parseSlamnavState(new TextEncoder().encode(json).buffer as ArrayBuffer)!;
    expect(a).toEqual(b);
    expect(a.map.version).toBe(3);
    expect(a.map.topoVersion).toBe(7);
    expect(a.pose.yaw).toBe(1.5);
    expect(a.flags.loc).toBe(true);
    expect(a.ctrl.localPath).toEqual([[1, 2, 0, 0.5]]);
    expect(a.schedule.entries).toEqual(['08:30 task_a.json']);
  });

  it('빠진 필드는 기본값, 모르는 필드는 버린다', () => {
    const s = parseSlamnavState('{"v":1,"mode":"ANNOTATION"}')!;
    expect(s.mode).toBe('ANNOTATION');
    expect(s.flags.slam).toBe(false);
    expect(s.ctrl.state).toBe('stop');
    expect(s.robot.size.x).toEqual([-0.65, 0.35]);
    expect((s as unknown as Record<string, unknown>).info).toBeUndefined();
  });

  it('JSON 이 아니면 null', () => {
    expect(parseSlamnavState('not json')).toBeNull();
    expect(parseSlamnavState('42')).toBeNull();
  });

  it('한글 노드 이름 — utf8 디코드', () => {
    expect(utf8(new TextEncoder().encode('정문 GOAL'))).toBe('정문 GOAL');
  });
});

describe('deriveSlamUi — 버튼 하이라이트 대응', () => {
  const base = parseSlamnavState(JSON.stringify(SAMPLE))!;
  const withState = (patch: (s: typeof base) => void) => { const s = structuredClone(base); patch(s); return s; };

  it('mapBuilding=flags.slam, annotationMode=mode, quickAnnotation, locaStarted=flags.loc', () => {
    const s = withState((x) => { x.flags.slam = true; x.mode = 'ANNOTATION'; x.flags.quickAnnotation = true; });
    const u = deriveSlamUi(s);
    expect(u.mapBuilding).toBe(true);
    expect(u.annotationMode).toBe(true);
    expect(u.quickAnnotation).toBe(true);
    expect(u.locaStarted).toBe(true);
    expect(deriveSlamUi(base).annotationMode).toBe(false);
  });

  it('주행 방향은 마지막으로 누른 쪽만, 움직이는 동안만 켠다', () => {
    const moving = withState((x) => { x.ctrl.state = 'move'; });
    expect(deriveSlamUi(moving, { route: 'start' }).driveStarted).toBe(true);
    expect(deriveSlamUi(moving, { route: 'start' }).autoTravel).toBe(false);
    expect(deriveSlamUi(moving, { route: 'return' }).autoTravel).toBe(true);
    expect(deriveSlamUi(moving).driveStarted).toBe(false);
    expect(deriveSlamUi(moving).navActive).toBe(true);
    expect(deriveSlamUi(base, { route: 'start' }).driveStarted).toBe(false);
  });

  it('task·schedule 이 돌면 주행 중으로 본다', () => {
    expect(deriveSlamUi(withState((x) => { x.flags.tasking = true; })).navActive).toBe(true);
    expect(deriveSlamUi(withState((x) => { x.flags.scheduleRunning = true; })).navActive).toBe(true);
  });

  it('저장됨·다시 불러옴은 회신 결과에서만 온다', () => {
    expect(deriveSlamUi(base).mapSaved).toBe(false);
    expect(deriveSlamUi(base, { mapSaved: true, annotationSaved: true }).mapSaved).toBe(true);
    expect(deriveSlamUi(base, { annotationSaved: true }).annotationSaved).toBe(true);
  });
});

function cloudFrame(kind: 1 | 2 | 3, seq: number, part: number, parts: number, origin: [number, number, number], pts: number[][]) {
  const stride = kind === 1 ? 6 : 7;
  const buf = new ArrayBuffer(CLOUD_HEADER + pts.length * stride);
  const dv = new DataView(buf);
  dv.setUint16(0, CLOUD_MAGIC, true);
  dv.setUint8(2, kind);
  dv.setUint32(4, seq, true);
  dv.setUint16(8, part, true);
  dv.setUint16(10, parts, true);
  dv.setFloat64(12, 1760000000.5, true);
  dv.setFloat64(20, origin[0], true); dv.setFloat64(28, origin[1], true); dv.setFloat64(36, origin[2], true);
  dv.setUint32(44, pts.length, true);
  pts.forEach((p, i) => {
    const o = CLOUD_HEADER + i * stride;
    dv.setInt16(o, p[0], true); dv.setInt16(o + 2, p[1], true); dv.setInt16(o + 4, p[2], true);
    if (stride === 7) dv.setUint8(o + 6, p[3]);
  });
  return buf;
}

describe('slam-cloud 프레임', () => {
  it('SCAN: origin + cm/100, values 없음', () => {
    const p = parseCloudFrame(cloudFrame(1, 5, 0, 1, [10, -2, 0.5], [[100, -50, 0], [-1, 2, 30]]))!;
    expect(p.kind).toBe(1);
    expect(p.count).toBe(2);
    expect(p.values).toBeNull();
    expect(Array.from(p.positions).map((v) => +v.toFixed(3))).toEqual([11, -2.5, 0.5, 9.99, -1.98, 0.8]);
  });

  it('OBS/LIVE_MAP: 7 B/점, 마지막 바이트가 값', () => {
    const p = parseCloudFrame(cloudFrame(2, 1, 0, 1, [0, 0, 0], [[0, 0, 0, 200]]))!;
    expect(p.values?.[0]).toBe(200);
  });

  it('magic·kind·길이가 틀리면 버린다', () => {
    const ok = cloudFrame(1, 1, 0, 1, [0, 0, 0], [[1, 1, 1]]);
    const bad = ok.slice(0); new DataView(bad).setUint16(0, 0x1234, true);
    expect(parseCloudFrame(bad)).toBeNull();
    const kind = ok.slice(0); new DataView(kind).setUint8(2, 9);
    expect(parseCloudFrame(kind)).toBeNull();
    expect(parseCloudFrame(ok.slice(0, ok.byteLength - 1))).toBeNull();
  });

  it('여러 부분은 같은 seq 가 다 모여야 한 세트가 된다 — 순서가 뒤바뀌어도', () => {
    const asm = new CloudAssembler();
    const a = parseCloudFrame(cloudFrame(3, 7, 1, 2, [0, 0, 0], [[100, 0, 0, 9]]))!;
    const b = parseCloudFrame(cloudFrame(3, 7, 0, 2, [0, 0, 0], [[200, 0, 0, 8], [300, 0, 0, 7]]))!;
    expect(asm.push(a)).toBeNull();
    const set = asm.push(b)!;
    expect(set.count).toBe(3);
    expect(set.positions[0]).toBeCloseTo(2); expect(set.positions[3]).toBeCloseTo(3); expect(set.positions[6]).toBeCloseTo(1);
    expect(Array.from(set.values!)).toEqual([8, 7, 9]);
  });

  it('새 seq 가 오면 덜 모인 옛 seq 는 버리고, 늦게 온 옛 부분은 무시한다', () => {
    const asm = new CloudAssembler();
    asm.push(parseCloudFrame(cloudFrame(1, 1, 0, 2, [0, 0, 0], [[1, 1, 1]]))!);
    expect(asm.push(parseCloudFrame(cloudFrame(1, 2, 1, 2, [0, 0, 0], [[1, 1, 1]]))!)).toBeNull();
    expect(asm.push(parseCloudFrame(cloudFrame(1, 1, 1, 2, [0, 0, 0], [[1, 1, 1]]))!)).toBeNull();
    expect(asm.push(parseCloudFrame(cloudFrame(1, 2, 0, 2, [0, 0, 0], [[1, 1, 1]]))!)?.seq).toBe(2);
  });
});

describe('맵 클라우드(GET /api/map/cloud)', () => {
  function mapBuf(pts: [number, number, number, number, number][]) {
    const n = pts.length;
    const buf = new ArrayBuffer(16 + n * 12 + n + n * 3 + n);
    const dv = new DataView(buf);
    dv.setUint32(0, MAP_MAGIC, true); dv.setUint32(4, 9, true); dv.setUint32(8, n, true);
    pts.forEach(([x, y, z, r, l], i) => {
      dv.setFloat32(16 + i * 12, x, true); dv.setFloat32(20 + i * 12, y, true); dv.setFloat32(24 + i * 12, z, true);
      dv.setUint8(16 + n * 12 + i, r);
      dv.setUint8(16 + n * 12 + n + n * 3 + i, l);
    });
    return buf;
  }

  it('블록 순서: xyz → intensity → normal → label', () => {
    const m = parseMapCloud(mapBuf([[1, 2, 3, 255, 0], [4, 5, 6, 0, 1]]))!;
    expect(m.version).toBe(9);
    expect(m.count).toBe(2);
    expect(Array.from(m.positions)).toEqual([1, 2, 3, 4, 5, 6]);
    expect(Array.from(m.intensity)).toEqual([255, 0]);
    expect(Array.from(m.label)).toEqual([0, 1]);
  });

  it('magic 이 틀리거나 짧으면 null', () => {
    const b = mapBuf([[0, 0, 0, 0, 0]]);
    expect(parseMapCloud(b.slice(0, b.byteLength - 1))).toBeNull();
    new DataView(b).setUint32(0, 1, true);
    expect(parseMapCloud(b)).toBeNull();
  });

  it('Intensity 색 — viridis 양 끝, 노이즈 표시는 빨강', () => {
    const c = intensityColors(Uint8Array.from([0, 255, 128]), 3, Uint8Array.from([0, 0, 1]));
    expect(c[0]).toBeCloseTo(0.267); expect(c[2]).toBeCloseTo(0.329);
    expect(c[3]).toBeCloseTo(0.993); expect(c[4]).toBeCloseTo(0.906);
    expect([c[6], c[7], c[8]]).toEqual([1, expect.closeTo(0.1), expect.closeTo(0.1)]);
  });

  it('점 예산 — 넘으면 stride 로 솎는다', () => {
    expect(mapStride(1000)).toBe(1);
    expect(mapStride(MAP_POINT_BUDGET)).toBe(1);
    expect(mapStride(MAP_POINT_BUDGET + 1)).toBe(2);
    expect(mapStride(812345)).toBe(3);
  });
});

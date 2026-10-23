import type { CloudSet, MapCloud, SlamnavState } from './slamnavWire';

export type DemoTopoNode = { id: string; name: string; type: string; info: string; linked: string[]; pos: [number, number, number]; yaw: number };

const W = 20, H = 12;
const LOOP: [number, number][] = [[-7, -3.5], [-2, -4], [3, -4], [7, -3], [7, 3], [2, 4], [-3, 4], [-7, 3]];

function buildMap(): MapCloud {
  const pts: number[] = [];
  const inten: number[] = [];
  const add = (x: number, y: number, z: number, r: number) => { pts.push(x, y, z); inten.push(r); };
  for (let z = 0; z <= 2.4; z += 0.1) {
    const r = Math.round(60 + 160 * (z / 2.4));
    for (let x = -W / 2; x <= W / 2; x += 0.05) { add(x, -H / 2, z, r); add(x, H / 2, z, r); }
    for (let y = -H / 2; y <= H / 2; y += 0.05) { add(-W / 2, y, z, r); add(W / 2, y, z, r); }
  }
  for (let x = -W / 2; x <= W / 2; x += 0.2) for (let y = -H / 2; y <= H / 2; y += 0.2) add(x, y, 0, 30 + ((x * 7 + y * 13) & 15));
  for (const [cx, cy] of [[0, 0], [-4, 0.5]]) {
    for (let z = 0; z <= 2.4; z += 0.1) for (let a = 0; a < Math.PI * 2; a += 0.15) add(cx + 0.3 * Math.cos(a), cy + 0.3 * Math.sin(a), z, 200);
  }
  const count = inten.length;
  return { version: 1, count, positions: new Float32Array(pts), intensity: Uint8Array.from(inten), label: new Uint8Array(count) };
}

function buildTopo(): DemoTopoNode[] {
  const nodes: DemoTopoNode[] = LOOP.map(([x, y], i) => ({
    id: `N_demo_${i}`, name: i === 0 ? 'BASE' : `R${i}`, type: i === 0 ? 'BASE' : i === 4 ? 'GOAL' : 'ROUTE', info: '',
    linked: [`N_demo_${(i + 1) % LOOP.length}`], pos: [x, y, 0], yaw: 0,
  }));
  return nodes;
}

function sampleLoop(): [number, number, number][] {
  const out: [number, number, number][] = [];
  for (let i = 0; i < LOOP.length; i++) {
    const [ax, ay] = LOOP[i], [bx, by] = LOOP[(i + 1) % LOOP.length];
    const n = Math.max(1, Math.round(Math.hypot(bx - ax, by - ay) / 0.1));
    for (let k = 0; k < n; k++) out.push([ax + ((bx - ax) * k) / n, ay + ((by - ay) * k) / n, 0]);
  }
  return out;
}

export type DemoSink = { state: (s: SlamnavState) => void; cloud: (c: CloudSet) => void; log: (text: string) => void };

class SlamnavDemo {
  private map = buildMap();
  private topo = buildTopo();
  private loop = sampleLoop();
  private timer: ReturnType<typeof setInterval> | null = null;
  private tick = 0;
  private idx = 0;
  private dir = 0;
  private st: SlamnavState = {
    v: 1, t: 0, mode: 'DRIVE', boot: 'AUTO_BOOT_COMPLETE', busy: false,
    flags: { connected: true, mapLoaded: true, slam: false, loc: true, sim: false, recording: false, quickAnnotation: false,
      quickNoise: false, tasking: false, scheduleRunning: false, recovery: false, autoInit: true },
    map: { dir: '/home/rbq/maps_3d/demo', name: 'demo', version: 1, topoVersion: 1, pathVersion: 1, points: this.map.count },
    pose: { x: 0, y: 0, z: 0.47, roll: 0, pitch: 0, yaw: 0 },
    loc: { state: 'good', ie: 0.04, ir: 0.93 },
    robot: { link: 'good', battery: 'good', gait: 2, extJoy: false, docking: 0, vel: [0, 0, 0], cmd: [0, 0, 0],
      size: { x: [-0.65, 0.35], y: [-0.25, 0.25], z: [-0.47, 0.2] } },
    ctrl: { state: 'stop', fsm: 'AUTO_FSM_COMPLETE', goal: '', obs: 'none', zone: '', localPath: [] },
    task: { phase: 'PHASE_READY', fsm: 'TASK_READY', nodes: [], odo: 0 },
    schedule: { entries: [], fsm: '' },
  };

  start(sink: DemoSink) {
    this.stop();
    this.place();
    sink.state(this.snapshot());
    this.timer = setInterval(() => {
      this.tick++;
      if (this.dir !== 0) { this.idx = (this.idx + this.dir + this.loop.length) % this.loop.length; this.place(); }
      sink.state(this.snapshot());
      if (this.tick % 2 === 0) sink.cloud(this.scan());
    }, 100);
  }

  stop() { if (this.timer) { clearInterval(this.timer); this.timer = null; } }

  private place() {
    const [x, y] = this.loop[this.idx];
    const [nx, ny] = this.loop[(this.idx + (this.dir < 0 ? -1 : 1) + this.loop.length) % this.loop.length];
    this.st.pose = { ...this.st.pose, x, y, yaw: Math.atan2(ny - y, nx - x) };
    this.st.robot.vel = [this.dir !== 0 ? 0.5 : 0, 0, 0];
    this.st.ctrl.localPath = this.dir === 0 ? [] : Array.from({ length: 20 }, (_, k) => {
      const p = this.loop[(this.idx + this.dir * k + this.loop.length * 4) % this.loop.length];
      return [p[0], p[1], 0, 0] as [number, number, number, number];
    });
  }

  private snapshot(): SlamnavState {
    return { ...this.st, t: Date.now() / 1000, flags: { ...this.st.flags }, map: { ...this.st.map }, ctrl: { ...this.st.ctrl } };
  }

  private scan(): CloudSet {
    const { x, y } = this.st.pose;
    const src = this.map.positions;
    const out: number[] = [];
    for (let i = 0; i < this.map.count; i += 7) {
      const px = src[i * 3], py = src[i * 3 + 1], pz = src[i * 3 + 2];
      if (pz < 0.05) continue;
      if ((px - x) ** 2 + (py - y) ** 2 < 64) out.push(px, py, pz);
    }
    return { kind: 1, seq: this.tick, positions: new Float32Array(out), values: null, count: out.length / 3 };
  }

  mapCloud(stride: number): MapCloud {
    if (stride <= 1) return this.map;
    const n = Math.floor(this.map.count / stride);
    const positions = new Float32Array(n * 3), intensity = new Uint8Array(n), label = new Uint8Array(n);
    for (let i = 0; i < n; i++) {
      positions.set(this.map.positions.subarray(i * stride * 3, i * stride * 3 + 3), i * 3);
      intensity[i] = this.map.intensity[i * stride];
    }
    return { version: this.st.map.version, count: n, positions, intensity, label };
  }

  topoJson() { return { version: this.st.map.topoVersion, nodes: this.topo }; }

  pathJson() {
    const pts = this.loop.filter((_, i) => i % 5 === 0).map(([x, y]) => [x, y, 0]);
    return { version: this.st.map.pathVersion, nodes: this.topo.map((n) => n.id), points: [...pts, pts[0]] };
  }

  command(path: string, body: Record<string, any>): { status: number; body: Record<string, unknown> } {
    const ok = { status: 200, body: { status: 'ok' } };
    const accepted = { status: 202, body: { status: 'accepted' } };
    const f = this.st.flags;
    switch (path) {
      case '/api/map/build': f.slam = true; return ok;
      case '/api/map/stop': f.slam = false; return ok;
      case '/api/map/save': return accepted;
      case '/api/map/reload': this.topo = buildTopo(); this.st.map.topoVersion++; f.loc = true; return accepted;
      case '/api/mode': this.st.mode = body.mode === 'ANNOTATION' ? 'ANNOTATION' : 'DRIVE'; if (this.st.mode === 'DRIVE') f.quickAnnotation = false; return ok;
      case '/api/annot/quick': f.quickAnnotation = !!body.on; return ok;
      case '/api/annot/clear': this.topo = []; this.st.map.topoVersion++; return ok;
      case '/api/annot/save': return ok;
      case '/api/route/start': this.dir = 1; this.st.ctrl.state = 'move'; return ok;
      case '/api/route/return': this.dir = -1; this.st.ctrl.state = 'move'; return ok;
      case '/api/task/start': this.dir = 1; this.st.ctrl.state = 'move'; f.tasking = true; return ok;
      case '/api/task/cancel': case '/api/schedule/stop': case '/api/drive/stop': case '/api/estop':
        this.dir = 0; this.st.ctrl.state = 'stop'; f.tasking = false; f.scheduleRunning = false; this.place(); return ok;
      case '/api/task/pause': this.dir = 0; this.st.ctrl.state = 'pause'; return ok;
      case '/api/task/resume': this.dir = 1; this.st.ctrl.state = 'move'; return ok;
      case '/api/loc/auto-init': f.loc = true; f.autoInit = true; return accepted;
      default: return ok;
    }
  }
}

export const slamnavDemo = new SlamnavDemo();

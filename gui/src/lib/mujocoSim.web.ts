import type { SimPose, MujocoSim } from './mujocoSim.types';

type MujocoFactory = (opts?: { wasmBinary?: Uint8Array }) => Promise<any>;

export const MUJOCO_SUPPORT = { available: true, reason: '' } as const;

const BASE = (globalThis as { __MUJOCO_BASE?: string }).__MUJOCO_BASE ?? '/mujoco';
const ROOT = '/w';

function quatToRpy(w: number, x: number, y: number, z: number): [number, number, number] {
  const sinr = 2 * (w * x + y * z);
  const cosr = 1 - 2 * (x * x + y * y);
  const sinp = 2 * (w * y - z * x);
  const siny = 2 * (w * z + x * y);
  const cosy = 1 - 2 * (y * y + z * z);
  return [
    Math.atan2(sinr, cosr),
    Math.abs(sinp) >= 1 ? Math.sign(sinp) * (Math.PI / 2) : Math.asin(sinp),
    Math.atan2(siny, cosy),
  ];
}

let gluePromise: Promise<MujocoFactory> | null = null;
function loadGlue(): Promise<MujocoFactory> {
  if (gluePromise) return gluePromise;
  gluePromise = new Promise<MujocoFactory>((resolve, reject) => {
    const w = window as unknown as { __mujocoFactory?: MujocoFactory };
    if (w.__mujocoFactory) return resolve(w.__mujocoFactory);
    const el = document.createElement('script');
    el.type = 'module';
    el.src = `${BASE}/loader.js`;
    el.onerror = () => reject(new Error(`${BASE}/loader.js 로드 실패`));
    window.addEventListener(
      'mujoco-glue-ready',
      () => (w.__mujocoFactory ? resolve(w.__mujocoFactory) : reject(new Error('글루가 팩토리를 안 노출'))),
      { once: true },
    );
    document.head.appendChild(el);
  });
  return gluePromise;
}

async function fetchBytes(url: string): Promise<Uint8Array> {
  const r = await fetch(url);
  if (!r.ok) throw new Error(`${url} → HTTP ${r.status}`);
  return new Uint8Array(await r.arrayBuffer());
}

export async function loadMujocoSim(
  entry = 'model/rbq_environment.xml',
  init?: { joints: number[]; height: number; pos?: [number, number, number]; yaw?: number },
  opts?: { interactive?: boolean },
): Promise<MujocoSim> {
  const [loadMujoco, wasmBinary, manifest] = await Promise.all([
    loadGlue(),
    fetchBytes(`${BASE}/mujoco.wasm`),
    fetch(`${BASE}/files.json`).then((r) => r.json() as Promise<string[]>),
  ]);
  const mj = await loadMujoco({ wasmBinary });

  const mkdirp = (dir: string) => {
    let cur = '';
    for (const seg of `${ROOT}/${dir}`.split('/')) {
      if (!seg || seg === '.') continue;
      cur += `/${seg}`;
      try {
        mj.FS.mkdir(cur);
      } catch {
      }
    }
  };
  mkdirp('');
  const files = await Promise.all(
    manifest.map(async (rel) => [rel, await fetchBytes(`${BASE}/${rel}`)] as const),
  );
  for (const [rel, bytes] of files) {
    const slash = rel.lastIndexOf('/');
    if (slash > 0) mkdirp(rel.slice(0, slash));
    try {
      mj.FS.writeFile(`${ROOT}/${rel}`, bytes);
    } catch (e) {
      throw new Error(`MEMFS 쓰기 실패 ${rel}: ${e instanceof Error ? e.message : e}`);
    }
  }

  const model = mj.MjModel.from_xml_path(`${ROOT}/${entry}`);
  let data = new mj.MjData(model);

  if (opts?.interactive !== false) {
    model.opt.timestep = 0.005;
    model.opt.integrator = 0;
    model.opt.cone = 1;
    model.opt.impratio = 10;
    model.opt.iterations = 30;
    model.opt.ls_iterations = 20;
    model.opt.tolerance = 1e-8;
  }
  const timestep = model.opt.timestep;
  const nu = model.nu;

  const JOINT_NAMES = [
    'joint0_HRR', 'joint1_HRP', 'joint2_HRK',
    'joint3_HLR', 'joint4_HLP', 'joint5_HLK',
    'joint6_FRR', 'joint7_FRP', 'joint8_FRK',
    'joint9_FLR', 'joint10_FLP', 'joint11_FLK',
  ];
  const NJ = JOINT_NAMES.length;
  const qAdr: number[] = [];
  const vAdr: number[] = [];
  for (const n of JOINT_NAMES) {
    const j = model.jnt(n);
    qAdr.push(j.qposadr);
    vAdr.push(j.dofadr);
  }
  const trunkId: number = model.body('base_link').id;
  const trunkJnt = model.jnt(model.body('base_link').jntadr ?? 0);
  const trunkQ: number = trunkJnt.qposadr;
  const trunkV: number = trunkJnt.dofadr;
  const FOOT_BODIES = ['RR_foot', 'RL_foot', 'FR_foot', 'FL_foot'];
  const footIds = FOOT_BODIES.map((n) => model.body(n).id as number);

  const applyInit = () => {
    if (!init) return;
    const q = data.qpos;
    const [x0, y0, z0] = init.pos ?? [0, 0, 0];
    const half = ((init.yaw ?? 0) * Math.PI) / 360;
    const put = (h: number) => {
      q[trunkQ] = x0;
      q[trunkQ + 1] = y0;
      q[trunkQ + 2] = h;
      q[trunkQ + 3] = Math.cos(half); q[trunkQ + 4] = 0; q[trunkQ + 5] = 0; q[trunkQ + 6] = Math.sin(half);
      for (let i = 0; i < NJ; i++) q[qAdr[i]] = init.joints[i] ?? 0;
      const v = data.qvel;
      for (let i = 0; i < v.length; i++) v[i] = 0;
      mj.mj_forward(model, data);
    };
    const PROBE = z0 + 1.0;
    put(PROBE);
    const xp = data.xpos;
    let minFoot = Infinity;
    for (const id of footIds) minFoot = Math.min(minFoot, xp[id * 3 + 2]);
    const FOOT_R = 0.032;
    put(PROBE - minFoot + z0 + FOOT_R);
    standHeight = q[trunkQ + 2];
  };
  let standHeight = 0;
  applyInit();

  const sim: MujocoSim & { debug?: () => unknown } = {
    timestep,
    nu,
    advance(dt) {
      const n = Math.min(Math.round(dt / timestep), Math.ceil(0.05 / timestep));
      for (let i = 0; i < n; i++) mj.mj_step(model, data);
      return n;
    },
    setCtrl(ctrl) {
      const c = data.ctrl;
      const n = Math.min(ctrl.length, nu);
      for (let i = 0; i < n; i++) c[i] = ctrl[i];
    },
    pose(): SimPose {
      const q = data.qpos;
      const joints = new Array<number>(NJ);
      for (let i = 0; i < NJ; i++) joints[i] = q[qAdr[i]];
      const xp = data.xpos, xq = data.xquat;
      return {
        joints,
        rpy: quatToRpy(xq[trunkId * 4], xq[trunkId * 4 + 1], xq[trunkId * 4 + 2], xq[trunkId * 4 + 3]),
        xyz: [xp[trunkId * 3], xp[trunkId * 3 + 1], xp[trunkId * 3 + 2]],
      };
    },
    jointVels() {
      const v = data.qvel;
      const out = new Array<number>(NJ);
      for (let i = 0; i < NJ; i++) out[i] = v[vAdr[i]];
      return out;
    },
    trunkState() {
      const xq = data.xquat;
      const v = data.qvel;
      return {
        quat: [xq[trunkId * 4], xq[trunkId * 4 + 1], xq[trunkId * 4 + 2], xq[trunkId * 4 + 3]] as [number, number, number, number],
        gyro: [v[trunkV + 3], v[trunkV + 4], v[trunkV + 5]] as [number, number, number],
      };
    },
    jointTorques() {
      const c = data.ctrl;
      return Array.from({ length: NJ }, (_, i) => c[i]);
    },
    trunkVel() {
      const v = data.qvel;
      return [v[trunkV], v[trunkV + 1], v[trunkV + 2]] as [number, number, number];
    },
    setInit(joints) {
      if (init) init.joints = joints;
    },
    setSpawn(pos, yaw) {
      if (init) { init.pos = pos; init.yaw = yaw; }
    },
    reset() {
      data.delete();
      data = new mj.MjData(model);
      applyInit();
    },
    dispose() {
      data.delete();
      model.delete();
    },
    debug: () => ({
      nq: model.nq, nv: model.nv, nu: model.nu, nbody: model.nbody,
      qAdr, vAdr, trunkId, trunkQ, timestep, standHeight,
      qpos: Array.from({ length: model.nq }, (_, i) => +data.qpos[i].toFixed(3)),
      ctrl: Array.from({ length: model.nu }, (_, i) => +data.ctrl[i].toFixed(1)),
    }),
  };
  (globalThis as { __mjsim?: unknown }).__mjsim = sim;
  return sim;
}

export type { SimPose, MujocoSim };

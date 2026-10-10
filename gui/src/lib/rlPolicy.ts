
export type PolicyMeta = {
  iteration: number;
  layers: { in: number; out: number }[];
  numObs: number;
  numActions: number;
  policyDt: number;
  actionScale: number;
  jointNames: string[];
  defaultAngles: number[];
  stiffness: { R: number; P: number; K: number };
  damping: { R: number; P: number; K: number };
  obsScales: { lin_vel: number; ang_vel: number; dof_pos: number; dof_vel: number };
  clipObs: number;
  clipActions: number;
  cmdRanges: { lin_vel_x: [number, number]; lin_vel_y: [number, number]; ang_vel_yaw: [number, number] };
};

type Layer = { w: Float32Array; b: Float32Array; in: number; out: number };

const NJ = 12;
const HISTORY = 7;
const LAGS = [0, 2, 4, 6];

const elu = (v: number) => (v >= 0 ? v : Math.exp(v) - 1);
const clamp = (v: number, a: number, b: number) => (v < a ? a : v > b ? b : v);

function gemm(l: Layer, x: Float32Array, y: Float32Array) {
  const { w, b, in: nin, out: nout } = l;
  for (let o = 0; o < nout; o++) {
    let acc = b[o];
    const row = o * nin;
    for (let i = 0; i < nin; i++) acc += w[row + i] * x[i];
    y[o] = acc;
  }
}

export class RlTrotPolicy {
  readonly meta: PolicyMeta;
  private layers: Layer[];
  private bufs: Float32Array[];
  private obs: Float32Array;
  private pos: Float32Array[] = [];
  private vel: Float32Array[] = [];
  private action1 = new Float32Array(NJ);
  private action2 = new Float32Array(NJ);
  private first = true;
  readonly kp: Float32Array;
  readonly kd: Float32Array;

  constructor(meta: PolicyMeta, weights: Float32Array) {
    this.meta = meta;
    this.layers = [];
    let off = 0;
    for (const { in: nin, out: nout } of meta.layers) {
      const w = weights.subarray(off, off + nout * nin);
      off += nout * nin;
      const b = weights.subarray(off, off + nout);
      off += nout;
      this.layers.push({ w, b, in: nin, out: nout });
    }
    if (off !== weights.length) throw new Error(`가중치 길이 불일치: ${off} vs ${weights.length}`);
    this.bufs = meta.layers.map((l) => new Float32Array(l.out));
    this.obs = new Float32Array(meta.numObs);
    for (let i = 0; i < HISTORY; i++) {
      this.pos.push(new Float32Array(NJ));
      this.vel.push(new Float32Array(NJ));
    }
    const { stiffness: s, damping: d } = meta;
    this.kp = new Float32Array(NJ);
    this.kd = new Float32Array(NJ);
    for (let i = 0; i < NJ; i++) {
      const k = i % 3 === 0 ? 'R' : i % 3 === 1 ? 'P' : 'K';
      this.kp[i] = s[k];
      this.kd[i] = d[k];
    }
  }

  static async load(base = '/mujoco/policy/rbq10_trot'): Promise<RlTrotPolicy> {
    const [meta, bin] = await Promise.all([
      fetch(`${base}.json`).then((r) => r.json() as Promise<PolicyMeta>),
      fetch(`${base}.bin`).then((r) => r.arrayBuffer()),
    ]);
    return new RlTrotPolicy(meta, new Float32Array(bin));
  }

  reset(jointPos: ArrayLike<number>) {
    for (let f = 0; f < HISTORY; f++) {
      for (let i = 0; i < NJ; i++) {
        this.pos[f][i] = jointPos[i];
        this.vel[f][i] = 0;
      }
    }
    this.action1.fill(0);
    this.action2.fill(0);
    this.first = true;
  }

  step(
    jointPos: ArrayLike<number>,
    jointVel: ArrayLike<number>,
    quat: [number, number, number, number],
    gyro: [number, number, number],
    cmd: [number, number, number],
  ): Float32Array {
    if (!this.first) {
      const p = this.pos.pop()!;
      const v = this.vel.pop()!;
      this.pos.unshift(p);
      this.vel.unshift(v);
      this.action2.set(this.action1);
    }
    this.first = false;
    for (let i = 0; i < NJ; i++) {
      this.pos[0][i] = jointPos[i];
      this.vel[0][i] = jointVel[i];
    }

    const m = this.meta;
    const sc = m.obsScales;
    const o = this.obs;
    let k = 0;
    for (let i = 0; i < 3; i++) o[k++] = gyro[i] * sc.ang_vel;
    const [qw, qx, qy, qz] = quat;
    o[k++] = -2 * (qx * qz - qw * qy);
    o[k++] = -2 * (qy * qz + qw * qx);
    o[k++] = -(1 - 2 * (qx * qx + qy * qy));
    o[k++] = cmd[0] * sc.lin_vel;
    o[k++] = cmd[1] * sc.lin_vel;
    o[k++] = cmd[2] * sc.ang_vel;
    for (const lag of LAGS) {
      const src = this.pos[lag];
      for (let i = 0; i < NJ; i++) o[k++] = (src[i] - m.defaultAngles[i]) * sc.dof_pos;
    }
    for (const lag of LAGS) {
      const src = this.vel[lag];
      for (let i = 0; i < NJ; i++) o[k++] = src[i] * sc.dof_vel;
    }
    for (let i = 0; i < NJ; i++) o[k++] = this.action1[i];
    for (let i = 0; i < NJ; i++) o[k++] = this.action2[i];
    o[k++] = 0;
    for (let i = 0; i < k; i++) o[i] = clamp(o[i], -m.clipObs, m.clipObs);

    let x: Float32Array = o;
    for (let li = 0; li < this.layers.length; li++) {
      const y = this.bufs[li];
      gemm(this.layers[li], x, y);
      if (li < this.layers.length - 1) for (let i = 0; i < y.length; i++) y[i] = elu(y[i]);
      x = y;
    }

    const target = new Float32Array(NJ);
    for (let i = 0; i < NJ; i++) {
      const a = clamp(x[i], -m.clipActions, m.clipActions);
      this.action1[i] = a;
      target[i] = a * m.actionScale + m.defaultAngles[i];
    }
    return target;
  }

  lastObs(): Float32Array {
    return this.obs;
  }
}

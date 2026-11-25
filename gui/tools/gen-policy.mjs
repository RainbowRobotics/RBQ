import { readFile, writeFile, mkdir } from 'node:fs/promises';
import { join } from 'node:path';

function reader(buf) {
  let o = 0;
  const varint = () => {
    let r = 0n;
    let shift = 0n;
    for (;;) {
      const b = buf[o++];
      r |= BigInt(b & 0x7f) << shift;
      if (!(b & 0x80)) break;
      shift += 7n;
    }
    return r;
  };
  return {
    get done() {
      return o >= buf.length;
    },
    field() {
      const key = Number(varint());
      const no = key >> 3;
      const wire = key & 7;
      if (wire === 0) return { no, wire, val: varint() };
      if (wire === 2) {
        const len = Number(varint());
        const val = buf.subarray(o, o + len);
        o += len;
        return { no, wire, val };
      }
      if (wire === 5) {
        o += 4;
        return { no, wire, val: null };
      }
      if (wire === 1) {
        o += 8;
        return { no, wire, val: null };
      }
      throw new Error(`지원 안 하는 wire type ${wire}`);
    },
  };
}

function readInitializers(graphBuf) {
  const out = {};
  const r = reader(graphBuf);
  while (!r.done) {
    const f = r.field();
    if (f.no !== 5 || f.wire !== 2) continue;
    const t = reader(f.val);
    const dims = [];
    let name = '';
    let raw = null;
    let dtype = 0;
    while (!t.done) {
      const g = t.field();
      if (g.no === 1 && g.wire === 0) dims.push(Number(g.val));
      else if (g.no === 1 && g.wire === 2) {
        throw new Error('packed int64 dims 미지원 — 정책 export 형식이 바뀌었다(gen-policy.mjs 갱신 필요)');
      } else if (g.no === 2 && g.wire === 0) dtype = Number(g.val);
      else if (g.no === 8 && g.wire === 2) name = g.val.toString('utf8');
      else if (g.no === 9 && g.wire === 2) raw = g.val;
    }
    if (!raw) continue;
    if (dtype !== 1) throw new Error(`${name}: FLOAT 아님(data_type=${dtype})`);
    out[name] = { dims, data: new Float32Array(raw.buffer.slice(raw.byteOffset, raw.byteOffset + raw.byteLength)) };
  }
  return out;
}

export async function buildPolicy(policyDir) {
  const onnx = await readFile(join(policyDir, 'policy.onnx'));
  const info = JSON.parse(await readFile(join(policyDir, 'info.json'), 'utf8'));

  let graphBuf = null;
  const r = reader(onnx);
  while (!r.done) {
    const f = r.field();
    if (f.no === 7 && f.wire === 2) {
      graphBuf = f.val;
      break;
    }
  }
  if (!graphBuf) throw new Error('ONNX graph 를 못 찾음');

  const init = readInitializers(graphBuf);
  const LAYERS = ['0', '2', '4', '6'];
  const layers = [];
  const chunks = [];
  for (const l of LAYERS) {
    const w = init[`${l}.weight`];
    const b = init[`${l}.bias`];
    if (!w || !b) throw new Error(`${l}.weight/bias 없음`);
    layers.push({ out: w.dims[0], in: w.dims[1] });
    chunks.push(w.data, b.data);
  }
  const total = chunks.reduce((n, c) => n + c.length, 0);
  const flat = new Float32Array(total);
  let off = 0;
  for (const c of chunks) {
    flat.set(c, off);
    off += c.length;
  }

  const cfg = info.config_info;
  const ja = cfg.init_state.default_joint_angles;
  const meta = {
    source: 'resources/policy/rbq10_trot (ONNX → f32, tools/gen-policy.mjs)',
    iteration: info.export_info.iteration,
    layers,
    activation: 'elu',
    numObs: cfg.env.num_observations,
    numActions: cfg.env.num_actions,
    policyDt: cfg.control.policy_dt,
    actionScale: cfg.control.action_scale,
    jointNames: Object.keys(ja),
    defaultAngles: Object.values(ja),
    stiffness: cfg.control.stiffness,
    damping: cfg.control.damping,
    obsScales: cfg.normalization.obs_scales,
    clipObs: cfg.normalization.clip_observations,
    clipActions: cfg.normalization.clip_actions,
    cmdRanges: cfg.commands.ranges,
  };
  if (meta.layers[0].in !== meta.numObs || meta.layers.at(-1).out !== meta.numActions) {
    throw new Error('레이어 크기가 info.json 과 안 맞음');
  }
  return { meta, bin: Buffer.from(flat.buffer) };
}

export async function writePolicy(outDir, policyDir) {
  const { meta, bin } = await buildPolicy(policyDir);
  const dir = join(outDir, 'policy');
  await mkdir(dir, { recursive: true });
  await writeFile(join(dir, 'rbq10_trot.bin'), bin);
  await writeFile(join(dir, 'rbq10_trot.json'), JSON.stringify(meta));
  return { meta, bytes: bin.length };
}


export const FLOOR_Z = -4;

const f = (n) => Number(Number(n).toFixed(4));
const RAD = Math.PI / 180;

function rgba(hex) {
  const n = parseInt(hex.slice(1), 16);
  return `${f(((n >> 16) & 255) / 255)} ${f(((n >> 8) & 255) / 255)} ${f((n & 255) / 255)} 1`;
}

export function buildLevelMjcf(level, colors) {
  const names = ['floor'];
  const geoms = [];
  const shapes = [];
  const used = new Set();
  level.boxes.forEach((b, i) => {
    const nm = `b${i}`;
    names.push(nm);
    used.add(b.m);
    const roll = b.roll || 0, pitch = b.pitch || 0, yaw = b.yaw || 0;
    const eul = roll || pitch || yaw ? ` euler="${f(roll * RAD)} ${f(pitch * RAD)} ${f(yaw * RAD)}"` : '';
    geoms.push(`        <geom name="${nm}" type="box" material="${b.m}" pos="${f(b.x)} ${f(b.y)} ${f(b.z)}" size="${f(b.hx)} ${f(b.hy)} ${f(b.hz)}"${eul}/>`);
    shapes.push({ t: 'b', p: [f(b.x), f(b.y), f(b.z)], s: [f(b.hx), f(b.hy), f(b.hz)], m: b.m, e: f(pitch), ...(yaw ? { yaw: f(yaw) } : {}), ...(roll ? { roll: f(roll) } : {}) });
  });

  const FEET = ['RR', 'RL', 'FR', 'FL'];
  const pairs = names.flatMap((n) => FEET.map((ft) => `        <pair geom1="${ft}" geom2="${n}"/>`));
  const mats = [...used].map((m) => `        <material name="${m}" rgba="${rgba(colors[m] ?? '#8a8f99')}"/>`);

  const xml = `<?xml version="1.0" encoding="utf-8"?>
<!-- 자동 생성 (tools/sim-level-mjcf.mjs ← src/lib/sim/levels.json) — 직접 고치지 말 것. ${level.id} ${level.name ?? ''} -->
<mujoco model="rbq level ${level.id}">
    <include file="rbq/rbq.xml"/>
    <statistic center="0 0 0.1" extent="0.8"/>
    <visual>
        <headlight diffuse="0.65 0.65 0.65" ambient="0.32 0.32 0.32" specular="0 0 0"/>
    </visual>
    <asset>
        <material name="ground" rgba="0.17 0.2 0.27 1"/>
${mats.join('\n')}
    </asset>
    <worldbody>
        <light pos="0 0 8" dir="0 0 -1" directional="true"/>
        <!-- 허공 바닥 — 코스에서 떨어진 로봇을 받기만 한다(앱이 이탈로 판정해 복귀시킨다) -->
        <geom name="floor" type="plane" material="ground" size="0 0 0.1" pos="0 0 ${FLOOR_Z}"/>
${geoms.join('\n')}
    </worldbody>
    <contact>
${pairs.join('\n')}
    </contact>
</mujoco>
`;
  return { xml, shapes };
}

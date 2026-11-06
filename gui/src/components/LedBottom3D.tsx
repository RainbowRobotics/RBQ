import { useMemo, useRef, type RefObject } from 'react';
import { Color, DoubleSide, Matrix4, Object3D, PlaneGeometry, Quaternion, ShaderMaterial, Vector3, type Group, type Mesh, type MeshBasicMaterial, type SpotLight } from 'three';
import { useFrame } from '@/lib/r3fCanvas';
import { useLedBottom } from '@/store/ledBottom';
import { LED_BODY_POS, LED_CONE_DEG, LED_REF_HEIGHT, LED_SIDES, LED_TILT_DEG, ledShownAt, ledDisplay, type LedSide } from '@/lib/ledBottom';

const APP_POS: Record<LedSide, Vector3> = {
  right: new Vector3(LED_BODY_POS.right[0], LED_BODY_POS.right[2] - 0.001, -LED_BODY_POS.right[1]),
  left: new Vector3(LED_BODY_POS.left[0], LED_BODY_POS.left[2] - 0.001, -LED_BODY_POS.left[1]),
};
const TILT = (LED_TILT_DEG * Math.PI) / 180;
const APP_DIR: Record<LedSide, Vector3> = {
  right: new Vector3(0, -Math.cos(TILT), Math.sin(TILT)),
  left: new Vector3(0, -Math.cos(TILT), -Math.sin(TILT)),
};
const UNLIT = new Color(0.15, 0.15, 0.15);
const FLOOR_SIZE = 3.2;
const SPOT_ANGLE = ((LED_CONE_DEG + 3) * Math.PI) / 180;
const SPOT_CD = 2;

function shown(side: LedSide, now: number) {
  const st = useLedBottom.getState();
  const rgb = ledShownAt(st.live, st.track[side], side, now);
  return rgb ? ledDisplay(rgb) : null;
}

export function LedBottomBody({ hidden }: { hidden: boolean }) {
  const mats = useRef<Partial<Record<LedSide, MeshBasicMaterial | null>>>({});
  const spots = useRef<Partial<Record<LedSide, SpotLight | null>>>({});
  const targets = useMemo(() => Object.fromEntries(LED_SIDES.map((sd) => {
    const o = new Object3D();
    o.position.copy(APP_POS[sd]).add(APP_DIR[sd]);
    return [sd, o];
  })) as Record<LedSide, Object3D>, []);
  useFrame(() => {
    const now = Date.now();
    for (const sd of LED_SIDES) {
      const d = hidden ? null : shown(sd, now);
      const m = mats.current[sd];
      if (m) { if (d) m.color.setRGB(d.color[0], d.color[1], d.color[2]); else m.color.copy(UNLIT); }
      const l = spots.current[sd];
      if (l) { if (d) l.color.setRGB(d.color[0], d.color[1], d.color[2]); l.intensity = d ? SPOT_CD * d.k : 0; }
    }
  });
  return (
    <>
      {LED_SIDES.map((sd) => (
        <group key={sd}>
          <mesh position={APP_POS[sd]} rotation-x={Math.PI / 2}>
            <circleGeometry args={[0.015, 20]} />
            <meshBasicMaterial ref={(m) => { mats.current[sd] = m; }} color={UNLIT} side={DoubleSide} toneMapped={false} />
          </mesh>
          <primitive object={targets[sd]} />
          <spotLight ref={(l) => { spots.current[sd] = l; }} position={APP_POS[sd]} target={targets[sd]}
            angle={SPOT_ANGLE} penumbra={0.15} decay={2} distance={0} intensity={0} />
        </group>
      ))}
    </>
  );
}

const VERT = `
uniform vec3 uOff;
varying vec3 vP;
void main() {
  vP = position + uOff;
  gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
}`;
const FRAG = `
uniform vec3 uP[2];
uniform vec3 uD[2];
uniform vec3 uC[2];
uniform float uH2;
uniform float uCos;
uniform float uAlpha;
uniform float uRingA;
uniform float uDarkA;
varying vec3 vP;
void main() {
  vec3 light = vec3(0.0);
  float sum = 0.0;
  vec3 ringRgb = vec3(0.0);
  float ringA = 0.0;
  float darkA = 0.0;
  for (int i = 0; i < 2; i++) {
    float k = max(uC[i].r, max(uC[i].g, uC[i].b));
    if (k < 0.001) continue;
    vec3 v = vP - uP[i];
    float d2 = max(dot(v, v), 1e-6);
    vec3 dir = v * inversesqrt(d2);
    float c = dot(dir, uD[i]);
    float edge = (c - uCos) / max(fwidth(c), 1e-6);
    float inside = clamp(edge + 0.5, 0.0, 1.0);
    float e = inside * pow(max(c, 0.0) * max(-dir.y, 0.0) * (uH2 / d2), 0.35);   // 원 안은 거의 고르게(가장자리 0.7), 테두리에서 딱 끊긴다
    light += uC[i] * e;
    sum += k * e;
    float r = (1.0 - smoothstep(0.0, 1.2, abs(edge - 1.0))) * uRingA * k;
    ringRgb += (uC[i] / k) * r;
    ringA += r;
    float sat = (k - min(uC[i].r, min(uC[i].g, uC[i].b))) / k;   // 흰색(채도 0)일수록 바깥 선을 진하게 — 색 있는 빛은 제 색 선으로 충분
    darkA += (1.0 - smoothstep(0.0, 0.8, abs(edge + 1.0))) * mix(uDarkA, 0.06, sat);
  }
  float m = max(light.r, max(light.g, light.b));
  float fa = min(sum, 1.6) * uAlpha;
  vec3 rgb = m > 1e-4 ? (light / m) * fa : vec3(0.0);
  float a = fa;
  float da = min(darkA, 1.0);
  rgb = rgb * (1.0 - da);
  a = da + a * (1.0 - da);
  float ra = min(ringA, 1.0);
  vec3 rc = ringA > 1e-4 ? ringRgb / ringA : vec3(0.0);
  rgb = rc * ra + rgb * (1.0 - ra);
  a = ra + a * (1.0 - ra);
  if (a < 0.01) discard;
  gl_FragColor = vec4(rgb / a, a);
}`;

const _p = new Vector3();
const _d = new Vector3();
const _q = new Quaternion();
const _qi = new Quaternion();
const _m = new Matrix4();

export function LedBottomFloor({ body, hidden }: { body: RefObject<Group | null>; hidden: boolean }) {
  const mesh = useRef<Mesh>(null);
  const geo = useMemo(() => new PlaneGeometry(FLOOR_SIZE, FLOOR_SIZE).rotateX(-Math.PI / 2), []);
  const mat = useMemo(() => new ShaderMaterial({
    vertexShader: VERT, fragmentShader: FRAG, transparent: true, depthWrite: false, side: DoubleSide,
    polygonOffset: true, polygonOffsetFactor: -2, polygonOffsetUnits: -2,
    uniforms: {
      uOff: { value: new Vector3() },
      uP: { value: [new Vector3(), new Vector3()] },
      uD: { value: [new Vector3(0, -1, 0), new Vector3(0, -1, 0)] },
      uC: { value: [new Vector3(), new Vector3()] },
      uH2: { value: LED_REF_HEIGHT * LED_REF_HEIGHT },
      uCos: { value: Math.cos((LED_CONE_DEG * Math.PI) / 180) },
      uAlpha: { value: 0.28 },
      uRingA: { value: 0.55 },
      uDarkA: { value: 0.3 },
    },
  }), []);
  useFrame(() => {
    const m = mesh.current;
    const ground = m?.parent;
    const b = body.current;
    if (!m || !ground || !b) return;
    const now = Date.now();
    const u = mat.uniforms;
    let any = false;
    b.updateWorldMatrix(true, false);
    ground.updateWorldMatrix(true, false);
    _m.copy(ground.matrixWorld).invert();
    ground.getWorldQuaternion(_qi).invert();
    b.getWorldQuaternion(_q);
    let cx = 0, cz = 0;
    LED_SIDES.forEach((sd, i) => {
      _p.copy(APP_POS[sd]).applyMatrix4(b.matrixWorld).applyMatrix4(_m);
      _d.copy(APP_DIR[sd]).applyQuaternion(_q).applyQuaternion(_qi);
      (u.uP.value[i] as Vector3).copy(_p);
      (u.uD.value[i] as Vector3).copy(_d);
      cx += _p.x / 2; cz += _p.z / 2;
      const d = hidden ? null : shown(sd, now);
      if (d) { (u.uC.value[i] as Vector3).set(d.color[0] * d.k, d.color[1] * d.k, d.color[2] * d.k); any = true; }
      else (u.uC.value[i] as Vector3).set(0, 0, 0);
    });
    m.position.set(cx, 0.002, cz);
    (u.uOff.value as Vector3).set(cx, 0.002, cz);
    m.visible = !hidden && any;
  });
  return <mesh ref={mesh} geometry={geo} material={mat} renderOrder={1} visible={false} />;
}

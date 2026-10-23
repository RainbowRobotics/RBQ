import { useEffect, useState } from 'react';
import { Asset } from 'expo-asset';
import * as FileSystem from 'expo-file-system/legacy';
import { Buffer } from 'buffer';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import type { Object3D } from 'three';

if (typeof navigator !== 'undefined' && (navigator as any).userAgent == null) {
  try {
    (navigator as any).userAgent = 'react-native';
  } catch {
  }
}

export type RobotMeshKey = 'trunk' | 'hip2' | 'hip3' | 'thigh' | 'calf';
export type RobotMeshes = Record<RobotMeshKey, Object3D>;
export type ArmMeshKey = 'base' | 'link1' | 'link2' | 'link3' | 'link4' | 'link5' | 'link6';
export type ArmMeshes = Record<ArmMeshKey, Object3D>;

const MODULES: Record<RobotMeshKey, number> = {
  trunk: require('@/assets/models/rbq/trunk.glb'),
  hip2: require('@/assets/models/rbq/hip2.glb'),
  hip3: require('@/assets/models/rbq/hip3.glb'),
  thigh: require('@/assets/models/rbq/thigh.glb'),
  calf: require('@/assets/models/rbq/calf.glb'),
};
const ARM_MODULES: Record<ArmMeshKey, number> = {
  base: require('@/assets/models/rb1/base.glb'),
  link1: require('@/assets/models/rb1/link1.glb'),
  link2: require('@/assets/models/rb1/link2.glb'),
  link3: require('@/assets/models/rb1/link3.glb'),
  link4: require('@/assets/models/rb1/link4.glb'),
  link5: require('@/assets/models/rb1/link5.glb'),
  link6: require('@/assets/models/rb1/link6.glb'),
};

let cache: RobotMeshes | null = null;
let inflight: Promise<RobotMeshes> | null = null;

let lastError: string | null = null;
const errListeners = new Set<() => void>();
function setLoadError(msg: string | null) {
  lastError = msg;
  errListeners.forEach((l) => l());
}
export function getRobotMeshError(): string | null {
  return lastError;
}

async function loadOne(key: RobotMeshKey): Promise<Object3D> {
  return loadModule(MODULES[key], key);
}

async function loadModule(mod: number, name: string): Promise<Object3D> {
  const asset = Asset.fromModule(mod);
  await asset.downloadAsync();
  const uri = asset.localUri ?? asset.uri;
  const b64 = await FileSystem.readAsStringAsync(uri, { encoding: FileSystem.EncodingType.Base64 });
  const nodeBuf = Buffer.from(b64, 'base64');
  const arrayBuffer = nodeBuf.buffer.slice(nodeBuf.byteOffset, nodeBuf.byteOffset + nodeBuf.byteLength);
  const loader = new GLTFLoader();
  const gltf: any = await new Promise((res, rej) => loader.parse(arrayBuffer, '', res, rej));
  const scene: Object3D = gltf.scene;
  scene.name = name;
  return scene;
}

export async function loadRobotMeshes(): Promise<RobotMeshes> {
  if (cache) return cache;
  if (inflight) return inflight;
  inflight = (async () => {
    try {
      const [trunk, hip2, hip3, thigh, calf] = await Promise.all([
        loadOne('trunk'),
        loadOne('hip2'),
        loadOne('hip3'),
        loadOne('thigh'),
        loadOne('calf'),
      ]);
      cache = { trunk, hip2, hip3, thigh, calf };
      setLoadError(null);
      return cache;
    } catch (e: any) {
      inflight = null;
      setLoadError(`3D 모델 로드 실패 — ${e?.message ?? String(e)}`);
      throw e;
    }
  })();
  return inflight;
}

let armCache: ArmMeshes | null = null;
let armInflight: Promise<ArmMeshes> | null = null;

export async function loadArmMeshes(): Promise<ArmMeshes> {
  if (armCache) return armCache;
  if (armInflight) return armInflight;
  armInflight = (async () => {
    const keys = Object.keys(ARM_MODULES) as ArmMeshKey[];
    const objs = await Promise.all(keys.map((k) => loadModule(ARM_MODULES[k], k)));
    for (const o of objs) {
      o.traverse((n: any) => {
        if (n.isMesh && n.material) {
          const mats = Array.isArray(n.material) ? n.material : [n.material];
          for (const m of mats) { m.color?.setHex(0x33383f); m.needsUpdate = true; }
        }
      });
    }
    armCache = Object.fromEntries(keys.map((k, i) => [k, objs[i]])) as ArmMeshes;
    return armCache;
  })();
  return armInflight;
}

export function useArmMeshes(enabled: boolean): ArmMeshes | null {
  const [meshes, setMeshes] = useState<ArmMeshes | null>(armCache);
  useEffect(() => {
    if (!enabled || meshes) return;
    let alive = true;
    loadArmMeshes()
      .then((m) => alive && setMeshes(m))
      .catch((e) => console.warn('[robotMeshes] arm load 실패:', e?.message ?? String(e)));
    return () => { alive = false; };
  }, [enabled, meshes]);
  return enabled ? meshes : null;
}

export type WheelMeshes = { calfLeft: Object3D; calfRight: Object3D; wheelLeft: Object3D; wheelRight: Object3D };
const WHEEL_MODULES: Record<keyof WheelMeshes, number> = {
  calfLeft: require('@/assets/models/rbq/calf_wheel_left.glb'),
  calfRight: require('@/assets/models/rbq/calf_wheel_right.glb'),
  wheelLeft: require('@/assets/models/rbq/wheel_left.glb'),
  wheelRight: require('@/assets/models/rbq/wheel_right.glb'),
};
let wheelCache: WheelMeshes | null = null;
let wheelInflight: Promise<WheelMeshes> | null = null;

export async function loadWheelMeshes(): Promise<WheelMeshes> {
  if (wheelCache) return wheelCache;
  if (wheelInflight) return wheelInflight;
  wheelInflight = (async () => {
    const keys = Object.keys(WHEEL_MODULES) as (keyof WheelMeshes)[];
    const objs = await Promise.all(keys.map((k) => loadModule(WHEEL_MODULES[k], k)));
    wheelCache = Object.fromEntries(keys.map((k, i) => [k, objs[i]])) as WheelMeshes;
    return wheelCache;
  })();
  return wheelInflight;
}

export function useWheelMeshes(enabled: boolean): WheelMeshes | null {
  const [meshes, setMeshes] = useState<WheelMeshes | null>(wheelCache);
  useEffect(() => {
    if (!enabled || meshes) return;
    let alive = true;
    loadWheelMeshes()
      .then((m) => alive && setMeshes(m))
      .catch((e) => console.warn('[robotMeshes] wheel load 실패:', e?.message ?? String(e)));
    return () => { alive = false; };
  }, [enabled, meshes]);
  return enabled ? meshes : null;
}

export type AttMeshKey = 'ptz' | 'mid360' | 'ouster';
export type AttMeshes = Partial<Record<AttMeshKey, Object3D>>;
const ATT_MODULES: Record<AttMeshKey, number> = {
  ptz: require('@/assets/models/att/ptz.glb'),
  mid360: require('@/assets/models/sensor/mid360.glb'),
  ouster: require('@/assets/models/sensor/ouster.glb'),
};
const attCache: AttMeshes = {};
const attInflight = new Map<AttMeshKey, Promise<Object3D>>();

export async function loadAttMesh(key: AttMeshKey): Promise<Object3D> {
  const hit = attCache[key];
  if (hit) return hit;
  let p = attInflight.get(key);
  if (!p) {
    p = loadModule(ATT_MODULES[key], key)
      .then((o) => { attCache[key] = o; attInflight.delete(key); return o; })
      .catch((e) => { attInflight.delete(key); throw e; });
    attInflight.set(key, p);
  }
  return p;
}

export function useAttMeshes(keys: AttMeshKey[]): AttMeshes {
  const want = keys.join(',');
  const [, bump] = useState(0);
  useEffect(() => {
    let alive = true;
    for (const k of want ? (want.split(',') as AttMeshKey[]) : []) {
      if (attCache[k]) continue;
      loadAttMesh(k)
        .then(() => { if (alive) bump((n) => n + 1); })
        .catch((e) => console.warn('[robotMeshes] att load 실패:', k, e?.message ?? String(e)));
    }
    return () => { alive = false; };
  }, [want]);
  const out: AttMeshes = {};
  for (const k of keys) if (attCache[k]) out[k] = attCache[k];
  return out;
}

export function useRobotMeshes(): RobotMeshes | null {
  const [meshes, setMeshes] = useState<RobotMeshes | null>(cache);
  useEffect(() => {
    if (meshes) return;
    let alive = true;
    loadRobotMeshes()
      .then((m) => alive && setMeshes(m))
      .catch((e) => console.warn('[robotMeshes] load 실패:', e?.message ?? String(e), '\n', e?.stack));
    return () => {
      alive = false;
    };
  }, [meshes]);
  return meshes;
}

export function useRobotMeshError(): string | null {
  const [err, setErr] = useState<string | null>(lastError);
  useEffect(() => {
    const l = () => setErr(lastError);
    errListeners.add(l);
    l();
    return () => { errListeners.delete(l); };
  }, []);
  return err;
}

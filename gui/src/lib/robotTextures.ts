import { useEffect, useState } from 'react';
import { TextureLoader, SRGBColorSpace, type Texture } from 'three';

export type RobotTexKey = 'trunk' | 'hip' | 'thighLeft' | 'thighRight' | 'calf';
export type RobotTextures = Record<RobotTexKey, Texture>;

const MODULES: Record<RobotTexKey, number> = {
  trunk: require('@/assets/models/rbq/trunk_skin.png'),
  hip: require('@/assets/models/rbq/hip_skin.png'),
  thighLeft: require('@/assets/models/rbq/thigh_left_skin.png'),
  thighRight: require('@/assets/models/rbq/thigh_right_skin.png'),
  calf: require('@/assets/models/rbq/calf_skin.png'),
};

let cache: RobotTextures | null = null;
let inflight: Promise<RobotTextures> | null = null;

async function loadOne(key: RobotTexKey): Promise<Texture> {
  const loader = new TextureLoader();
  const tex: Texture = await new Promise((res, rej) =>
    loader.load(MODULES[key] as unknown as string, res, undefined, rej),
  );
  tex.colorSpace = SRGBColorSpace;
  tex.repeat.y = -1;
  tex.offset.y = 1;
  tex.needsUpdate = true;
  return tex;
}

export async function loadRobotTextures(): Promise<RobotTextures> {
  if (cache) return cache;
  if (inflight) return inflight;
  inflight = (async () => {
    const [trunk, hip, thighLeft, thighRight, calf] = await Promise.all([
      loadOne('trunk'),
      loadOne('hip'),
      loadOne('thighLeft'),
      loadOne('thighRight'),
      loadOne('calf'),
    ]);
    cache = { trunk, hip, thighLeft, thighRight, calf };
    return cache;
  })();
  return inflight;
}

export function useRobotTextures(): RobotTextures | null {
  const [tex, setTex] = useState<RobotTextures | null>(cache);
  useEffect(() => {
    if (tex) return;
    let alive = true;
    loadRobotTextures()
      .then((t) => alive && setTex(t))
      .catch((e) => console.warn('[robotTextures] load 실패:', e?.message ?? String(e)));
    return () => {
      alive = false;
    };
  }, [tex]);
  return tex;
}

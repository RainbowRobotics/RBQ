import { useEffect, useMemo, useState } from 'react';
import { simEngine, type SimState, type ShapeFile } from '@/lib/simEngine';
import { detachAllForSim } from '@/lib/transports';
import { reconnectCurrent, currentTarget } from '@/lib/connectNow';
import { levelById, GATE_R, type Level } from '@/lib/simCourse';

const FLOOR_Z = -4;

function Terrain({ data, level, passed }: { data: ShapeFile; level?: Level; passed: number }) {
  const d2rad = Math.PI / 180;
  return (
    <group name="sim-terrain">
      <mesh rotation={[-Math.PI / 2, 0, 0]} position={[10, FLOOR_Z, 0]}>
        <planeGeometry args={[160, 160]} />
        <meshStandardMaterial color="#1c2431" roughness={1} />
      </mesh>
      {level && <Markers level={level} passed={passed} />}
      {data.shapes.map((sh, i) => {
        const color = data.colors[sh.m] ?? '#888';
        const pos: [number, number, number] = [sh.p[0], sh.p[2], -sh.p[1]];
        if (sh.t === 'c') {
          return (
            <mesh key={i} position={pos}>
              <cylinderGeometry args={[sh.s[0], sh.s[0], sh.s[1] * 2, 12]} />
              <meshStandardMaterial color={color} roughness={0.9} />
            </mesh>
          );
        }
        return (
          <mesh key={i} position={pos} rotation={[(sh.roll ?? 0) * d2rad, (sh.yaw ?? 0) * d2rad, -sh.e * d2rad, 'YZX']}>
            <boxGeometry args={[sh.s[0] * 2, sh.s[2] * 2, sh.s[1] * 2]} />
            <meshStandardMaterial color={color} roughness={0.95} />
          </mesh>
        );
      })}
    </group>
  );
}

function Markers({ level, passed }: { level: Level; passed: number }) {
  const d2rad = Math.PI / 180;
  const G = level.goal;
  const allPassed = passed >= level.gates.length;
  return (
    <group>
      {level.gates.map((g, i) => {
        const side = g.facing != null, next = i === passed, done = i < passed;
        const color = side ? '#4fd1c5' : '#f2c14e';
        const r = g.r ?? GATE_R;
        return (
          <group key={i} position={[g.x, g.z, -g.y]}>
            <mesh position={[0, 0.25, 0]}>
              <cylinderGeometry args={[r, r, 0.5, 28, 1, true]} />
              <meshBasicMaterial color={color} transparent opacity={done ? 0.04 : next ? 0.22 : 0.08} side={2} depthWrite={false} />
            </mesh>
            {side && !done && (
              <mesh position={[Math.cos(g.facing! * d2rad) * 0.4, 0.3, -Math.sin(g.facing! * d2rad) * 0.4]}
                rotation={[0, g.facing! * d2rad, -Math.PI / 2]}>
                <coneGeometry args={[0.14, 0.42, 10]} />
                <meshBasicMaterial color={color} />
              </mesh>
            )}
          </group>
        );
      })}
      <mesh position={[G.x, G.z + 0.02, -G.y]}>
        <boxGeometry args={[G.hx * 2, 0.04, G.hy * 2]} />
        <meshBasicMaterial color="#ff5d8f" transparent opacity={allPassed ? 0.85 : 0.35} />
      </mesh>
    </group>
  );
}

export function useSimScene(active: boolean) {
  const [sim, setSim] = useState<SimState | null>(null);

  useEffect(() => simEngine.subscribe(setSim), []);

  useEffect(() => {
    if (!active) return;
    const detached = detachAllForSim();
    simEngine.acquire();
    return () => {
      simEngine.release();
      if (detached && currentTarget()) reconnectCurrent();
    };
  }, [active]);

  const shapes = sim?.shapes ?? null;
  const level = sim ? levelById(sim.map) : undefined;
  const passed = sim?.course.passed ?? 0;
  const terrain = useMemo(() => (active && shapes ? <Terrain data={shapes} level={level} passed={passed} /> : undefined), [active, shapes, level, passed]);

  return {
    state: sim,
    terrain,
    refresh: () => simEngine.refresh(),
    reset: () => simEngine.reset(),
  };
}

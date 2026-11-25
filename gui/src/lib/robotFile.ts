import { useEffect, useState } from 'react';
import * as FileSystem from 'expo-file-system/legacy';
import { robotAuth } from './auth';
import { restBase, viaRemote } from './rest';
import { robotDownloadAsync } from './download';

const PREFIX = 'remote-';
const inUse = new Set<string>();

export type RobotFileSrc = { uri: string; headers?: Record<string, string> } | null;

export function useRobotFile(ip: string, direct: string | null, route: string | null) {
  const [local, setLocal] = useState<string | null>(null);
  const [failed, setFailed] = useState(false);
  const remote = !direct && !!route && viaRemote();
  useEffect(() => {
    setLocal(null);
    setFailed(false);
    if (!remote || !route) return;
    let alive = true;
    const ext = /\.(\w+)(?:$|[?&])/.exec(decodeURIComponent(route))?.[1] ?? 'bin';
    const name = PREFIX + route.replace(/[^\w.-]+/g, '_').slice(-80) + '.' + ext;
    const uri = (FileSystem.cacheDirectory || '') + name;
    inUse.add(name);
    (async () => {
      await purgeRemote();
      const info = await FileSystem.getInfoAsync(uri);
      if (!info.exists) {
        const part = uri + '.part';
        try {
          const r = await robotDownloadAsync(restBase(ip) + route, part, { headers: robotAuth() });
          if (r.status >= 300) throw new Error(`HTTP ${r.status}`);
          await FileSystem.moveAsync({ from: part, to: uri });
        } catch (e) {
          await FileSystem.deleteAsync(part, { idempotent: true }).catch(() => {});
          throw e;
        }
      }
      if (alive) setLocal(uri);
    })().catch(() => { if (alive) setFailed(true); });
    return () => { alive = false; inUse.delete(name); };
  }, [remote, ip, route]);
  const src: RobotFileSrc = direct ? { uri: direct, headers: robotAuth() } : local ? { uri: local } : null;
  return { src, loading: remote && !local && !failed, failed };
}

async function purgeRemote() {
  const dir = FileSystem.cacheDirectory;
  if (!dir) return;
  try {
    for (const n of await FileSystem.readDirectoryAsync(dir)) {
      if (n.startsWith(PREFIX) && !inUse.has(n)) await FileSystem.deleteAsync(dir + n, { idempotent: true });
    }
  } catch { }
}

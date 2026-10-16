import { useEffect, useState } from 'react';
import { connection } from './connection';
import { webrtcClient } from './webrtcClient';
import { wireBytesFromStats } from './connectionRoute';

const POLL_MS = 2000;

type Bytes = { rx: number; tx: number };

type StatsPeer = { stats?: () => Promise<Parameters<typeof wireBytesFromStats>[0]> };
async function bytesOf(peer: StatsPeer): Promise<Bytes | null> {
  try { return wireBytesFromStats((await peer.stats?.()) ?? []); } catch { return null; }
}

async function peerBytes(): Promise<Bytes> {
  const [a, b] = await Promise.all([bytesOf(connection), bytesOf(webrtcClient)]);
  return { rx: (a?.rx ?? 0) + (b?.rx ?? 0), tx: (a?.tx ?? 0) + (b?.tx ?? 0) };
}

export function useNetRate(enabled: boolean): number | null {
  const [rate, setRate] = useState<number | null>(null);
  useEffect(() => {
    if (!enabled) { setRate(null); return; }
    let dead = false;
    let prev: { rx: number; tx: number; at: number } | null = null;
    const sample = async () => {
      const b = await peerBytes();
      if (dead || !b) return;
      const cur = { rx: b.rx, tx: b.tx, at: Date.now() };
      if (prev && cur.at > prev.at && cur.rx >= prev.rx && cur.tx >= prev.tx) {
        const dt = (cur.at - prev.at) / 1000;
        setRate(((cur.rx - prev.rx) + (cur.tx - prev.tx)) / dt);

      }
      prev = cur;
    };
    void sample();
    const id = setInterval(() => { void sample(); }, POLL_MS);
    return () => { dead = true; clearInterval(id); };
  }, [enabled]);
  return enabled ? rate : null;
}

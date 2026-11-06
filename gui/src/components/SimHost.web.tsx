import { useEffect, useRef, useState } from 'react';
import { simEngine } from '@/lib/simEngine';

let hosts: number[] = [];
let nextId = 0;
const hostSubs = new Set<() => void>();
const notifyHosts = () => hostSubs.forEach((f) => f());

export function SimHost() {
  const ref = useRef<HTMLIFrameElement>(null);
  const [seq, setSeq] = useState(simEngine.reloadSeq);
  useEffect(() => simEngine.subscribe(() => setSeq(simEngine.reloadSeq)), []);
  const [id] = useState(() => ++nextId);
  const [active, setActive] = useState(false);
  useEffect(() => {
    const f = () => setActive(hosts[hosts.length - 1] === id);
    hosts.push(id);
    hostSubs.add(f);
    notifyHosts();
    return () => {
      hosts = hosts.filter((h) => h !== id);
      hostSubs.delete(f);
      notifyHosts();
    };
  }, [id]);

  useEffect(() => {
    const el = ref.current;
    if (!el) return;
    const onMsg = (e: MessageEvent) => {
      if (e.source !== el.contentWindow) return;
      simEngine.onMessage(String(e.data));
    };
    window.addEventListener('message', onMsg);
    const off = simEngine.attachHost((j) => el.contentWindow?.postMessage(j, '*'));
    return () => {
      window.removeEventListener('message', onMsg);
      off();
    };
  }, [seq, active]);

  if (!active) return null;
  return (
    <iframe
      key={seq}
      ref={ref}
      src="/mujoco/worker.html"
      title="rbq-sim-worker"
      style={{ position: 'absolute', width: 1, height: 1, opacity: 0, pointerEvents: 'none', border: 0 }}
    />
  );
}

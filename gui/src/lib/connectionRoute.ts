
export type Route = 'p2p' | 'relay' | 'unknown';

export type RelayProto = 'udp' | 'tcp' | 'tls' | null;

type StatLike = {
  type?: string;
  state?: string;
  nominated?: boolean;
  selected?: boolean;
  localCandidateId?: string;
  remoteCandidateId?: string;
  candidateType?: string;
  relayProtocol?: string;
  kind?: string;
  id?: string;
  bytesSent?: number;
  bytesReceived?: number;
  currentRoundTripTime?: number;
};

export async function readStats(pc: unknown): Promise<StatLike[]> {
  const p = pc as { getStats?: () => Promise<unknown> } | null;
  if (!p || typeof p.getStats !== 'function') return [];
  try {
    const r = await p.getStats() as { forEach?: (f: (v: StatLike) => void) => void };
    if (Array.isArray(r)) return r as StatLike[];
    const a: StatLike[] = [];
    if (typeof r?.forEach === 'function') r.forEach((v) => a.push(v));
    return a;
  } catch { return []; }
}

function selectedPair(stats: StatLike[]): StatLike | null {
  const pairs = stats.filter((s) => s.type === 'candidate-pair');
  return (
    pairs.find((p) => p.selected) ??
    pairs.find((p) => p.state === 'succeeded' && p.nominated) ??
    pairs.find((p) => p.state === 'succeeded') ??
    null
  );
}

export function routeFromStats(stats: StatLike[]): Route {
  return routeDetail(stats).route;
}

export function routeDetail(stats: StatLike[]): { route: Route; proto: RelayProto } {
  const pair = selectedPair(stats);
  if (!pair) return { route: 'unknown', proto: null };
  const byId = new Map(stats.filter((s) => s.id).map((s) => [s.id as string, s]));
  const local = pair.localCandidateId ? byId.get(pair.localCandidateId) : undefined;
  const remote = pair.remoteCandidateId ? byId.get(pair.remoteCandidateId) : undefined;
  if (!local && !remote) return { route: 'unknown', proto: null };
  const isRelay = local?.candidateType === 'relay' || remote?.candidateType === 'relay';
  if (!isRelay) return { route: 'p2p', proto: null };
  const raw = (local?.candidateType === 'relay' ? local.relayProtocol : remote?.relayProtocol) ?? '';
  const proto = raw === 'udp' || raw === 'tcp' || raw === 'tls' ? raw : null;
  return { route: 'relay', proto };
}

export function routeLabel(r: Route, proto: RelayProto = null): string {
  if (r === 'p2p') return 'P2P';
  if (r !== 'relay') return '';
  return proto ? `릴레이(${proto})` : '릴레이';
}

export function wireBytesFromStats(stats: StatLike[]): { rx: number; tx: number } | null {
  const num = (v: unknown) => Number(v ?? 0) || 0;
  const pair = selectedPair(stats);
  if (pair && (pair.bytesReceived != null || pair.bytesSent != null)) {
    return { rx: num(pair.bytesReceived), tx: num(pair.bytesSent) };
  }
  const tr = stats.filter((s) => s.type === 'transport');
  if (!tr.length) return null;
  return {
    rx: tr.reduce((a, b) => a + num(b.bytesReceived), 0),
    tx: tr.reduce((a, b) => a + num(b.bytesSent), 0),
  };
}

export type InboundVideo = {
  packetsReceived: number;
  packetsLost: number;
  framesDecoded: number;
  keyFramesDecoded: number;
  framesDropped: number;
  frameWidth: number;
  frameHeight: number;
  lossPct: number;
  framesPerSecond: number;
  bytesReceived: number;
  jitterBufferMs: number;
  nackCount: number;
};

export function inboundVideoFromStats(stats: StatLike[]): InboundVideo | null {
  const v = stats.find((s) => s.type === 'inbound-rtp' && (s as Record<string, unknown>).kind === 'video');
  if (!v) return null;
  const n = (k: string) => Number((v as Record<string, unknown>)[k] ?? 0) || 0;
  const recv = n('packetsReceived');
  const lost = n('packetsLost');
  const total = recv + lost;
  const emitted = n('jitterBufferEmittedCount');
  return {
    packetsReceived: recv,
    packetsLost: lost,
    framesDecoded: n('framesDecoded'),
    keyFramesDecoded: n('keyFramesDecoded'),
    framesDropped: n('framesDropped'),
    frameWidth: n('frameWidth'),
    frameHeight: n('frameHeight'),
    lossPct: total > 0 ? Math.round((lost / total) * 1000) / 10 : 0,
    framesPerSecond: n('framesPerSecond'),
    bytesReceived: n('bytesReceived'),
    jitterBufferMs: emitted > 0 ? Math.round(n('jitterBufferDelay') / emitted * 1000) : 0,
    nackCount: n('nackCount'),
  };
}

export function linkQualityPct(stats: StatLike[]): number | null {
  const ms = linkRttMs(stats);
  if (ms == null) return null;
  return ms < 150 ? 100 : ms < 300 ? 85 : ms < 500 ? 60 : ms < 1000 ? 40 : 15;
}

export function linkRttMs(stats: StatLike[]): number | null {
  const rtt = selectedPair(stats)?.currentRoundTripTime;
  return typeof rtt === 'number' && rtt >= 0 ? Math.round(rtt * 1000) : null;
}

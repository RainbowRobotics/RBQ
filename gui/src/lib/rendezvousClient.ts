
export interface RendezvousAnswer {
  sdp: string;
  features?: string[];
}

export interface RendezvousExchangeArgs {
  url: string;
  robot: string;
  service: 'motion' | 'vision';
  clientId: string;
  sdp: string;
  token?: string;
  ticket?: string;
  timeoutMs?: number;
}

export function rendezvousExchange(args: RendezvousExchangeArgs): Promise<RendezvousAnswer> {
  const { url, robot, service, clientId, sdp, token = '', ticket = '', timeoutMs = 8000 } = args;

  return new Promise<RendezvousAnswer>((resolve, reject) => {
    let done = false;
    let timer: any = null;
    let ws: WebSocket;

    const finish = (err: Error | null, ans?: RendezvousAnswer) => {
      if (done) return;
      done = true;
      if (timer) clearTimeout(timer);
      try { ws?.close(); } catch {}
      if (err) reject(err);
      else resolve(ans as RendezvousAnswer);
    };

    try {
      ws = new WebSocket(url);
    } catch (e: any) {
      reject(new Error(`랑데부 WS 생성 실패: ${e?.message ?? e}`));
      return;
    }

    timer = setTimeout(() => finish(new Error('랑데부 타임아웃')), timeoutMs);

    ws.onopen = () => {
      ws.send(JSON.stringify({ type: 'offer', robot, service, clientId, sdp, token, ticket }));
    };
    ws.onmessage = (ev: any) => {
      let m: any;
      try { m = JSON.parse(typeof ev.data === 'string' ? ev.data : ''); } catch { return; }
      if (m?.type === 'answer') {
        if (!m.sdp) { finish(new Error('랑데부 answer에 sdp 없음')); return; }
        finish(null, { sdp: String(m.sdp), features: m.features });
      } else if (m?.type === 'error') {
        finish(new Error(`랑데부 오류: ${m.reason ?? '알 수 없음'}`));
      }
    };
    ws.onerror = () => finish(new Error('랑데부 WS 오류'));
    ws.onclose = () => finish(new Error('랑데부 WS 닫힘 (answer 전)'));
  });
}

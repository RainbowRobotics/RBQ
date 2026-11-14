import { unzipSync } from 'fflate';
import { rest } from '@/lib/rest';
import { parsePcdBin, type PcdFrame } from '@/lib/heightmapCloud';

export type BbVideoClip = {
  skewMs: number; durationMs: number; buf?: ArrayBuffer;
  pending?: boolean;
  failed?: boolean;
};

export type BbLogLine = {
  epochMs: number;
  ts: string;
  process: string;
  level: string;
  msg: string;
};

export type BlackboxSession = {
  tickMs: number;
  frameCount: number;
  startEpochMs: number;
  cols: Map<string, number>;
  numCols: number;
  data: Float32Array;
  logs: BbLogLine[];
  sync: { warn: boolean; reason: string; retry?: boolean };
  video: { front?: BbVideoClip; rear?: BbVideoClip };
  pcd?: PcdFrame[];
  robot?: RobotOrigin;
  id?: { date: string; session: string };
};

export type RobotOrigin = { serial: string; via: 'meta' | 'filename' | 'connection' };

export function serialFromZipName(fileName: string): string | null {
  const m = /^blackbox-(.+)-\d{8}_\d{2}_\d{2}_\d{2}\.zip$/i.exec(fileName.replace(/^.*[/\\]/, ''));
  return m ? m[1] : null;
}

export function chan(s: BlackboxSession, frame: number, name: string): number {
  const col = s.cols.get(name);
  if (col === undefined || frame < 0 || frame >= s.frameCount) return 0;
  return s.data[frame * s.numCols + col];
}

export function chanOpt(s: BlackboxSession, frame: number, name: string): number | undefined {
  return s.cols.has(name) ? chan(s, frame, name) : undefined;
}

function parseLogEpoch(ts: string): number {
  const m = /^(\d{4})-(\d{2})-(\d{2})[ T](\d{2}):(\d{2}):(\d{2})(?:\.(\d{1,3}))?/.exec(ts);
  if (!m) return 0;
  return new Date(+m[1], +m[2] - 1, +m[3], +m[4], +m[5], +m[6], +(m[7] ?? '0').padEnd(3, '0')).getTime();
}

export async function loadBlackboxSession(ip: string, date: string, session: string): Promise<BlackboxSession> {
  const base = `${date}/${session}`;
  const havePlayer = typeof document !== 'undefined';
  const clip = (cam: string) => (havePlayer
    ? rest.blackboxBinary(ip, `${base}/${cam}.mp4`).catch(() => undefined)
    : Promise.resolve(undefined));
  let metaLost = false;
  const [metaTxt, dataTxt, logTxt, videoTxt, pcdBuf, front, rear, serial] = await Promise.all([
    rest.blackboxRaw(ip, `${base}/meta.json`).catch((e: any) => { metaLost = e?.status !== 404; return ''; }),
    rest.blackboxRaw(ip, `${base}/data.log`),
    rest.blackboxRaw(ip, `${base}/systemlog.log`).catch(() => ''),
    rest.blackboxRaw(ip, `${base}/video.json`).catch(() => ''),
    rest.blackboxBinary(ip, `${base}/pcd.bin`).catch(() => undefined),
    clip('front'), clip('rear'),
    rest.serialNumber(ip).then((r) => (r.serial_number || '').trim()).catch(() => ''),
  ]);
  return buildSession({ metaTxt, dataTxt, logTxt, videoTxt, pcdBuf, metaLost,
                       clips: { front, rear }, clipsAttempted: havePlayer, session, date,
                       robotFallback: serial ? { serial, via: 'connection' } : undefined });
}

export function buildVideo(videoTxt: string, startEpochMs: number | null,
                           clips?: Partial<Record<'front' | 'rear', ArrayBuffer | undefined>>,
                           clipsAttempted?: boolean): BlackboxSession['video'] {
  const video: BlackboxSession['video'] = {};
  if (startEpochMs === null) return video;
  try {
    const vj: any = JSON.parse(videoTxt);
    for (const cam of ['front', 'rear'] as const) {
      const c = vj?.[cam];
      if (c?.ok && typeof c.start_epoch_ms === 'number' && c.duration_ms > 0) {
        video[cam] = {
          skewMs: startEpochMs - c.start_epoch_ms,
          durationMs: c.duration_ms,
          buf: clips?.[cam],
          pending: clipsAttempted && !clips?.[cam],
        };
      }
    }
  } catch { }
  return video;
}

export async function fetchBlackboxVideo(ip: string, date: string, session: string,
                                         startEpochMs: number): Promise<BlackboxSession['video']> {
  const base = `${date}/${session}`;
  const havePlayer = typeof document !== 'undefined';
  const videoTxt = await rest.blackboxRaw(ip, `${base}/video.json`).catch(() => '');
  if (!videoTxt) return {};
  const clips = havePlayer ? await fetchBlackboxClips(ip, date, session) : {};
  return buildVideo(videoTxt, startEpochMs, clips, havePlayer);
}

export async function fetchBlackboxClips(ip: string, date: string, session: string):
    Promise<Partial<Record<'front' | 'rear', ArrayBuffer>>> {
  const base = `${date}/${session}`;
  const [front, rear] = await Promise.all([
    rest.blackboxBinary(ip, `${base}/front.mp4`).catch(() => undefined),
    rest.blackboxBinary(ip, `${base}/rear.mp4`).catch(() => undefined),
  ]);
  const out: Partial<Record<'front' | 'rear', ArrayBuffer>> = {};
  if (front) out.front = front;
  if (rear) out.rear = rear;
  return out;
}

export function buildSession({ metaTxt, dataTxt, logTxt, videoTxt, pcdBuf, metaLost, clips, clipsAttempted, session, date, robotFallback }: {
  metaTxt: string; dataTxt: string; logTxt: string; videoTxt: string; pcdBuf?: ArrayBuffer;
  metaLost?: boolean;
  clips?: Partial<Record<'front' | 'rear', ArrayBuffer | undefined>>;
  clipsAttempted?: boolean;
  session: string; date: string;
  robotFallback?: RobotOrigin;
}): BlackboxSession {

  const lines = dataTxt.split('\n');
  const header = (lines[0] ?? '').split('\t');
  const numCols = header.length;
  if (numCols < 2) throw new Error('data.log 헤더가 비어 있습니다');
  const cols = new Map<string, number>();
  header.forEach((name, i) => cols.set(name.trim(), i));
  const rows: number[][] = [];
  for (let i = 1; i < lines.length; i++) {
    const ln = lines[i];
    if (!ln) continue;
    const parts = ln.split('\t');
    if (parts.length < numCols) continue;
    const row = new Array<number>(numCols);
    for (let j = 0; j < numCols; j++) row[j] = Number(parts[j]) || 0;
    rows.push(row);
  }
  const frameCount = rows.length;
  if (frameCount === 0) throw new Error('data.log에 프레임이 없습니다');
  const data = new Float32Array(frameCount * numCols);
  rows.forEach((row, f) => data.set(row, f * numCols));

  let tickMs = 10;
  let startEpochMs = 0;
  try {
    const meta = JSON.parse(metaTxt);
    if (typeof meta.data_tick_ms === 'number' && meta.data_tick_ms > 0) tickMs = meta.data_tick_ms;
    if (typeof meta.data_start_epoch_ms === 'number') startEpochMs = meta.data_start_epoch_ms;
  } catch {}
  if (!startEpochMs) {
    const t = /^(\d{2})_(\d{2})_(\d{2})$/.exec(session);
    const d = /^(\d{4})(\d{2})(\d{2})$/.exec(date);
    if (t && d) {
      const end = new Date(+d[1], +d[2] - 1, +d[3], +t[1], +t[2], +t[3]).getTime();
      startEpochMs = end - frameCount * tickMs;
    }
  }

  let haveMeta = false; let triggerEpochMs = 0; let robot: RobotOrigin | undefined;
  try {
    const meta = JSON.parse(metaTxt);
    haveMeta = typeof meta.data_start_epoch_ms === 'number' && meta.data_start_epoch_ms > 0;
    if (typeof meta.trigger_epoch_ms === 'number') triggerEpochMs = meta.trigger_epoch_ms;
    const sn = typeof meta.robot_serial === 'string' ? meta.robot_serial.trim() : '';
    if (sn) robot = { serial: sn, via: 'meta' };
  } catch {}
  if (!robot && robotFallback?.serial) robot = robotFallback;
  const video = buildVideo(videoTxt, haveMeta ? startEpochMs : null, clips, clipsAttempted);
  let pcSkewMs = 0;
  try {
    const vj: any = JSON.parse(videoTxt);
    if (triggerEpochMs > 0 && typeof vj?.clip_end_epoch_ms === 'number' && vj.clip_end_epoch_ms > 0) {
      pcSkewMs = vj.clip_end_epoch_ms - triggerEpochMs;
    }
  } catch {}
  const sync = !haveMeta
    ? { warn: true, reason: metaLost ? 'meta.json 못 받음' : 'meta.json 없음', retry: true }
    : Math.abs(pcSkewMs) > 500
    ? { warn: true, reason: `PC clock skew ${pcSkewMs} ms` }
    : Math.abs(video.front?.skewMs ?? 0) > 2000
    ? { warn: true, reason: `front skew ${video.front!.skewMs} ms` }
    : Math.abs(video.rear?.skewMs ?? 0) > 2000
    ? { warn: true, reason: `rear skew ${video.rear!.skewMs} ms` }
    : { warn: false, reason: '' };

  const logs: BbLogLine[] = [];
  for (const ln of logTxt.split('\n')) {
    const s = ln.trim();
    if (!s) continue;
    try {
      const o: any = JSON.parse(s);
      const tsStr = String(o.timestamp ?? '');
      logs.push({
        epochMs: parseLogEpoch(tsStr),
        ts: tsStr.slice(11, 23),
        process: o.app || o.application || '',
        level: String(o.level || 'INFO').toUpperCase(),
        msg: o.message ?? '',
      });
    } catch {}
  }
  logs.sort((a, b) => a.epochMs - b.epochMs);

  let pcd: PcdFrame[] | undefined;
  if (pcdBuf && pcdBuf.byteLength > 0) {
    try {
      const frames = parsePcdBin(pcdBuf, { frameZeroEpochMs: haveMeta ? startEpochMs : null, durMs: frameCount * tickMs });
      if (frames.length) pcd = frames;
    } catch (e) { console.warn('[blackbox] pcd.bin parse failed', e); }
  }

  return { tickMs, frameCount, startEpochMs, cols, numCols, data, logs, sync, video, pcd, robot };
}

export function parseBlackboxZip(bytes: Uint8Array, fileName = ''): BlackboxSession {
  const entries = unzipSync(bytes);
  const dec = new TextDecoder();
  const find = (name: string) => {
    const key = Object.keys(entries).find((k) => k === name || k.endsWith(`/${name}`));
    return key ? dec.decode(entries[key]) : '';
  };
  const dataTxt = find('data.log');
  if (!dataTxt) throw new Error('zip 안에 data.log가 없습니다');
  const bin = (name: string) => {
    const key = Object.keys(entries).find((k) => k === name || k.endsWith(`/${name}`));
    const u8 = key ? entries[key] : undefined;
    return u8 ? u8.buffer.slice(u8.byteOffset, u8.byteOffset + u8.byteLength) : undefined;
  };
  const pcdBuf = bin('pcd.bin');
  const anyKey = Object.keys(entries)[0] ?? '';
  const m = /(\d{8})[/_-](\d{2}_\d{2}_\d{2})/.exec(anyKey) ?? /(\d{8})[/_-](\d{2}_\d{2}_\d{2})/.exec(fileName);
  const snFromName = serialFromZipName(fileName);
  const sess = buildSession({
    metaTxt: find('meta.json'), dataTxt, logTxt: find('systemlog.log'), videoTxt: find('video.json'), pcdBuf,
    clips: { front: bin('front.mp4'), rear: bin('rear.mp4') },
    date: m?.[1] ?? '', session: m?.[2] ?? '',
    robotFallback: snFromName ? { serial: snFromName, via: 'filename' } : undefined,
  });
  return m ? { ...sess, id: { date: m[1], session: m[2] } } : sess;
}

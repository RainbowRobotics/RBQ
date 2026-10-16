import { gzip } from 'pako';
import { robotAuth } from './auth';
import { restBase } from './rest';
import { isDemo } from './demoFlag';
import { objectPath, uploadUrl, uploadHeaders, fmtBytes, type UploadStage } from './logUploadCommon';

async function fetchRobot(ip: string, path: string) {
  if (isDemo()) throw new Error('데모 모드 — 로그 서버 전송 없음');
  const res = await fetch(`${restBase(ip)}${path}`, { headers: robotAuth() });
  if (!res.ok) throw new Error(`로봇 응답 HTTP ${res.status}`);
  return new Uint8Array(await res.arrayBuffer());
}

async function put(body: Uint8Array, path: string, contentType: string, onStage?: UploadStage) {
  onStage?.('업로드 중…');
  const res = await fetch(uploadUrl(path), {
    method: 'POST',
    headers: uploadHeaders(contentType),
    body: body as unknown as BodyInit,
  });
  if (!res.ok) throw new Error(`업로드 실패 (HTTP ${res.status}) ${(await res.text()).slice(0, 200)}`);
}

export async function uploadSystemlog(ip: string, serial: string, date: string, onStage?: UploadStage) {
  onStage?.('로봇에서 받는 중…');
  const raw = await fetchRobot(ip, `/api/systemlog/date?date=${encodeURIComponent(date)}`);
  onStage?.('압축 중…');
  const gz = gzip(raw, { level: 6 });
  const path = objectPath(serial, 'systemlog', `${date}.jsonl.gz`);
  await put(gz, path, 'application/gzip', onStage);
  return { path, bytes: gz.length };
}

export async function uploadBlackbox(ip: string, serial: string, dir: string, onStage?: UploadStage) {
  onStage?.('로봇에서 받는 중…');
  const zip = await fetchRobot(ip, `/api/blackbox/zip?path=${encodeURIComponent(dir)}`);
  const path = objectPath(serial, 'blackbox', `${dir.replace(/[/\\]/g, '_')}.zip`);
  await put(zip, path, 'application/zip', onStage);
  return { path, bytes: zip.length };
}

export { fmtBytes };

export type UploadResult = { path: string; bytes: number; queued?: boolean };
export const pendingCount = async () => 0;
export const flushPending = async () => ({ sent: 0, failed: 0, dropped: 0 });
export const clearPending = async () => {};


export const SB_URL = (process.env.EXPO_PUBLIC_SUPABASE_URL || '').replace(/\/+$/, '');
export const SB_KEY = process.env.EXPO_PUBLIC_SUPABASE_ANON_KEY || '';
export const SB_BUCKET = process.env.EXPO_PUBLIC_SUPABASE_LOG_BUCKET || 'robot-logs';

export const uploadConfigured = !!(SB_URL && SB_KEY);

export const cloudConfigured = uploadConfigured;

export type UploadStage = (msg: string) => void;

export function objectPath(serial: string, kind: 'systemlog' | 'blackbox', name: string) {
  const s = (serial || 'unknown').replace(/[^A-Za-z0-9_.-]/g, '_');
  const stamp = new Date().toISOString().replace(/[-:]/g, '').replace(/\..+/, '');
  return `${s}/${kind}/${stamp}-${name}`;
}

export function uploadUrl(path: string) {
  return `${SB_URL}/storage/v1/object/${SB_BUCKET}/${path.split('/').map(encodeURIComponent).join('/')}`;
}

export function uploadHeaders(contentType: string) {
  return {
    Authorization: `Bearer ${SB_KEY}`,
    apikey: SB_KEY,
    'Content-Type': contentType,
  };
}

export const fmtBytes = (n: number) =>
  n >= 1048576 ? `${(n / 1048576).toFixed(1)} MB` : n >= 1024 ? `${Math.round(n / 1024)} KB` : `${n} B`;

export type Pending = {
  uri: string;
  path: string;
  contentType: string;
  bytes: number;
  label: string;
  at: number;
};

export const PENDING_KEY = 'rbq.logUpload.pending';

export function shouldQueue(err: unknown) {
  const m = String((err as Error)?.message || err);
  const http = m.match(/HTTP (\d{3})/);
  if (!http) return true;
  const s = Number(http[1]);
  if (s === 408 || s === 429) return true;
  return s >= 500;
}

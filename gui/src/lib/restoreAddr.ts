export type ConnProfile = 'lo' | 'lan' | 'wan';

const trim = (v?: string | null) => (v ?? '').trim();

export function restoreAddr(
  profile: ConnProfile,
  lanIp?: string | null,
  wanIp?: string | null,
  robotIp?: string | null,
): string {
  if (profile === 'lo') return '127.0.0.1';
  if (profile === 'wan') return trim(wanIp);
  return trim(lanIp) || trim(robotIp);
}

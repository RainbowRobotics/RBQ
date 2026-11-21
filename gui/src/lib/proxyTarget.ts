export type ProxyTargetSync = 'unchanged' | 'switched' | 'failed';

export async function syncProxyTarget(
  _robot: string,
  _vision: string,
  _f?: typeof fetch,
): Promise<ProxyTargetSync> {
  return 'unchanged';
}

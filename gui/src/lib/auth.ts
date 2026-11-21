const DEFAULT_PASSWORD = process.env.EXPO_PUBLIC_ROBOT_DEFAULT_AUTH || undefined;

let passwordFor: (target?: string) => string | undefined = () => undefined;

export function setRobotPasswordSource(fn: (target?: string) => string | undefined) { passwordFor = fn; }

export const ipKey = (ip: string) => `ip:${ip}`;

export function robotAuth(target?: string): Record<string, string> {
  const password = passwordFor(target) ?? DEFAULT_PASSWORD;
  return password ? { Authorization: `Basic ${btoa(`rbq:${password}`)}` } : {};
}

let onAuthFailure: () => void = () => {};
let failed: { target?: string; probe: boolean } = { probe: false };

export function setAuthFailureHandler(fn: () => void) { onAuthFailure = fn; }

export function reportAuthFailure(target?: string, probe = false, sent?: Record<string, string>): boolean {
  if (sent && (sent.Authorization ?? '') !== (robotAuth(probe ? target : undefined).Authorization ?? '')) return false;
  failed = { target, probe }; onAuthFailure(); return true;
}
export const authFailure = () => failed;

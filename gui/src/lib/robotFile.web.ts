import { robotAuth } from './auth';

export type RobotFileSrc = { uri: string; headers?: Record<string, string> } | null;

export function useRobotFile(_ip: string, direct: string | null, _route: string | null) {
  const src: RobotFileSrc = direct ? { uri: direct, headers: robotAuth() } : null;
  return { src, loading: false, failed: false };
}

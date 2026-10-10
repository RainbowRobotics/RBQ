import { useRobot } from '@/store/robot';
import { resolveEndpoints } from './resolveEndpoints';

export function restBase(ip: string) {
  return `http://${ip}:8080`;
}

export function streamerBase(ip: string) {
  const { vision } = resolveEndpoints(ip, useRobot.getState().visionIp);
  return `http://${vision}:8081`;
}

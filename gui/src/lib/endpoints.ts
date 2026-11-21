import { useRobot } from '@/store/robot';
import { resolveEndpoints } from './resolveEndpoints';
import { robotTlsPlatform } from './robotTlsPlatform';

export function restBase(ip: string) {
  return robotTlsPlatform.base(ip, 8080) ?? `https://${ip}:8080`;
}

export function streamerBase(ip: string) {
  const { vision } = resolveEndpoints(ip, useRobot.getState().visionIp);
  return robotTlsPlatform.base(vision, 8081) ?? `https://${vision}:8081`;
}

export function slamnavBase(ip: string) {
  const { vision } = resolveEndpoints(ip, useRobot.getState().visionIp);
  return robotTlsPlatform.base(vision, 8082) ?? `https://${vision}:8082`;
}

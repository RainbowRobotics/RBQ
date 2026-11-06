export type EndpointIps = { motion: string; vision: string };

export function resolveEndpoints(motionIp: string, visionIp?: string): EndpointIps {
  const v = (visionIp ?? '').trim();
  return { motion: motionIp, vision: v || motionIp };
}

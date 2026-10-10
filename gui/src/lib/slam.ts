import { buildVisionRequest, sendVisionRequest } from './vision';

export const SLAM_REQ = {
  stateSendback: 299,
  mapBuild: 300, mapStop: 301, mapSave: 302, mapLoad: 303, mapReload: 304,
  autoInit: 309,
  taskStart: 316, taskPause: 318, taskResume: 319, taskCancel: 320,
  scheduleStart: 321, scheduleStop: 322,
  annotModeOnOff: 324, quickAnnotOnOff: 325, annotSave: 327, clearTopo: 328,
  rtb: 344, eStop: 345,
  view2d: 400, view3d: 401, viewFollow: 402,
  MAPPING_MODE: 420, CONNECT: 430,
  lastRequest: 999,
} as const;

export const SLAM_MODE = { DEFAULT: 0, INDOOR: 1, OUTDOOR: 2 } as const;

const inRange = (req: number) => req >= SLAM_REQ.stateSendback && req <= SLAM_REQ.lastRequest;

export const slam = {
  bool(req: number, arg: boolean) {
    if (!inRange(req)) return;
    sendVisionRequest(buildVisionRequest(req, { bools: [arg] }));
  },
  int(req: number, arg: number) {
    if (!inRange(req)) return;
    sendVisionRequest(buildVisionRequest(req, { ints: [arg] }));
  },
  simple(req: number) {
    if (!inRange(req)) return;
    sendVisionRequest(buildVisionRequest(req));
  },
};


export type SimPose = {
  joints: number[];
  rpy: [number, number, number];
  xyz: [number, number, number];
};

export type MujocoSim = {
  advance(dt: number): number;
  setCtrl(ctrl: ArrayLike<number>): void;
  pose(): SimPose;
  jointVels(): number[];
  trunkState(): { quat: [number, number, number, number]; gyro: [number, number, number] };
  jointTorques(): number[];
  trunkVel(): [number, number, number];
  setInit(joints: number[]): void;
  setSpawn(pos: [number, number, number], yaw: number): void;
  reset(): void;
  dispose(): void;
  readonly timestep: number;
  readonly nu: number;
};

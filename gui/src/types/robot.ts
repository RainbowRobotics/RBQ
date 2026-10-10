
export type GaitName =
  | 'SITTING' | 'STANDING' | 'TROTTING' | 'TROT_STAIRS' | 'WAVING' | 'TROT_RUNNING' | 'DOCKING'
  | string;

export type MotionName =
  | 'sit' | 'stand' | 'walk' | 'stairs' | 'wave' | 'run' | 'dock'
  | 'ai_walk' | 'ai_vision' | 'ai_vision_slow' | 'ai_pronk' | 'ai_bound' | 'ai_pace' | 'ai_run'
  | 'ai_2leg_l' | 'ai_2leg_r' | 'ai_2leg_f' | 'ai_3leg_hl'
  | 'rl_walk' | 'rl_walk_vision' | 'zmp_calib'
  | 'pos_stand' | 'pos_sit' | 'lock'
  | 'estop' | 'auto_start';

export const INIT_STEPS = [
  'none', 'precheck', 'leg_power', 'comm', 'param', 'homing', 'imu', 'control',
] as const;
export type InitStep = (typeof INIT_STEPS)[number];

export const INIT_STATE = { idle: 0, run: 1, pass: 2, warn: 3, fail: 4 } as const;
export type InitState = 0 | 1 | 2 | 3 | 4;

export type AutostartInfo = {
  running: boolean;
  can_fd?: boolean;
  leg_rail_v?: number;
  step: number;
  steps: InitState[];
  step_ms: number[];
  elapsed_ms: number;
  fail_code: number;
  fail_ch: number;
  can_mode_warn: boolean;
  pre_standing?: boolean;
  emo_blocked?: boolean;
  gyro_warn?: boolean;
  acc_warn?: boolean;
  ch_comm: InitState[];
  ch_param: InitState[];
  ch_home: InitState[];
  home_err_deg: number[];
  power_retry: number;
  gyro_try: number;
  gyro_bias_dps?: number[];
  acc_norm_before?: number;
  acc_norm?: number;
  acc_pct: number;
};

export const ZMP_CALIB = { idle: 0, aligning: 1, running: 2, done: 3, failed: 4 } as const;
export type ZmpCalibInfo = {
  run: number;
  state: number;
  reason: number;
  percent: number;
  err_mm: number;
};

export const GYRO_CALIB = { idle: 0, resetting: 1, done: 2, failed: 3 } as const;
export type GyroCalibInfo = {
  run: number;
  state: number;
  reason: number;
  pass: boolean;
  limit_dps: number;
  elapsed_ms: number;
  total_ms: number;
  bias_dps: number[];
};

export const ACC_CALIB = { idle: 0, running: 1, done: 2, failed: 3 } as const;
export type AccCalibInfo = {
  run: number;
  state: number;
  reason: number;
  percent: number;
  norm_before: number;
  norm_after: number;
  ratio: number;
};

export type LegQcInfo = {
  status: number;
  current_joint: number;
  mode: number;
  n_samples: number;
  verdict: number[];
  roll_overall: number;
  pitch_overall: number;
  knee_overall: number;
};

export type LegAgingInfo = {
  status: number;
  lap: number;
  laps: number;
  stage: number;
  stages: number;
  elapsed_s: number;
  total_s: number;
};

export type LegCheckInfo = {
  status: number;
  joint: number;
  elapsed_s: number;
  total_s: number;
  log_count: number;
};

export type LegJointResult = {
  joint: number;
  mode: number;
  n_ref: number;
  ref: (number | null)[];
  val: (number | null)[][];
  z: number[][];
  verdict: number[][];
  leg: number[];
  err: number[];
  jump: number[];
  temp_warn: boolean[];
  temp: number[][];
  can_err: (number | null)[];
};

export type LegResultInfo = {
  seq: number;
  stage: number;
  items: LegJointResult[];
};

export const GAIT_CHECK = { idle: 0, entering: 1, walking: 2, stopping: 3, done: 4, failed: 5 } as const;
export type GaitCheckInfo = {
  run: number;
  state: number;
  reason: number;
  percent: number;
  mode?: number;
  duration_s: number;
  x_mm: number;
  y_mm: number;
  yaw_deg: number;
};

export const JOINT_COMM = { idle: 0, running: 1, done: 2, failed: 3 } as const;
export type JointCommCheckInfo = {
  run: number;
  state: number;
  reason: number;
  percent: number;
  measured: boolean;
  duration_s: number;
  amplitude_deg: number;
  rx_hz_min: number[];
  gap_max_ms: number[];
  missed?: number[];
  motor_ch?: number[];
  ch_err_frames?: number[];
  ch_warning?: number[];
  ch_passive?: number[];
  ch_bus_off?: number[];
  hz_warn: number;
  gap_warn_ms: number;
};

export type RobotStatus = {
  battery_pct: number;
  battery_voltage: number;
  gait_id: number;
  gait_name: GaitName;
  docking_status?: number;
  imu: boolean;
  imu_connected?: boolean;
  can_bus: boolean;
  find_pose: boolean;
  control_started: boolean;
  autostart?: AutostartInfo;
  zmp_calib?: ZmpCalibInfo;
  gyro_calib?: GyroCalibInfo;
  acc_calib?: AccCalibInfo;
  leg_qc?: LegQcInfo;
  leg_aging?: LegAgingInfo;
  leg_check?: LegCheckInfo;
  leg_result?: LegResultInfo;
  gait_check?: GaitCheckInfo;
  joint_comm_check?: JointCommCheckInfo;
};

export type PcStatus = {
  cpu_temp_c: number;
  cpu_throttled: boolean;
  cpu_core_usage: number[];
  mem_total_kb: number;
  mem_available_kb: number;
  mem_used_pct: number;
  swap_total_kb: number;
  swap_used_kb: number;
  swap_used_pct: number;
};

export type TripCounter = {
  distance_mm: number;
  step_cnt: number;
  time_s: number;
  last_reset_unix?: number;
};

export type Trip = {
  total: TripCounter;
  tripA: TripCounter;
  tripB: TripCounter;
};

export type GamepadStatus = {
  ownerIP: string;
  requesterIP: string;
  IsOwner: boolean;
};

export type VersionInfo = {
  version: string;
  branch: string;
  commit_date: string;
  build_date: string;
};

export type LogLevel = 'TRACE' | 'DEBUG' | 'INFO' | 'SUCCESS' | 'WARNING' | 'ERROR' | 'FATAL';

export type LogLine = {
  ts: string;
  process: string;
  level: LogLevel;
  msg: string;
  rxMs?: number;
};

export type AggStatus = 'green' | 'amber' | 'red';

export type ConnectionState = 'disconnected' | 'connecting' | 'connected';

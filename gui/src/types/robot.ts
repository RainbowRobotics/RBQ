
export type GaitName =
  | 'SITTING' | 'STANDING' | 'TROTTING' | 'TROT_STAIRS' | 'WAVING' | 'TROT_RUNNING' | 'DOCKING'
  | string;

export type MotionName =
  | 'sit' | 'stand' | 'walk' | 'stairs' | 'wave' | 'run' | 'dock'
  | 'ai_walk' | 'ai_vision' | 'ai_vision_slow' | 'ai_eco' | 'ai_pronk' | 'ai_bound' | 'ai_pace' | 'ai_run'
  | 'ai_2leg_l' | 'ai_2leg_r' | 'ai_2leg_f' | 'ai_3leg_hl'
  | 'rl_walk' | 'rl_walk_vision' | 'zmp_calib'
  | 'pos_stand' | 'pos_sit' | 'lock'
  | 'estop' | 'auto_start';

export const INIT_STEPS = [
  'none', 'precheck', 'leg_power', 'comm', 'param', 'homing', 'imu', 'control', 'data_rate',
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
  data_rate_kbps?: number;
  rate_rewritten?: number;
  ch_rate_kbps?: number[];
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
  temp: (number | null)[][];
  can_err: (number | null)[];
};

export type LegResultInfo = {
  seq: number;
  stage: number;
  items: LegJointResult[];
};

export type LegRecordBoard = { state: number; saved_at: number | null; vals: (number | null)[] | null };
export type LegRecordInfo = { seq: number; boards: LegRecordBoard[] };

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

export const DQ_CHECK = { idle: 0, running: 1, done: 2, failed: 3 } as const;
export type DqCheckInfo = {
  run: number;
  state: number;
  reason: number;
  percent: number;
  leg: number;
  ask_joint: number;
  ask_left: number;
  ask_aligned?: number;
  step_result: number[][];
  step_err_deg: number[][];
  err_deg: number[];
  result: number[];
  limit_deg: number;
};

export const DQ_ALIGN = { done: 0, rejected: 1, noReply: 2, standing: 3 } as const;
export type DqAlignInfo = {
  seq: number;
  running: boolean;
  joint: number;
  result: number;
};

export type RobotStatus = {
  battery_pct: number;
  battery_voltage: number;
  gait_id: number;
  gait_name: GaitName;
  goal_gait_id?: number;
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
  leg_record?: LegRecordInfo;
  gait_check?: GaitCheckInfo;
  joint_comm_check?: JointCommCheckInfo;
  dq_check?: DqCheckInfo;
  dq_align?: DqAlignInfo;
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
  uptime_s?: number;
  boot_time_unix?: number;
  processes?: { name: string; pid: number; cpu_pct: number; mem_rss_kb: number }[];
};

export type SystemNic = {
  name: string;
  mac?: string;
  state?: string;
  speed_mbps?: number;
  duplex?: string;
  mtu?: number;
  wireless?: boolean;
  wifi_signal_dbm?: number;
  wifi_link_quality?: number;
  rx_bytes?: number;
  tx_bytes?: number;
  rx_packets?: number;
  tx_packets?: number;
  rx_errors?: number;
  tx_errors?: number;
  rx_dropped?: number;
  tx_dropped?: number;
  rx_fifo_errors?: number;
  tx_fifo_errors?: number;
  rx_frame_errors?: number;
  tx_carrier_errors?: number;
  collisions?: number;
  flags?: string[];
  tx_queue_len?: number;
  irq?: number;
  memory?: string;
  driver?: string;
  bus_id?: string;
  can?: SystemCan;
  ipv4: string[];
};

export type SystemCan = {
  state: string;
  bitrate?: number;
  sample_point?: string;
  dbitrate?: number;
  dsample_point?: string;
  berr_tx?: number;
  berr_rx?: number;
  ctrlmode?: string[];
  restart_ms?: number;
  controller?: string;
  restarts?: number;
  bus_error?: number;
  arbitration_lost?: number;
  error_warning?: number;
  error_passive?: number;
  bus_off?: number;
};

export type SystemStatsSample = {
  t: number;
  cpu: number;
  core: number[];
  mhz: number[];
  temp: number;
  ctemp: number[];
  pw: { pkg?: number; core?: number; uncore?: number; psys?: number };
  mem: number;
  swap: number;
  thr: number;
};

export type SystemStats = {
  period_s: number;
  window_s: number;
  layout: {
    types: ('P' | 'E' | 'LPE')[];
    isolated: number[];
    offline?: number[];
    roles: { daemon: number; task: number; can: number; comm: number; irq: number; heightmap: number };
    max_mhz?: number;
    temp_high?: number;
    temp_crit?: number;
  };
  samples: SystemStatsSample[];
};

export type SystemInfo = {
  hostname?: string;
  os?: string;
  kernel?: string;
  kernel_build?: string;
  arch?: string;
  realtime?: boolean;
  cpu_model?: string;
  cpu_count?: number;
  cpu_isolated?: string;
  cpu_mhz?: number;
  cpu_max_mhz?: number;
  cpu_roles?: { daemon: number; task: number; can: number; comm: number; irq: number; heightmap: number };
  load_avg?: [number, number, number];
  throttle_count?: number;
  disks?: { mount: string; device: string; fs: string; total_kb: number; avail_kb: number }[];
  interfaces?: SystemNic[];
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
  system?: string;
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

export type HealthFinding = {
  id: string;
  severity: 'warn' | 'fault';
  component?: string;
  value?: number;
  threshold?: number;
  unit?: string;
  code?: number;
  detail?: string;
  since: number;
};

export type RobotHealth = {
  overall: 'ok' | 'warn' | 'fault';
  findings: HealthFinding[];
};

export type ConnectionState = 'disconnected' | 'connecting' | 'connected';

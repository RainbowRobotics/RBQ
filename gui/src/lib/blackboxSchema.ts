const PDU_OUT = ['lan_12v_1', 'lan_12v_2', 'lan_12v_3', 'wan_12v', 'tbd_12v', 'front_12v', 'hind_12v',
  'camera_0', 'camera_1', 'camera_2', 'camera_3', 'camera_4', 'camera_5', 'leg_48v', 'arm_48v',
  'pdu_48v', 'side_cam_left', 'side_cam_right', 'amp'];
const PDU_IN = ['emo', 'chg_station', 'chg_external'];
const PDU_BAT_F = ['voltage', 'current', 'soc', 'detect', 'ov', 'uv', 'ot', 'ut', 'occ', 'ocd', 'scd', 'cid'];
const PDU_TEMP = ['bat_left', 'bat_right', 'top_power', 'pdu_power', 'pdu_signal'];
const STATUS_BEFORE_ATTCH = ['con_start', 'ready_pos', 'ground_pos', 'force_con', 'ext_joy',
  'is_standing', 'can_check', 'find_home', 'gait_id', 'docking_stat'];
const STATUS_AFTER_ATTCH = ['is_fall', 'imu_success', 'dq_success', 'key_mapping'];
const CMD_F = ['vel_x', 'vel_y', 'omega_z', 'roll', 'pitch', 'yaw', 'delta_body_h', 'delta_foot_h',
  'delta_max_speed', 'gait_id', 'gait_transition', 'updated'];
const JOY_F = ['l_ud', 'l_rl', 'r_ud', 'r_rl', 'lt', 'rt'];
const CAN_CH_F = ['rx_hz', 'tx_hz', 'err', 'state', 'bus_off', 'overrun'];

export function demoBlackboxChannels(): string[] {
  const c: string[] = [];
  const per = (pre: string) => ['joint.pos', 'joint.vel', 'joint.torque', 'motor.cur', 'motor.temp',
    'board.temp', 'motor.err.jam', 'motor.err.cur', 'motor.connect', 'motor.comm_stat']
    .map((k) => (pre ? `${pre}.${k}` : k));
  for (let i = 0; i < 12; i++) for (const k of per('')) c.push(`${k}[${i}]`);
  for (let i = 0; i < 4; i++) for (const k of per('wheel')) c.push(`${k}[${i}]`);
  c.push('imu.rpy.r', 'imu.rpy.p', 'imu.rpy.y', 'imu.gyro.x', 'imu.gyro.y', 'imu.gyro.z',
    'imu.acc.x', 'imu.acc.y', 'imu.acc.z', 'imu.connected');
  for (let i = 0; i < 12; i++) c.push(`ref.joint.pos[${i}]`);
  for (let i = 0; i < 12; i++) c.push(`ref.joint.torque[${i}]`);
  c.push('cpu.temp', 'cpu.throttled');
  for (let i = 0; i < 4; i++) c.push(`cpu.core[${i}]`);
  c.push('mem.total_kb', 'mem.avail_kb', 'swap.total_kb', 'swap.used_kb',
    'deadline_miss', 'deadline_miss.motion', 'deadline_miss.wbc', 'process_time_ms',
    'lan2can.connected', 'can.type');
  for (const p of PDU_OUT) c.push(`pdu.out.${p}.state`, `pdu.out.${p}.v`, `pdu.out.${p}.i`);
  for (const p of PDU_IN) c.push(`pdu.in.${p}.state`, `pdu.in.${p}.v`, `pdu.in.${p}.i`);
  for (const b of ['left', 'right']) for (const f of PDU_BAT_F) c.push(`pdu.bat.${b}.${f}`);
  for (const k of PDU_TEMP) c.push(`pdu.temp.${k}`);
  for (const f of STATUS_BEFORE_ATTCH) c.push(`status.${f}`);
  for (let i = 0; i < 16; i++) c.push(`status.attch[${i}]`);
  for (const f of STATUS_AFTER_ATTCH) c.push(`status.${f}`);
  for (const f of CMD_F) c.push(`cmd.${f}`);
  for (const f of JOY_F) c.push(`joy.${f}`);
  for (let i = 0; i < 16; i++) c.push(`joy.btn[${i}]`);
  for (let i = 0; i < 12; i++) c.push(`can.motor.rx_hz[${i}]`, `can.motor.gap_ms[${i}]`);
  for (let i = 0; i < 4; i++) c.push(`can.wheel.rx_hz[${i}]`, `can.wheel.gap_ms[${i}]`);
  for (let i = 0; i < 4; i++) for (const f of CAN_CH_F) c.push(`can.ch.${f}[${i}]`);
  return c;
}


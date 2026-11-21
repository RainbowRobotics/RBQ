export const GAIT_NAMES: Readonly<Record<number, string>> = {
  [-3]: 'FALL_RECOVERY', [-2]: 'FALL_MODE', [-1]: 'CONTROL_OFF',
  0: 'SITTING', 1: 'STANDING', 2: 'AIMING', 3: 'TROTTING', 4: 'TROT_STAIRS', 5: 'WAVING',
  6: 'TROT_RUNNING', 7: 'DOOR_OPENING', 8: 'ZMP_INITIALIZING', 9: 'MANIPULATION', 10: 'DOCKING',
  30: 'RL_TROT', 31: 'RL_FRONT_WALK', 32: 'RL_HIND_WALK', 33: 'RL_LEFT_WALK', 34: 'RL_RIGHT_WALK',
  35: 'RL_BOUND', 36: 'RL_PACE', 37: 'RL_PRONK', 38: 'RL_3LEG_HR', 39: 'RL_3LEG_HL', 40: 'RL_3LEG_FR',
  41: 'RL_3LEG_FL', 42: 'RL_TROT_VISION', 45: 'RL_TROT_RUN', 46: 'RL_SILENT', 47: 'RL_TROT_VISION_SLOW',
  48: 'RL_WALK_VISION', 49: 'RL_WALK', 50: 'RL_TROT_ECO', 51: 'RL_TROT_VISION_ECO',
};

export function gaitIdName(id: number): string {
  return GAIT_NAMES[id] ?? `#${id}`;
}

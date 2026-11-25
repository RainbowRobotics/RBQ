export type RobotGuard = { expected(): string | null; mismatch(found: string): void };

let guard: RobotGuard = { expected: () => null, mismatch: () => {} };

export function setRobotGuard(g: RobotGuard) { guard = g; }
export const robotGuard = (): RobotGuard => guard;

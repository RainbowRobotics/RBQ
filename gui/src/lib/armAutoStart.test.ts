import { describe, expect, it, vi } from 'vitest';
vi.mock('./rest', () => ({ rest: { commandStruct: vi.fn().mockResolvedValue({}) } }));
vi.mock('./robotState', () => ({ PROGRAM: { Motion: 0 } }));
import { rest } from './rest';
import { ARM_ALREADY_STARTED, ARM_START_RETRY_MS, armStartBlockReason, armStartFeedback, createArmStartRequests, type ArmStartContext } from './armAutoStart';
import type { RobotState, PduState } from './robotState';
const context = (): ArmStartContext => ({ accessLevel: 2, connected: true, switching: false, owner: '', isMine: false,
  lastRobotAt: Date.now(), lastRobotStatusAt: Date.now(), legStarting: false, robotKey: 'robot-A', isSim: false,
  robot: { attached: { arm: true }, gaitId: 0, isFall: false,
    joints: Array.from({ length: 18 }, () => ({ connected: true, run: false, locked: false })),
    armStat: { conStart: false, manualControl: false, missionType: 0, motionCmd: 0 } } as RobotState });

describe('arm auto start', () => {
  it('uses only the existing arm sequence and forbids replay after reconnect', async () => {
    vi.mocked(rest.commandStruct).mockClear();
    await createArmStartRequests().request(context());
    expect(rest.commandStruct).toHaveBeenCalledExactlyOnceWith('', 0, 802, undefined, { replayOnReconnect: false });
  });
  it('blocks detached, stale, moving, fallen, switching, owned and already-started robots', async () => {
    const base = context();
    const cases: ArmStartContext[] = [
      { ...base, accessLevel: 1 }, { ...base, accessLevel: 1, acknowledgedRunning: true }, { ...base, accessLevel: NaN }, { ...base, connected: false }, { ...base, switching: true }, { ...base, robot: undefined },
      { ...base, lastRobotAt: Date.now() - 4000 }, { ...base, owner: 'other', isMine: false },
      { ...base, lastRobotStatusAt: Date.now() - 4000 },
      { ...base, lastRobotStatusAt: undefined }, { ...base, lastRobotAt: NaN },
      { ...base, lastRobotAt: Date.now() + 10000 }, { ...base, droppedAt: Date.now() },
      { ...base, isSim: true }, { ...base, robotKey: '' },
      { ...base, legStarting: true },
      { ...base, robot: { ...base.robot!, attached: { ...base.robot!.attached, arm: false } } },
      { ...base, robot: { ...base.robot!, gaitId: 3 } },
      { ...base, robot: { ...base.robot!, isFall: true } },
      { ...base, robot: { ...base.robot!, armStat: { ...base.robot!.armStat, conStart: true } } },
    ];
    vi.mocked(rest.commandStruct).mockClear();
    for (const c of cases) { expect(armStartBlockReason(c)).not.toBeNull(); await expect(createArmStartRequests().request(c)).rejects.toThrow(); }
    expect(rest.commandStruct).not.toHaveBeenCalled();
  });
  it('allows stopped gaits and reports transport rejection instead of claiming completion', async () => {
    for (const gaitId of [-1, 0, 1]) {
      const c = context(); c.robot!.gaitId = gaitId;
      expect(armStartBlockReason(c)).toBeNull();
    }
    vi.mocked(rest.commandStruct).mockRejectedValueOnce(new Error('not owner'));
    await expect(createArmStartRequests().request(context())).rejects.toThrow('not owner');
  });
  it('blocks partial FD startup even when the legacy CON_START bit stays false', () => {
    for (const field of ['run', 'locked'] as const) {
      const c = context(); c.robot!.joints[14][field] = true;
      expect(armStartBlockReason(c)).toBe('팔 제어가 이미 시작돼 있습니다.');
    }
    const c = context(); c.robot!.joints = [];
    expect(armStartBlockReason(c)).not.toBeNull();
  });
  it('keeps the confirmation bound to its robot and rejects a switched target', async () => {
    vi.mocked(rest.commandStruct).mockClear();
    await expect(createArmStartRequests().request({ ...context(), robotKey: 'robot-B' }, 'robot-A')).rejects.toThrow();
    expect(rest.commandStruct).not.toHaveBeenCalled();
  });
  it('retains the request across modal lifetimes, concurrent clicks and reconnects', async () => {
    let resolve!: (value: unknown) => void;
    vi.mocked(rest.commandStruct).mockClear().mockImplementationOnce(() => new Promise(r => { resolve = r; }));
    const requests = createArmStartRequests();
    const pending = requests.request(context());
    expect(requests.get('robot-A')?.phase).toBe('sending');
    await expect(requests.request(context())).rejects.toThrow();
    resolve({}); await pending;
    expect(requests.get('robot-A')?.phase).toBe('sent');
    await expect(requests.request({ ...context(), connected: false })).rejects.toThrow();
    await expect(requests.request(context())).rejects.toThrow();
    expect(rest.commandStruct).toHaveBeenCalledTimes(1);
    await requests.request({ ...context(), robotKey: 'robot-B' });
    expect(rest.commandStruct).toHaveBeenCalledTimes(2);
  });
  it('only the already-started reason can be acknowledged away', () => {
    const running = (): ArmStartContext => { const c = context();
      c.robot!.armStat.conStart = true; c.robot!.joints[12].locked = true; return c; };
    expect(armStartBlockReason(running())).toBe(ARM_ALREADY_STARTED);
    expect(armStartBlockReason({ ...running(), acknowledgedRunning: true })).toBeNull();
    for (const blocked of [{ connected: false }, { switching: true }, { isSim: true },
                           { owner: 'other', isMine: false }, { legStarting: true }] as const)
      expect(armStartBlockReason({ ...running(), ...blocked, acknowledgedRunning: true })).not.toBeNull();
    const fallen = running(); fallen.robot!.isFall = true;
    expect(armStartBlockReason({ ...fallen, acknowledgedRunning: true })).not.toBeNull();
    const noArm = running(); noArm.robot!.attached.arm = false;
    expect(armStartBlockReason({ ...noArm, acknowledgedRunning: true })).not.toBeNull();
  });
  it('keeps an ambiguous failure locked while the sequence could still be in flight', async () => {
    vi.mocked(rest.commandStruct).mockClear().mockRejectedValueOnce(new Error('timeout'));
    const requests = createArmStartRequests();
    await expect(requests.request(context())).rejects.toThrow('timeout');
    expect(requests.get('robot-A')).toMatchObject({ phase: 'unknown', error: 'timeout' });
    await expect(requests.request(context())).rejects.toThrow();
    expect(rest.commandStruct).toHaveBeenCalledTimes(1);
  });
  it('releases the lock for a deliberate retry once the sequence cannot be in flight', async () => {
    vi.useFakeTimers();
    try {
      vi.mocked(rest.commandStruct).mockClear().mockResolvedValue({});
      const requests = createArmStartRequests();
      const fresh = (): ArmStartContext => ({ ...context(), lastRobotAt: Date.now(), lastRobotStatusAt: Date.now() });
      await requests.request(fresh());
      expect(requests.inFlight('robot-A')).toBe(true);
      await expect(requests.request(fresh())).rejects.toThrow('이미 팔 기동을 요청했습니다');

      vi.advanceTimersByTime(ARM_START_RETRY_MS + 1);
      expect(requests.inFlight('robot-A')).toBe(false);
      await requests.request(fresh());
      expect(rest.commandStruct).toHaveBeenCalledTimes(2);
      expect(requests.inFlight('robot-A')).toBe(true);
    } finally { vi.useRealTimers(); }
  });
});

describe('power-aware startup feedback',()=>{
  it('does not count retained RUN and KP bits while the rail is off or unknown',()=>{
    const r=context().robot!;
    r.joints.forEach(j=>{j.run=true;j.locked=true;});
    expect(armStartFeedback(r,{fetAdd:true} as PduState)).toEqual([6,6,6]);
    expect(armStartFeedback(r,{fetAdd:false} as PduState)).toEqual([0,0,0]);
    expect(armStartFeedback(r,undefined)).toEqual([0,0,0]);
  });
  it('reports joints in simulation, where no PDU board drives the rail',()=>{
    const r=context().robot!;
    r.joints.forEach(j=>{j.locked=true;});
    expect(armStartFeedback(r,{fetAdd:false} as PduState,true)).toEqual([6,0,6]);
    expect(armStartFeedback(r,undefined,true)).toEqual([6,0,6]);
  });
});

import { describe, it, expect, vi, beforeEach } from 'vitest';
vi.mock('./userCommand', () => ({ sendUserCommand: vi.fn() }));
vi.mock('./robotState', () => ({ PROGRAM: { ManiControl: 8, Motion: 2, QuadWalk: 3 } }));
import { arm, MANI_MOTION, DRAWING_SHAPE, GRASP_ROLL, GRASP_PITCH } from './arm';
import { armMotionReady } from './armMotion';
import { sendUserCommand } from './userCommand';

const sent = () => (sendUserCommand as unknown as ReturnType<typeof vi.fn>).mock.calls.at(-1);

describe('arm motion readiness', () => {
  it('needs control started and the arm not locked — manual mode is irrelevant', () => {
    expect(armMotionReady({ conStart: true, lockPosition: false })).toBe(true);
    expect(armMotionReady({ conStart: false, lockPosition: false })).toBe(false);
    expect(armMotionReady({ conStart: true, lockPosition: true })).toBe(false);
    expect(armMotionReady(undefined)).toBe(false);
  });
});

describe('static and demo motion commands', () => {
  beforeEach(() => { (sendUserCommand as unknown as ReturnType<typeof vi.fn>).mockClear(); });

  it('keeps the Enums.h numbering for Mani_Motions', () => {
    expect(MANI_MOTION).toMatchObject({
      Straight: 100, Folding: 101, Ready: 102, Lift: 103, Packing: 104,
      DoorReady: 105, DoorFinish: 106, ShakeHand: 107, PayloadBody: 108,
      PickUpReady: 109, TaskMotionReady: 110,
    });
    expect(DRAWING_SHAPE).toMatchObject({ Circle: 10, Heart: 11, Star: 12, Infinity: 13 });
    expect(GRASP_ROLL).toMatchObject({ Vertical: 10, Horizontal: 11, None: -1 });
    expect(GRASP_PITCH).toMatchObject({ Forward: 10, Downward: 11, None: -1 });
  });

  it('sends GO_MOTION(102) with the pose id, and Folding carries mode 1 like the Qt button', () => {
    arm.goMotion('Ready');
    expect(sent()).toEqual([8, 102, [102, 0], [], [], { replayOnReconnect: false }]);
    arm.goMotion('Folding', 1);
    expect(sent()).toEqual([8, 102, [101, 1], [], [], { replayOnReconnect: false }]);
  });

  it('sends the task motions with no arguments', () => {
    arm.moveCircle();   expect(sent()?.[1]).toBe(109);
    arm.moveInfinity(); expect(sent()?.[1]).toBe(110);
    arm.moveHeart();    expect(sent()?.[1]).toBe(111);
    arm.moveRbLogo();   expect(sent()?.[1]).toBe(115);
    arm.greeting();     expect(sent()?.[1]).toBe(360);
  });

  it('sends the demo commands on the 1000 block', () => {
    arm.demoGreeting();         expect(sent()?.[1]).toBe(1000);
    arm.demoAllJointMove();     expect(sent()?.[1]).toBe(1001);
    arm.demoHandshake();        expect(sent()?.[1]).toBe(1002);
    arm.demoChickenHead();      expect(sent()?.[1]).toBe(1010);
    arm.demoLockJoints(true);   expect(sent()?.[1]).toBe(1013);
    arm.demoLockJoints(false);  expect(sent()?.[1]).toBe(1014);
    arm.demoDrawingReady();     expect(sent()?.[1]).toBe(1020);
    arm.demoPayload(true);      expect(sent()?.[1]).toBe(1031);
    arm.demoPayload(false);     expect(sent()?.[1]).toBe(1032);
    arm.demoAutoRecovery();     expect(sent()?.[1]).toBe(1050);
    arm.demoDampingMode();      expect(sent()?.[1]).toBe(1051);
  });

  it('carries the drawing shape and the grasping roll/pitch selection', () => {
    arm.demoDrawingStart('Star');
    expect(sent()).toEqual([8, 1021, [12], [], [], { replayOnReconnect: false }]);
    arm.demoGraspingReady('Horizontal', 'Downward');
    expect(sent()).toEqual([8, 1030, [11, 11], [], [], { replayOnReconnect: false }]);
    arm.demoGraspingReady('None', 'None');
    expect(sent()).toEqual([8, 1030, [-1, -1], [], [], { replayOnReconnect: false }]);
  });
});

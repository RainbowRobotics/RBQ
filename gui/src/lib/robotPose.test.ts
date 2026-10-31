import {describe,it,expect} from 'vitest';
import {rbaGripperOpen} from './robotPose';
describe('RBA gripper telemetry units',()=>{
  it('keeps small hardware angles continuous across the old .05 threshold',()=>{
    for(const angle of [.04,.05,.051,1.14])expect(rbaGripperOpen(angle,false)).toBeCloseTo(angle/1.14);
    expect(rbaGripperOpen(.051,false)).toBeGreaterThan(rbaGripperOpen(.05,false));
  });
  it('interprets only reported simulation positions as metres',()=>{
    expect(rbaGripperOpen(.04,true)).toBe(1);
    expect(rbaGripperOpen(.02,true)).toBe(.5);
    expect(rbaGripperOpen(-1,false)).toBe(0);
    expect(rbaGripperOpen(10,false)).toBe(1);
    expect(rbaGripperOpen(NaN,false)).toBe(0);
  });
});

import { describe, it, expect } from 'vitest';
import { armAvailable } from './armAvailability';
import type { RobotState, PduState } from './robotState';
const robot = { attached: { arm: true }, joints: Array.from({length:18},()=>({connected:true})) } as RobotState;
const pdu = (fetAdd:boolean) => ({fetAdd}) as PduState;
describe('arm operation after PDU power loss',()=>{
 it('ignores stale attachment/RUN/control when ADD 48V is off',()=>{
  expect(armAvailable(robot,pdu(true))).toBe(true);
  expect(armAvailable(robot,pdu(false))).toBe(false);
  expect(armAvailable(robot,undefined)).toBe(false);
 });
 it('treats a reported simulation as powered — no PDU board fills S_FET_ADD there',()=>{
  expect(armAvailable(robot,pdu(false),true)).toBe(true);
  expect(armAvailable(robot,undefined,true)).toBe(true);
  expect(armAvailable(robot,pdu(false),false)).toBe(false);
  expect(armAvailable({...robot,attached:{...robot.attached,arm:false}},pdu(false),true)).toBe(false);
 });
 it('requires all six arm CAN nodes after power returns',()=>{
  const disconnected={...robot,joints:robot.joints.map((j,i)=>({...j,connected:i!==15}))};
  expect(armAvailable(disconnected,pdu(true))).toBe(false);
  expect(armAvailable({...robot,joints:robot.joints.slice(0,12)},pdu(true))).toBe(false);
  expect(armAvailable({...robot,attached:{...robot.attached,arm:false}},pdu(true))).toBe(false);
 });
});

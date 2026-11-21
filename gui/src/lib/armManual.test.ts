import {describe,it,expect,vi} from 'vitest';
vi.mock('./rest',()=>({rest:{commandStruct:vi.fn()}}));
vi.mock('./robotState',()=>({PROGRAM:{ManiControl:8}}));
import {armManualReady,createArmDrag,createArmHold,manualCommands as cmd,sendArmManual} from './armManual';
import {rest} from './rest';
import {MC} from './arm';
describe('manual arm command lifecycle',()=>{
 it('sends relocated modes through the acknowledged, non-replaying manual transport',async()=>{
  for(const command of [MC.DEMO_CHICKEN_HEAD_MODE,MC.DEMO_ROBOT_CONTROL_MODE]){
   await sendArmManual({command});
   expect(rest.commandStruct).toHaveBeenLastCalledWith('',8,command,{int:undefined,char:undefined},{replayOnReconnect:false});
  }
 });
 it('uses Qt entry and all six EE axes, and current gripper transport',async()=>{
  await sendArmManual(cmd.enter);
  expect(rest.commandStruct).toHaveBeenLastCalledWith('',8,130,{int:undefined,char:undefined},{replayOnReconnect:false});
  for(const type of [1,2] as const)for(const axis of [1,2,3] as const){
   await sendArmManual(cmd.jog(type,axis,-1));
   expect(rest.commandStruct).toHaveBeenLastCalledWith('',8,131,{int:[type,axis,-1],char:undefined},{replayOnReconnect:false});
  }
  await sendArmManual(cmd.grip(true));expect(rest.commandStruct).toHaveBeenLastCalledWith('',8,501,{int:undefined,char:[1]},{replayOnReconnect:false});
  await sendArmManual(cmd.home);expect(rest.commandStruct).toHaveBeenLastCalledWith('',8,102,{int:[101,1],char:undefined},{replayOnReconnect:false});
 });
 it('sends release before a pending start ACK and cancels its retries',async()=>{
  let releaseStop!:()=>void;
  const send=vi.fn().mockReturnValueOnce(new Promise(()=>{}))
    .mockReturnValueOnce(new Promise<void>(r=>{releaseStop=r;})).mockResolvedValue(undefined);
  const hold=createArmHold(send,vi.fn()),start=cmd.jog(1,1,1),stop=cmd.jog(1,1,0);
  expect(hold.start('x',start,stop)).toBe(true);
  expect(hold.start('y',cmd.jog(1,2,1),cmd.jog(1,2,0))).toBe(false);
  hold.stop('y');expect(send).toHaveBeenCalledTimes(1);
  hold.stop();hold.stop();
  expect(send.mock.calls.map(c=>c[0])).toEqual([start,stop]);
  expect(send.mock.calls[0][1].signal.aborted).toBe(true);
  expect(hold.start('grip',cmd.grip(true),cmd.gripStop)).toBe(false);
  releaseStop();await hold.settled();
  expect(hold.start('grip',cmd.grip(true),cmd.gripStop)).toBe(true);
 });
 it('reports a failed start and still sends stop',async()=>{
  const send=vi.fn().mockRejectedValueOnce(new Error('disconnect')).mockResolvedValue(undefined),error=vi.fn();
  const hold=createArmHold(send,error);hold.start('grip',cmd.grip(false),cmd.gripStop);
  await Promise.resolve();hold.pulse();hold.stop();await expect(hold.settled()).rejects.toThrow('disconnect');
  expect(send.mock.calls.map(c=>c[0])).toEqual([cmd.grip(false),cmd.gripStop]);expect(error).toHaveBeenCalledTimes(1);
 });
 it('lets a deliberate new press retry after a failed start, without settled()',async()=>{
  const send=vi.fn().mockRejectedValueOnce(new Error('not owner')).mockResolvedValue(undefined),error=vi.fn();
  const hold=createArmHold(send,error);
  expect(hold.start('x',cmd.jog(1,1,1),cmd.jog(1,1,0))).toBe(true);
  for(let i=0;i<8;i++)await Promise.resolve();
  expect(error).toHaveBeenCalledTimes(1);
  expect(send.mock.calls.map(c=>c[0])).toEqual([cmd.jog(1,1,1),cmd.jog(1,1,0)]);
  expect(hold.start('x',cmd.jog(1,1,1),cmd.jog(1,1,0))).toBe(true);
  await hold.settled();
  expect(send.mock.calls.map(c=>c[0])).toEqual([cmd.jog(1,1,1),cmd.jog(1,1,0),cmd.jog(1,1,1)]);
 });
 it('ignores old completion after disconnect and does not unlock a new hold',async()=>{
  let rejectOld!:(e:Error)=>void;
  const send=vi.fn().mockReturnValueOnce(new Promise((_,r)=>{rejectOld=r;})).mockResolvedValue(undefined),error=vi.fn();
  const hold=createArmHold(send,error);
  hold.start('x',cmd.jog(1,1,1),cmd.jog(1,1,0));hold.cancel();
  expect(send.mock.calls[0][1].signal.aborted).toBe(true);
  hold.start('grip',cmd.grip(true),cmd.gripStop);
  rejectOld(new Error('old connection'));await hold.settled();
  expect(error).not.toHaveBeenCalled();
  expect(hold.start('y',cmd.jog(1,2,1),cmd.jog(1,2,0))).toBe(false);
  hold.stop();await hold.settled();
  expect(send.mock.calls.map(c=>c[0])).toEqual([cmd.jog(1,1,1),cmd.grip(true),cmd.gripStop]);
 });
});

describe('manual telemetry readiness',()=>{
 const stat={conStart:true,manualControl:false,missionType:20,lockPosition:false};
 it('accepts the CONTROL_START and UNLOCK mission feedback without the cleared manual bit',()=>{
  expect(armManualReady(stat)).toBe(true);
  expect(armManualReady({...stat,missionType:0,manualControl:true})).toBe(true);
 });
 it('blocks preparing, fixed, unrelated and missing telemetry',()=>{
  expect(armManualReady({...stat,conStart:false})).toBe(false);
  expect(armManualReady({...stat,lockPosition:true})).toBe(false);
  expect(armManualReady({...stat,missionType:0})).toBe(false);
  expect(armManualReady(undefined)).toBe(false);
 });
});

describe('drag hold lane',()=>{
 it('does not send a start if the gesture ends before its first microtask',async()=>{
  const send=vi.fn().mockResolvedValue(undefined),hold=createArmHold(send,vi.fn());
  const drag=createArmDrag(hold,()=>true,vi.fn(),vi.fn());
  const pending=drag.move(1,1,1);drag.cancel();hold.stop();await pending;
  expect(send).not.toHaveBeenCalled();
 });
 it('deduplicates moves, stops on neutral and waits for stop ACK before reversing',async()=>{
  let acceptStop!:()=>void;
  const send=vi.fn().mockResolvedValue(undefined),hold=createArmHold(send,vi.fn());
  const drag=createArmDrag(hold,()=>true,vi.fn(),vi.fn());
  await drag.move(1,2,1);await drag.move(1,2,1);
  expect(send).toHaveBeenCalledTimes(1);
  send.mockImplementationOnce(()=>new Promise<void>(r=>{acceptStop=r;}));
  const reversed=drag.move(1,2,-1);
  expect(send.mock.calls.map(c=>c[0])).toEqual([cmd.jog(1,2,1),cmd.jog(1,2,0)]);
  acceptStop();await reversed;
  await drag.move(1,2,0);await hold.settled();
  expect(send.mock.calls.map(c=>c[0])).toEqual([cmd.jog(1,2,1),cmd.jog(1,2,0),cmd.jog(1,2,-1),cmd.jog(1,2,0)]);
 });
 it.each(['release','disconnect','not-ready','neutral'] as const)('does not resume queued rotation after %s',async(reason)=>{
  let acceptStop!:()=>void,ready=true;
  const send=vi.fn().mockResolvedValue(undefined),hold=createArmHold(send,vi.fn());
  const drag=createArmDrag(hold,()=>ready,vi.fn(),vi.fn());
  await drag.move(2,3,1);
  send.mockImplementationOnce(()=>new Promise<void>(r=>{acceptStop=r;}));
  const reversed=drag.move(2,3,-1);
  if(reason==='not-ready')ready=false;
  else if(reason==='neutral')await drag.move(2,3,0);
  else {drag.cancel();if(reason==='disconnect')hold.cancel();else hold.stop();}
  acceptStop();await reversed;
  expect(send.mock.calls.map(c=>c[0])).toEqual([cmd.jog(2,3,1),cmd.jog(2,3,0)]);
 });
 it('does not start the opposite direction after stop rejection',async()=>{
  const error=vi.fn(),send=vi.fn().mockResolvedValue(undefined),hold=createArmHold(send,error);
  const drag=createArmDrag(hold,()=>true,vi.fn(),error);
  await drag.move(1,1,1);send.mockRejectedValueOnce(new Error('stop rejected'));
  await drag.move(1,1,-1);
  expect(send.mock.calls.map(c=>c[0])).toEqual([cmd.jog(1,1,1),cmd.jog(1,1,0)]);
  expect(error).toHaveBeenCalled();
 });
});


describe('EE frame translation',()=>{
 it('uses type 3 only for EE translation; legacy movement and local rotation stay compatible',async()=>{
  for(const axis of [1,2,3] as const)for(const sign of [-1,0,1] as const){
   expect(cmd.jog(1,axis,sign,'ee')).toEqual({command:131,int:[3,axis,sign]});
   expect(cmd.jog(1,axis,sign)).toEqual({command:131,int:[1,axis,sign]});
   expect(cmd.jog(2,axis,sign,'ee')).toEqual({command:131,int:[2,axis,sign]});
  }
  await sendArmManual(cmd.jog(1,1,1,'ee'));
  expect(rest.commandStruct).toHaveBeenLastCalledWith('',8,131,{int:[3,1,1],char:undefined},{replayOnReconnect:false});
 });
 it('stops the previous frame before starting another and releases with the same type',async()=>{
  let acceptStop!:()=>void;
  const send=vi.fn().mockResolvedValue(undefined),hold=createArmHold(send,vi.fn());
  const drag=createArmDrag(hold,()=>true,vi.fn(),vi.fn());
  await drag.move(1,1,1,'robot');
  send.mockImplementationOnce(()=>new Promise<void>(r=>{acceptStop=r;}));
  const next=drag.move(1,1,1,'ee');
  expect(send.mock.calls.map(c=>c[0])).toEqual([cmd.jog(1,1,1),cmd.jog(1,1,0)]);
  acceptStop();await next;
  await drag.move(1,1,0,'ee');await hold.settled();
  expect(send.mock.calls.map(c=>c[0])).toEqual([cmd.jog(1,1,1),cmd.jog(1,1,0),cmd.jog(1,1,1,'ee'),cmd.jog(1,1,0,'ee')]);
 });
 it('cancels an EE start waiting for the previous stop',async()=>{
  let acceptStop!:()=>void;
  const send=vi.fn().mockResolvedValue(undefined),hold=createArmHold(send,vi.fn());
  const drag=createArmDrag(hold,()=>true,vi.fn(),vi.fn());
  await drag.move(1,3,1);
  send.mockImplementationOnce(()=>new Promise<void>(r=>{acceptStop=r;}));
  const pending=drag.move(1,3,1,'ee');drag.cancel();hold.stop();acceptStop();await pending;
  expect(send.mock.calls.map(c=>c[0])).toEqual([cmd.jog(1,3,1),cmd.jog(1,3,0)]);
 });
});

describe('EE speed wire contract',()=>{
 it('adds percent only to explicit starts and keeps release/legacy packets unchanged',()=>{
  for(const frame of ['robot','ee'] as const)for(const mode of [1,2] as const){
   expect(cmd.jog(mode,2,1,frame,35).int).toEqual([mode===1&&frame==='ee'?3:mode,2,1,35]);
   expect(cmd.jog(mode,2,0,frame,NaN)).toEqual(cmd.jog(mode,2,0,frame));
  }
  for(const bad of [0,9,101,-20,30.5,NaN,Infinity])expect(()=>cmd.jog(1,1,1,'robot',bad)).toThrow();
 });
 it('shares speed across drag modes and serializes a speed change behind stop acceptance',async()=>{
  let accept!:()=>void;
  const send=vi.fn().mockResolvedValue(undefined),hold=createArmHold(send,vi.fn());
  const drag=createArmDrag(hold,()=>true,vi.fn(),vi.fn());
  await drag.move(1,1,1,'ee',30);
  send.mockReturnValueOnce(new Promise<void>(r=>{accept=r;}));
  const next=drag.move(1,1,1,'ee',15);
  expect(send.mock.calls.map(c=>c[0])).toEqual([cmd.jog(1,1,1,'ee',30),cmd.jog(1,1,0,'ee')]);
  accept();await next;
  expect(send.mock.calls.at(-1)?.[0]).toEqual(cmd.jog(1,1,1,'ee',15));
  drag.cancel();hold.stop();await hold.settled();
 });
});

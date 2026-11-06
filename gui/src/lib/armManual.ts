import { rest } from './rest';
import { PROGRAM } from './robotState';
import type { CommandOptions } from './commandBus';
export type EeMoveFrame = 'robot' | 'ee';
export type ArmManualCommand = {command:number; int?:number[]; char?:number[]};
export const manualCommands = {
  enter: {command:130}, fix:{command:121}, control:{command:122},
  home: {command:102,int:[101,1]},
  jog: (type:1|2,axis:1|2|3,sign:-1|0|1,frame:EeMoveFrame='robot',speed?:number):ArmManualCommand => {
    if(sign!==0&&speed!==undefined&&(!Number.isInteger(speed)||speed<10||speed>100))throw new RangeError('Invalid EE speed');
    return {command:131,int:[type===1&&frame==='ee'?3:type,axis,sign,...(sign!==0&&speed!==undefined?[speed]:[])]};
  },
  grip: (open:boolean):ArmManualCommand => ({command:open?501:502,char:[1]}),
  gripStop: {command:501,char:[0]},
};
export function sendArmManual(c:ArmManualCommand, options?:CommandOptions) {
  return rest.commandStruct('',PROGRAM.ManiControl,c.command,{int:c.int,char:c.char},{...options,replayOnReconnect:false});
}
export function createArmHold<T = ArmManualCommand>(send:(c:T,options?:CommandOptions)=>Promise<unknown>,onError:(e:unknown)=>void) {
  let held:{id:string;start:T;stop:T;controller:AbortController}|null=null;
  let pending=Promise.resolve();
  let generation=0;
  let failure: {error:unknown}|null=null;
  let flight=false,stopping=false;
  let stopController:AbortController|null=null;
  const dispatch=(c:T,controller:AbortController)=>{
    const attempt=generation;
    let request:Promise<unknown>;
    try { request=send(c,{signal:controller.signal}); }
    catch(e) { request=Promise.reject(e); }
    return Promise.resolve(request).then(()=>{},e=>{
      if(attempt===generation&&!controller.signal.aborted){
        failure={error:e};onError(e);
        if(held?.controller===controller)stop();
      }
    });
  };
  function stop(id?:string){
    if(!held || (id && held.id!==id))return;
    const current=held;held=null;flight=false;
    current.controller.abort();
    stopping=true;
    const controller=stopController=new AbortController();
    pending=dispatch(current.stop,controller).finally(()=>{
      if(stopController===controller){stopping=false;stopController=null;}
    });
  }
  const pulse=()=>{
    if(!held||flight||failure)return;
    const current=held;
    flight=true;
    pending=dispatch(current.start,current.controller).finally(()=>{
      if(held===current)flight=false;
    });
  };
  return {
    start(id:string,start:T,stop:T){
      if(held||stopping)return false;
      failure=null;
      held={id,start,stop,controller:new AbortController()};pulse();return true;
    },
    pulse,
    stop,
    settled:()=>pending.then(()=>{if(failure){const {error}=failure;failure=null;throw error;}}),
    cancel(){
      generation++;held?.controller.abort();stopController?.abort();
      held=null;stopController=null;flight=false;stopping=false;failure=null;pending=Promise.resolve();
    },
  };
}

export function armManualReady(stat: {conStart:boolean; manualControl:boolean; missionType:number; lockPosition:boolean} | undefined) {
  return !!stat?.conStart && !stat.lockPosition && (stat.manualControl || stat.missionType === 20);
}

export function createArmDrag(hold:ReturnType<typeof createArmHold<ArmManualCommand>>,ready:()=>boolean,changed:(id:string|null)=>void,onError:(e:unknown)=>void) {
  let generation=0,requested:string|null=null;
  return {
    async move(mode:1|2,axis:1|2|3,sign:-1|0|1,frame:EeMoveFrame='robot',speed?:number){
      const id=sign?`drag${frame}${mode}${axis}${sign}${speed??''}`:null;
      if(id===requested)return;
      requested=id;const attempt=++generation;
      hold.stop();changed(null);
      if(!id||!ready())return;
      try {
        await hold.settled();
        if(attempt!==generation||!ready())return;
        if(hold.start(id,manualCommands.jog(mode,axis,sign,frame,speed),manualCommands.jog(mode,axis,0,frame)))changed(id);
      }catch(e){if(attempt===generation)onError(e);}
    },
    cancel(){generation++;requested=null;},
  };
}

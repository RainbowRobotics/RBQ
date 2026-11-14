import { useCallback, useEffect, useMemo, useRef, useState, type MutableRefObject } from 'react';
import { ActivityIndicator, AppState, PanResponder, Platform, StyleSheet, Text, View } from 'react-native';
import { Group, Vector3 } from 'three';
import { Canvas, useFrame } from '@/lib/r3fCanvas';
import { useArmMeshes, useRobotMeshError, type ArmMeshes } from '@/lib/robotMeshes';
import { useFeatureArm, useFeatures } from '@/store/capability';
import { GlBoundary, MeshPart } from '@/components/RobotModel3D';
import { ArmEeGizmo, type EeDragPicker } from '@/components/ArmEeGizmo';
import { eeDragSign, gripperOrientation, type EeDragProjection, type EeAxis, type EeMode, type EeGizmoControl } from '@/lib/armEeGizmo';
import { useTheme } from '@/theme';
import { t } from '@/lib/i18n';
import { useTelemetry } from '@/store/telemetry';
import { useRobots } from '@/store/robots';
import { rbaGripperOpen } from '@/lib/robotPose';

function GripperScene({ meshes, rba, simulation, control, picker, dragPicker }: {
  meshes: ArmMeshes; rba:boolean; simulation:boolean; control:EeGizmoControl;
  picker:MutableRefObject<((x:number,y:number)=>boolean)|null>; dragPicker:EeDragPicker;
}) {
  const body=useRef<Group>(null),wrist=useRef<Group>(null),anchor=useRef<Group>(null);
  const upper=useRef<Group>(null),lower=useRef<Group>(null);
  const angles=useMemo(()=>Array(6).fill(0),[]);
  useFrame(()=>{
    const robot=useTelemetry.getState().robot;
    if(!body.current||!wrist.current||!robot)return;
    body.current.rotation.set(robot.imu.rpy[0],robot.worldRpy[2],-robot.imu.rpy[1],'YXZ');
    for(let i=0;i<6;i++)angles[i]=robot.joints[12+i]?.position??0;
    gripperOrientation(wrist.current.quaternion,angles,rba);
    const opening=rbaGripperOpen(robot.joints[18]?.position??0,simulation)*.04;
    if(upper.current)upper.current.position.y=opening;
    if(lower.current)lower.current.position.y=-opening;
  },-1);
  const tip: [number,number,number]=rba?[.1737,-.015,-.015]:[0,.24555,0];
  return <group>
    <group ref={body}><group ref={wrist}>
      <group position={[-tip[0],-tip[1],-tip[2]]}>
        <MeshPart obj={meshes.link6} rot={rba?[-Math.PI/2,0,0]:[0,0,0]}/>
        {rba&&meshes.fingerUpper&&<group ref={upper} position={[.1737,0,-.015]}><MeshPart obj={meshes.fingerUpper} rot={[-Math.PI/2,0,0]}/></group>}
        {rba&&meshes.fingerLower&&<group ref={lower} position={[.1737,0,-.015]}><MeshPart obj={meshes.fingerLower} rot={[-Math.PI/2,0,0]}/></group>}
      </group>
      <group ref={anchor} rotation={rba?[0,0,0]:[0,0,Math.PI/2]}/>
    </group></group>
    <ArmEeGizmo anchor={anchor} body={body} control={control} picker={picker} dragPicker={dragPicker}/>
  </group>;
}

export function ArmEeSelector({ control, active, enabled, onDrag, onDragEnd, cancelRef }: {
  control:EeGizmoControl;active:boolean;enabled:boolean;cancelRef:MutableRefObject<(()=>void)|null>;
  onDrag:(mode:EeMode,axis:EeAxis,sign:-1|0|1)=>void;onDragEnd:()=>void;
}) {
  const {c,radius}=useTheme();
  const rba=useFeatureArm()==='rba1';
  const remoteSim=useFeatures(s=>s.features?.simulation===true);
  const localSim=useRobots(s=>s.currentSerial==='sim');
  const hasFc=useTelemetry(s=>Number.isFinite(s.robot?.fcYaw));
  const frameReady=control.mode===2||control.frame==='ee'||hasFc;
  const hasPose=useTelemetry(s=>(s.robot?.joints.length??0)>=18);
  const meshes=useArmMeshes(true,rba?'rba1':'rb1');
  const meshError=useRobotMeshError();
  const [error,setError]=useState(''),[epoch,setEpoch]=useState(0),[foreground,setForeground]=useState(AppState.currentState!=='background');
  const callbacks=useRef({control,enabled,onDrag,onDragEnd});callbacks.current={control,enabled,onDrag,onDragEnd};
  const gesture=useRef<EeDragProjection|null>(null), candidate=useRef<EeDragProjection|undefined>(undefined);
  const lastSign=useRef<-1|0|1>(0),start=useRef({x:0,y:0});
  const [vertical,setVertical]=useState(false);
  const cancel=useCallback(()=>{
    const held=gesture.current;gesture.current=null;candidate.current=undefined;
    setVertical(false);
    if(held)callbacks.current.onDragEnd();
  },[]);
  useEffect(()=>{
    const clear=()=>{gesture.current=null;candidate.current=undefined;setVertical(false);};
    cancelRef.current=clear;
    return()=>{if(cancelRef.current===clear)cancelRef.current=null;};
  },[cancelRef]);
  useEffect(()=>{
    const current=gesture.current;
    if(!active||!enabled||!hasPose||!frameReady||!!error||!!meshError||(current&&(current.mode!==control.mode||current.axis!==control.axis)))cancel();
  },[active,enabled,hasPose,frameReady,error,meshError,control.mode,control.axis,cancel]);
  useEffect(()=>{cancel();},[control.frame,cancel]);
  useEffect(()=>{
    const hidden=()=>{if(document.hidden)cancel();};
    if(Platform.OS==='web'){window.addEventListener('blur',cancel);document.addEventListener('visibilitychange',hidden);}
    return()=>{
      if(Platform.OS==='web'){window.removeEventListener('blur',cancel);document.removeEventListener('visibilitychange',hidden);}
      if(gesture.current){gesture.current=null;callbacks.current.onDragEnd();}
    };
  },[cancel]);
  useEffect(()=>{
    let background=false;
    const sub=AppState.addEventListener('change',state=>{
      setForeground(state==='active');
      if(state!=='active')cancel();
      if(state==='background')background=true;
      if(state==='active'&&background){if(Platform.OS==='ios')setEpoch(n=>n+1);background=false;}
    });
    return()=>sub.remove();
  },[cancel]);
  const picker=useRef<((x:number,y:number)=>boolean)|null>(null);
  const dragPicker:EeDragPicker=useRef(null);
  const size=useRef({width:1,height:1});
  const responder=useMemo(()=>PanResponder.create({
    onStartShouldSetPanResponder:e=>{
      const {locationX,locationY}=e.nativeEvent;
      start.current={x:locationX/size.current.width,y:locationY/size.current.height};
      candidate.current=dragPicker.current?.(start.current.x,start.current.y);
      return !!candidate.current;
    },
    onPanResponderGrant:()=>{
      const grab=candidate.current;candidate.current=undefined;
      if(!grab)return;
      callbacks.current.control.onSelect(grab.axis);
      if(!callbacks.current.enabled)return;
      gesture.current=grab;lastSign.current=0;setVertical(grab.vertical);
      callbacks.current.onDrag(grab.mode,grab.axis,0);
    },
    onPanResponderStart:(_,g)=>{if(g.numberActiveTouches>1)cancel();},
    onPanResponderMove:(_,g)=>{
      const grab=gesture.current;
      if(!grab)return;
      if(!callbacks.current.enabled||g.numberActiveTouches!==1){cancel();return;}
      const sign=eeDragSign(grab.project(start.current.x+g.dx/size.current.width,start.current.y+g.dy/size.current.height));
      if(sign!==lastSign.current){lastSign.current=sign;callbacks.current.onDrag(grab.mode,grab.axis,sign);}
    },
    onPanResponderRelease:cancel,onPanResponderTerminate:cancel,
    onPanResponderTerminationRequest:()=>false,
  }),[cancel]);


  return <View testID="arm-ee-selector" accessibilityHint={vertical?t('이 축은 위로 + · 아래로 −'):undefined} onLayout={e=>{const next=e.nativeEvent.layout;if(next.width!==size.current.width||next.height!==size.current.height)cancel();size.current=next;}}
    style={{flex:1,overflow:'hidden',borderRadius:radius.md,backgroundColor:c.panel2}}>
    {(error||meshError)?<Text accessibilityRole="alert" style={{color:c.redTx,padding:8}}>{error||meshError}</Text>:!hasPose?<Text style={{color:c.muted,padding:8}}>{t('팔 자세 수신 대기')}</Text>:!frameReady?<Text style={{color:c.muted,padding:8}}>{t('FC 좌표계 수신 대기 · 로봇 소프트웨어 확인')}</Text>:meshes?<GlBoundary onError={setError}><Canvas key={epoch} frameloop={active&&foreground?'always':'never'} camera={{position:[.4,.25,.5],fov:40}}
      onCreated={({camera})=>camera.lookAt(new Vector3(0,0,0))}>
      <color attach="background" args={[c.panel2]}/>
      <ambientLight intensity={.9}/><directionalLight position={[1,2,3]} intensity={1.5}/>
      <GripperScene meshes={meshes} rba={rba} simulation={remoteSim||localSim} control={control} picker={picker} dragPicker={dragPicker}/>
    </Canvas></GlBoundary>:<ActivityIndicator style={StyleSheet.absoluteFill}/>}
    <View style={[StyleSheet.absoluteFill,Platform.select({web:{touchAction:'none',userSelect:'none'}}) as any]} {...responder.panHandlers}/>
  </View>;
}

import { useEffect, useMemo, useRef, type MutableRefObject } from 'react';
import { Group, Quaternion, Vector3, type PerspectiveCamera } from 'three';
import { useFrame, useThree } from '@/lib/r3fCanvas';
import { useTelemetry } from '@/store/telemetry';
import { EE_AXES, eeGizmoScale, fcFrameOrientation, beginEeDrag, pickEeAxis, type EeDragProjection, type EeGizmoControl } from '@/lib/armEeGizmo';

export type EeDragPicker = MutableRefObject<((x:number,y:number)=>EeDragProjection|undefined)|null>;
export type EePicker = MutableRefObject<((x:number,y:number)=>boolean)|null>;

export function ArmEeGizmo({ anchor, body, control, picker, dragPicker }: {
  anchor: MutableRefObject<Group|null>; body: MutableRefObject<Group|null>;
  control: EeGizmoControl; picker: EePicker; dragPicker?: EeDragPicker;
}) {
  const group = useRef<Group>(null);
  const { camera, size } = useThree();
  const scratch = useMemo(()=>({position:new Vector3(), orientation:new Quaternion(), yaw:new Quaternion()}),[]);
  const handles = useMemo(()=>EE_AXES.map(a=>({ ...a,
    arrow:new Quaternion().setFromUnitVectors(new Vector3(0,1,0),new Vector3(...a.direction)),
    ring:new Quaternion().setFromUnitVectors(new Vector3(0,0,1),new Vector3(...a.direction)),
  })),[]);
  useEffect(()=>{
    const pick=(x:number,y:number)=>{
      if(!group.current?.visible)return false;
      const axis=pickEeAxis(camera,group.current,x,y);
      if(!axis)return false;
      control.onSelect(axis);return true;
    };
    picker.current=pick;
    return()=>{if(picker.current===pick)picker.current=null;};
  },[camera,control.onSelect,picker]);
  useEffect(()=>{
    if(!dragPicker)return;
    const begin=(x:number,y:number)=>group.current?.visible
      ?beginEeDrag(camera,group.current,control.mode,x,y,size.width,size.height):undefined;
    dragPicker.current=begin;
    return()=>{if(dragPicker.current===begin)dragPicker.current=null;};
  },[camera,control.mode,dragPicker,size.width,size.height]);
  useFrame(()=>{
    const g=group.current, a=anchor.current, b=body.current;
    if(!g)return;
    const robot=useTelemetry.getState().robot;
    const local=control.mode===2||control.frame==='ee';
    const fcYaw=robot?.fcYaw;
    g.visible=!!a&&!!b&&(local||Number.isFinite(fcYaw));
    if(!a||!b||!g.visible)return;
    a.getWorldPosition(scratch.position);
    const scale=eeGizmoScale(camera as PerspectiveCamera,scratch.position,size.width,size.height,control.scale);
    g.position.copy(g.parent!.worldToLocal(scratch.position));
    if(local){
      a.getWorldQuaternion(scratch.orientation);
      g.parent!.getWorldQuaternion(scratch.yaw).invert();
      g.quaternion.copy(scratch.yaw.multiply(scratch.orientation));
    }else{
      fcFrameOrientation(scratch.orientation,b,fcYaw!,robot!.worldRpy[2]);
      g.parent!.getWorldQuaternion(scratch.yaw).invert();
      g.quaternion.copy(scratch.yaw.multiply(scratch.orientation));
    }
    g.scale.setScalar(scale);
    g.visible=scale>0;
  });
  return <group ref={group}>
    {handles.map(a=>{
      const selected=control.axis===a.axis;
      const material={color:a.color,depthTest:false,depthWrite:false,transparent:true,opacity:control.highlight?(selected?1:.18):selected?1:.55};
      const data={eeAxis:a.axis};
      return <group key={a.axis}>
      <group quaternion={a.arrow}>
        <mesh position={[0,.6,0]} userData={{...data,eePickPadding:true}}>
          <cylinderGeometry args={[.23,.23,1.3,8]}/><meshBasicMaterial visible={false}/>
        </mesh>
        <mesh position={[0,.48,0]} renderOrder={100} userData={data}>
          <cylinderGeometry args={[selected?.075:.06,selected?.075:.06,.78,12]}/><meshBasicMaterial {...material}/>
        </mesh>
        <mesh position={[0,.98,0]} renderOrder={100} userData={data}>
          <coneGeometry args={[.20,.28,16]}/><meshBasicMaterial {...material}/>
        </mesh>
      </group>
      {control.mode===2&&selected&&<group quaternion={a.ring}>
        <mesh userData={{...data,eePickPadding:true}}>
          <torusGeometry args={[1.15,.20,8,40,Math.PI*1.5]}/><meshBasicMaterial visible={false}/>
        </mesh>
        <mesh renderOrder={100} userData={data}>
          <torusGeometry args={[1.15,.06,10,64,Math.PI*1.5]}/><meshBasicMaterial {...material}/>
        </mesh>
        <mesh position={[0,-1.15,0]} rotation={[0,0,-Math.PI/2]} renderOrder={101} userData={data}>
          <coneGeometry args={[.16,.25,16]}/><meshBasicMaterial {...material}/>
        </mesh>
      </group>}
      </group>;
    })}
    <mesh renderOrder={100}><sphereGeometry args={[.07,12,12]}/><meshBasicMaterial color={control.moving?'#f59e0b':'#ffffff'} depthTest={false}/></mesh>
  </group>;
}

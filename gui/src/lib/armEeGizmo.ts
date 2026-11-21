import type { EeMoveFrame } from './armManual';
import { Quaternion, Raycaster, Vector2, Vector3, type Camera, type PerspectiveCamera, type Group, type Object3D } from 'three';

export type EeAxis = 1 | 2 | 3;
export type EeMode = 1 | 2;
export type EeGizmoControl = {
  mode: EeMode; axis: EeAxis; moving: boolean; frame?: EeMoveFrame; scale?: number; highlight?: boolean;
  onSelect: (axis: EeAxis) => void;
};
export const EE_AXES = [
  { axis: 1 as const, label: 'X', rotation: 'Roll', color: '#e5484d', direction: [1, 0, 0] },
  { axis: 2 as const, label: 'Y', rotation: 'Pitch', color: '#30a46c', direction: [0, 0, -1] },
  { axis: 3 as const, label: 'Z', rotation: 'Yaw', color: '#3b82f6', direction: [0, 1, 0] },
];

const scalePosition = new Vector3();

export function eeGizmoScale(camera: PerspectiveCamera, position: Vector3, width: number, height: number, multiplier=1): number {
  if (![width,height].every(Number.isFinite) || width<=0 || height<=0) return 0;
  camera.updateMatrixWorld();
  const depth=-scalePosition.copy(position).applyMatrix4(camera.matrixWorldInverse).z;
  if (!Number.isFinite(depth) || depth<=0) return 0;
  scalePosition.copy(position).project(camera);
  const margin=Math.min((1+scalePosition.x)*width/2,(1-scalePosition.x)*width/2,
    (1+scalePosition.y)*height/2,(1-scalePosition.y)*height/2);
  const scale=Number.isFinite(multiplier)?Math.max(1,Math.min(2,multiplier)):1;
  const pixels=Math.max(0,Math.min(Math.min(width,height)*.18*scale,(margin-4)/1.32));
  return 2*depth*Math.tan(camera.fov*Math.PI/360)/camera.zoom/height*pixels;
}

export function pickEeAxis(camera: Camera, handles: Object3D, x: number, y: number): EeAxis | undefined {
  if (![x,y].every(Number.isFinite) || x < 0 || x > 1 || y < 0 || y > 1) return;
  const ray = new Raycaster();
  ray.setFromCamera(new Vector2(x * 2 - 1, 1 - y * 2), camera);
  handles.updateWorldMatrix(true, true);
  const hits=ray.intersectObject(handles,true).filter(hit=>hit.object.userData.eeAxis);
  return (hits.find(hit=>!hit.object.userData.eePickPadding)??hits[0])?.object.userData.eeAxis;
}

const wristX=new Vector3(1,0,0), wristY=new Vector3(0,1,0), wristZ=new Vector3(0,0,1), wristStep=new Quaternion();

export function gripperOrientation(out: Quaternion, joints: readonly number[], rba: boolean): Quaternion {
  const j = (i:number) => Number.isFinite(joints[i]) ? joints[i] : 0;
  const x=wristX, y=wristY, z=wristZ, q=wristStep;
  out.setFromAxisAngle(y,j(0));
  out.multiply(q.setFromAxisAngle(z,-j(1))).multiply(q.setFromAxisAngle(z,-j(2)));
  out.multiply(q.setFromAxisAngle(rba?x:y,j(3))).multiply(q.setFromAxisAngle(z,-j(4)));
  return out.multiply(q.setFromAxisAngle(rba?x:y,j(5)));
}

export function fcFrameOrientation(out: Quaternion, body: Group, fcYaw: number, bodyYaw: number): Quaternion {
  body.parent!.getWorldQuaternion(out);
  return out.multiply(wristStep.setFromAxisAngle(wristY, body.rotation.y + fcYaw - bodyYaw));
}

export type EeDragProjection = {
  axis: EeAxis; mode: EeMode; vertical: boolean;
  origin:{x:number;y:number};center:{x:number;y:number};direction:{x:number;y:number};facing:number;
  project: (x:number,y:number) => number;
};

export function beginEeDrag(camera: Camera, handles: Object3D, mode:EeMode, x:number,y:number,width:number,height:number):EeDragProjection|undefined {
  const axis=pickEeAxis(camera,handles,x,y);
  if(!axis||![width,height].every(Number.isFinite)||width<=0||height<=0)return;
  const center=handles.getWorldPosition(new Vector3());
  const normal=new Vector3(...EE_AXES[axis-1].direction).applyQuaternion(handles.getWorldQuaternion(new Quaternion()));
  const c=center.clone().project(camera);
  const tip=center.clone().addScaledVector(normal,handles.getWorldScale(new Vector3()).x).project(camera);
  const origin=new Vector2(x*width,y*height), screenCenter=new Vector2((c.x+1)*width/2,(1-c.y)*height/2);
  const direction=new Vector2((tip.x-c.x)*width/2,-(tip.y-c.y)*height/2);
  const facing=normal.dot(camera.getWorldPosition(new Vector3()).sub(center).normalize());
  const vertical=mode===1?direction.length()<8:Math.abs(facing)<.2||origin.distanceTo(screenCenter)<12;
  if(vertical)direction.set(0,-1);else direction.normalize();
  let previous=Math.atan2(screenCenter.y-origin.y,origin.x-screenCenter.x), accumulated=0, valid=true;
  const radius=Math.max(24,origin.distanceTo(screenCenter));
  return {axis,mode,vertical,origin:{x:origin.x,y:origin.y},center:{x:screenCenter.x,y:screenCenter.y},direction:{x:direction.x,y:direction.y},facing,project:(nx,ny)=>{
    if(![nx,ny].every(Number.isFinite))return 0;
    const point=new Vector2(nx*width,ny*height);
    if(mode===1||vertical)return point.sub(origin).dot(direction);
    const angle=Math.atan2(screenCenter.y-point.y,point.x-screenCenter.x);
    if(point.distanceTo(screenCenter)<12){valid=false;accumulated=0;return 0;}
    if(!valid){valid=true;previous=angle;return 0;}
    accumulated+=Math.atan2(Math.sin(angle-previous),Math.cos(angle-previous));previous=angle;
    if(point.distanceTo(origin)<8)accumulated=0;
    return accumulated*radius*Math.sign(facing);
  }};
}

export function eeDragSign(distance:number):-1|0|1 {
  return !Number.isFinite(distance)||Math.abs(distance)<8?0:distance>0?1:-1;
}

export function armOverviewRect(width:number,height:number) {
  const w=Math.max(0,Math.min(width*.3,210)),h=Math.max(0,Math.min(height*.3,150));
  return {left:Math.max(0,width-w-10),top:10,width:w,height:h};
}

export function displayHeading(yaw:number, origin?:number) {
  return origin===undefined?yaw:Math.atan2(Math.sin(yaw-origin),Math.cos(yaw-origin));
}
export function displayPosition(position:[number,number,number], origin?:number):[number,number,number] {
  if(origin===undefined)return position;
  const [x,y,z]=position,c=Math.cos(origin),s=Math.sin(origin);
  return [c*x+s*y,-s*x+c*y,z];
}

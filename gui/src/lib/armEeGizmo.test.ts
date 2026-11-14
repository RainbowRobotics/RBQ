import { describe, it, expect } from 'vitest';
import { parseRobotState } from './robotState';
import { Group, Mesh, MeshBasicMaterial, PerspectiveCamera, Quaternion, SphereGeometry, Vector3 } from 'three';
import { displayHeading, displayPosition, EE_AXES, armOverviewRect, eeGizmoScale, fcFrameOrientation, beginEeDrag, eeDragSign, gripperOrientation, pickEeAxis } from './armEeGizmo';

describe('EE handle selection',()=>{
  it('maps robot X/Y/Z into the existing Y-up scene without swapping axes',()=>{
    expect(EE_AXES.map(a=>[a.axis,a.direction])).toEqual([[1,[1,0,0]],[2,[0,0,-1]],[3,[0,1,0]]]);
  });
  it('picks only tagged handles and rejects taps outside the viewport',()=>{
    const camera=new PerspectiveCamera(50,1,.01,20);
    camera.position.set(0,0,3);camera.lookAt(0,0,0);camera.updateMatrixWorld();
    const group=new Group(),geometry=new SphereGeometry(.12),material=new MeshBasicMaterial();
    for(const axis of [1,2,3]){
      const mesh=new Mesh(geometry,material);mesh.position.x=(axis-2)*.5;mesh.userData.eeAxis=axis;group.add(mesh);
    }
    for(const axis of [1,2,3]){
      const point=new Vector3((axis-2)*.5,0,0).project(camera);
      expect(pickEeAxis(camera,group,(point.x+1)/2,(1-point.y)/2)).toBe(axis);
    }
    expect(pickEeAxis(camera,group,NaN,.5)).toBeUndefined();
    expect(pickEeAxis(camera,group,-.1,.5)).toBeUndefined();
    expect(pickEeAxis(camera,group,.5,0)).toBeUndefined();
    const padding=new Mesh(geometry,material);padding.position.z=1;
    padding.userData={eeAxis:3,eePickPadding:true};group.add(padding);
    expect(pickEeAxis(camera,group,.5,.5)).toBe(2);
    group.remove(padding);
    group.children.forEach(m=>delete m.userData.eeAxis);
    expect(pickEeAxis(camera,group,.5,.5)).toBeUndefined();
    geometry.dispose();material.dispose();
  });
});

describe('measured gripper orientation',()=>{
  it('follows measured shoulder yaw, and distinguishes RB1 from RBA wrist roll axes',()=>{
    const out=new Quaternion(), up=new Vector3(0,1,0), forward=new Vector3(1,0,0);
    gripperOrientation(out,[Math.PI/2,0,0,0,0,0],false);
    expect(forward.clone().applyQuaternion(out).distanceTo(new Vector3(0,0,-1))).toBeLessThan(1e-10);
    gripperOrientation(out,[0,0,0,0,0,Math.PI/2],false);
    expect(up.clone().applyQuaternion(out).distanceTo(up)).toBeLessThan(1e-10);
    gripperOrientation(out,[0,0,0,0,0,Math.PI/2],true);
    expect(up.clone().applyQuaternion(out).distanceTo(new Vector3(0,0,1))).toBeLessThan(1e-10);
  });
  it('rejects nonfinite joints rather than losing the entire model',()=>{
    const q=gripperOrientation(new Quaternion(),[NaN,Infinity],false);
    expect(q.toArray()).toEqual([0,0,0,1]);
  });
});

describe('screen drag projection',()=>{
 const sphere=new SphereGeometry(.12),material=new MeshBasicMaterial();
 function setup(axis:1|2|3,position:Vector3,cameraPosition:Vector3){
  const camera=new PerspectiveCamera(50,1,.01,20);camera.position.copy(cameraPosition);
  if(cameraPosition.y===3)camera.up.set(0,0,-1);
  camera.lookAt(0,0,0);camera.updateMatrixWorld();
  const group=new Group(),mesh=new Mesh(sphere,material);mesh.position.copy(position);mesh.userData.eeAxis=axis;group.add(mesh);
  const screen=position.clone().project(camera);
  return {camera,group,x:(screen.x+1)/2,y:(1-screen.y)/2};
 }
 it('starts neutral, ignores cross-axis motion, and freezes the grabbed axis while telemetry rotates',()=>{
  const s=setup(1,new Vector3(.8,0,0),new Vector3(0,0,3));
  const drag=beginEeDrag(s.camera,s.group,1,s.x,s.y,200,200)!;
  expect(drag.axis).toBe(1);expect(eeDragSign(drag.project(s.x,s.y))).toBe(0);
  expect(eeDragSign(drag.project(s.x+.02,s.y))).toBe(0);
  expect(eeDragSign(drag.project(s.x,s.y+.2))).toBe(0);
  expect(eeDragSign(drag.project(s.x+.1,s.y))).toBe(1);
  s.group.rotation.z=Math.PI/2;
  expect(eeDragSign(drag.project(s.x-.1,s.y))).toBe(-1);
  expect(eeDragSign(drag.project(NaN,s.y))).toBe(0);
 });
 it('uses the positive right-hand rotation direction and stops at the circle center',()=>{
  const s=setup(3,new Vector3(.8,0,0),new Vector3(0,3,0));
  const drag=beginEeDrag(s.camera,s.group,2,s.x,s.y,200,200)!;
  expect(drag.vertical).toBe(false);
  const up=new Vector3(0,0,-.8).project(s.camera);
  expect(eeDragSign(drag.project((up.x+1)/2,(1-up.y)/2))).toBe(1);
  expect(eeDragSign(drag.project(.5,.5))).toBe(0);
  expect(eeDragSign(drag.project(s.x,s.y))).toBe(0);
 });
 it('uses an explicit vertical fallback for a view-aligned axis and rejects empty grabs',()=>{
  const s=setup(1,new Vector3(.8,0,0),new Vector3(3,0,0));
  const drag=beginEeDrag(s.camera,s.group,1,s.x,s.y,200,200)!;
  expect(drag.vertical).toBe(true);
  expect(eeDragSign(drag.project(s.x,s.y-.1))).toBe(1);
  expect(eeDragSign(drag.project(s.x,s.y+.1))).toBe(-1);
  expect(beginEeDrag(s.camera,s.group,1,0,0,200,200)).toBeUndefined();
  expect(beginEeDrag(s.camera,s.group,1,s.x,s.y,0,200)).toBeUndefined();
 });
});


describe('FC frame display', () => {
  it('includes foot twist, ignores body tilt and retains viewer rotation in both model views', () => {
    for (const displayedBodyYaw of [0, 1.2]) {
      const viewer = new Group(), body = new Group();
      viewer.rotation.set(0.3, -0.7, 0.1);
      body.rotation.set(0.4, displayedBodyYaw, -0.25, 'YXZ');
      viewer.add(body); viewer.updateMatrixWorld(true);
      const world = fcFrameOrientation(new Quaternion(), body, 0.9, 1.2);
      const relative = viewer.getWorldQuaternion(new Quaternion()).invert().multiply(world);
      const x = new Vector3(1, 0, 0).applyQuaternion(relative);
      const yaw = displayedBodyYaw - 0.3;
      expect(x.distanceTo(new Vector3(Math.cos(yaw), 0, -Math.sin(yaw)))).toBeLessThan(1e-10);
      expect(new Vector3(0, 1, 0).applyQuaternion(relative).distanceTo(new Vector3(0, 1, 0))).toBeLessThan(1e-10);
      const local = body.getWorldQuaternion(new Quaternion()).invert().multiply(world);
      const rendered = body.getWorldQuaternion(new Quaternion()).multiply(local);
      expect(rendered.angleTo(world)).toBeLessThan(1e-7);
    }
  });
});


describe('viewport-proportional EE axes', () => {
  it('keeps the same projected proportion across panel aspect ratios, depths, FOV and zoom', () => {
    for (const [width,height] of [[160,120],[180,260],[460,240],[1000,600]]) {
      for (const [depth,fov,zoom] of [[1,40,1],[5,65,1],[3,40,2]]) {
        const camera=new PerspectiveCamera(fov,width/height,.01,100);
        camera.zoom=zoom;camera.updateProjectionMatrix();
        const origin=new Vector3(.02,.01,-depth);
        const scale=eeGizmoScale(camera,origin,width,height);
        const a=origin.clone().project(camera), b=origin.clone().add(new Vector3(scale,0,0)).project(camera);
        expect((b.x-a.x)*width/2).toBeCloseTo(Math.min(width,height)*.18,8);
      }
    }
  });
  it('applies the chosen display scale and keeps the enlarged axes inside the viewport',()=>{
    const camera=new PerspectiveCamera(50,1,.01,100),origin=new Vector3(0,0,-2);
    const base=eeGizmoScale(camera,origin,400,400);
    expect(eeGizmoScale(camera,origin,400,400,1.4)).toBeCloseTo(base*1.4,8);
    expect(eeGizmoScale(camera,origin,400,400,2)).toBeCloseTo(base*2,8);
    expect(eeGizmoScale(camera,origin,400,400,NaN)).toBe(base);
    expect(eeGizmoScale(camera,origin,400,400,-1)).toBe(base);
    expect(eeGizmoScale(camera,origin,400,400,10)).toBeCloseTo(base*2,8);
    const edgeOrigin=new Vector3(.7,0,-2);
    const scale=eeGizmoScale(camera,edgeOrigin,400,400,2);
    const tip=edgeOrigin.clone().add(new Vector3(scale*1.32,0,0)).project(camera);
    expect((tip.x+1)*200).toBeLessThanOrEqual(396.000001);
  });
  it('fits the rotation-ring radius inside the nearest screen edge', () => {
    const camera=new PerspectiveCamera(40,1,.01,100);
    const origin=new Vector3(.3,0,-1);
    const scale=eeGizmoScale(camera,origin,200,200);
    const center=origin.clone().project(camera), edge=origin.clone().add(new Vector3(scale*1.32,0,0)).project(camera);
    expect(scale).toBeGreaterThan(0);
    expect((edge.x+1)*100).toBeCloseTo(196,8);
    expect((edge.x-center.x)*100/1.32).toBeLessThan(36);
    expect(eeGizmoScale(camera,new Vector3(1,0,-1),200,200)).toBe(0);
  });
  it('does not produce an invalid scale for a zero-sized panel or an EE behind the camera', () => {
    const camera=new PerspectiveCamera();
    expect(eeGizmoScale(camera,new Vector3(0,0,-1),0,200)).toBe(0);
    expect(eeGizmoScale(camera,new Vector3(0,0,-1),200,NaN)).toBe(0);
    expect(eeGizmoScale(camera,new Vector3(0,0,1),200,200)).toBe(0);
  });
});


describe('live robot PIP placement',()=>{
  it('fits the PIP inside the main view at deck and phone sizes',()=>{
    for(const [width,height] of [[640,440],[380,260],[350,170],[280,120]]){
      const r=armOverviewRect(width,height);
      expect(r.left).toBeGreaterThanOrEqual(0);expect(r.top).toBeGreaterThanOrEqual(0);
      expect(r.left+r.width).toBeLessThanOrEqual(width);
      expect(r.top+r.height).toBeLessThanOrEqual(height);
      expect(r.width).toBeLessThanOrEqual(width*.3);
      expect(r.height).toBeLessThanOrEqual(height*.3);
    }
  });
});

describe('FC telemetry to rendered axis after a control-world yaw reset',()=>{
 it('preserves foot twist across odometry headings, viewer rotations and local view',()=>{
  for(const [bodyYaw,bits] of [[0,0],[1,0x3c00],[-2,0xc000]])for(const local of [false,true]){
   const buf=new ArrayBuffer(592),dv=new DataView(buf);dv.setUint16(138,bits,true);dv.setUint16(586,0xFC02,true);dv.setInt16(584,-2500,true);
   const robot=parseRobotState(buf),viewer=new Group(),body=new Group();viewer.rotation.set(.3,-.7,.1);body.rotation.set(.2,local?0:bodyYaw,-.3,'YXZ');viewer.add(body);viewer.updateMatrixWorld(true);
   const world=fcFrameOrientation(new Quaternion(),body,robot.fcYaw!,robot.worldRpy[2]);
   const relative=viewer.getWorldQuaternion(new Quaternion()).invert().multiply(world);
   const yaw=(local?0:bodyYaw)-.25;
   expect(new Vector3(1,0,0).applyQuaternion(relative).distanceTo(new Vector3(Math.cos(yaw),0,-Math.sin(yaw)))).toBeLessThan(1e-10);
  }
 });
});

describe('FC-aligned display frame',()=>{
 it('follows the current FC heading and wraps the pi boundary',()=>{
  for(const origin of [-3,-1,0,2,3]){
   expect(displayHeading(origin,origin)).toBe(0);
   expect(displayHeading(origin+.4,origin)).toBeCloseTo(.4);
  }
  expect(displayHeading(-3.1,3.1)).toBeCloseTo(2*Math.PI-6.2);
  expect(displayHeading(2)).toBe(2);
 });
 it('rotates world movement into the same heading frame without changing height or telemetry',()=>{
  const p:[number,number,number]=[2,3,.5];
  expect(displayPosition(p)).toBe(p);
  expect(displayPosition(p,Math.PI/2)[0]).toBeCloseTo(3);
  expect(displayPosition(p,Math.PI/2)[1]).toBeCloseTo(-2);
  expect(displayPosition(p,Math.PI/2)[2]).toBe(.5);expect(p).toEqual([2,3,.5]);
 });
 it('rebases FC and measured EE together while preserving the real wrist orientation',()=>{
  const origin=2,viewer=new Group(),body=new Group(),wrist=new Group();viewer.add(body);body.add(wrist);
  viewer.rotation.set(.35,-.7,0);body.rotation.set(.2,displayHeading(2.4,origin),-.1,'YXZ');
  wrist.quaternion.copy(gripperOrientation(new Quaternion(),[.8,-.2,.4,0,.1,.3],false));viewer.updateMatrixWorld(true);
  const fc=fcFrameOrientation(new Quaternion(),body,2.15,2.4);
  const relative=viewer.getWorldQuaternion(new Quaternion()).invert().multiply(fc);
  expect(relative.angleTo(new Quaternion().setFromAxisAngle(new Vector3(0,1,0),.15))).toBeLessThan(1e-7);
  const measured=body.getWorldQuaternion(new Quaternion()).invert().multiply(wrist.getWorldQuaternion(new Quaternion()));
  expect(measured.angleTo(wrist.quaternion)).toBeLessThan(1e-7);
 });
});

describe('live FC display reference',()=>{
 it('keeps FC axes at view yaw zero for every body/FC yaw and retains relative body/EE orientation',()=>{
  for(const fcYaw of [-3,-1,0,2,3])for(const bodyYaw of [-2,0,1,2.5]){
   const viewer=new Group(),body=new Group(),wrist=new Group();viewer.rotation.set(.35,-.7,0);viewer.add(body);body.add(wrist);
   body.rotation.set(.2,displayHeading(bodyYaw,fcYaw),-.3,'YXZ');
   wrist.quaternion.copy(gripperOrientation(new Quaternion(),[.8,-.2,.4,0,.1,.3],false));viewer.updateMatrixWorld(true);
   const fc=fcFrameOrientation(new Quaternion(),body,fcYaw,bodyYaw);
   expect(viewer.getWorldQuaternion(new Quaternion()).invert().multiply(fc).angleTo(new Quaternion())).toBeLessThan(1e-7);
   expect(body.rotation.y).toBeCloseTo(Math.atan2(Math.sin(bodyYaw-fcYaw),Math.cos(bodyYaw-fcYaw)));
   const localWrist=body.getWorldQuaternion(new Quaternion()).invert().multiply(wrist.getWorldQuaternion(new Quaternion()));
   expect(localWrist.angleTo(wrist.quaternion)).toBeLessThan(1e-7);
  }
 });
 it('preserves the relative scene when both headings turn and reflects body-only rotation',()=>{
  expect(displayHeading(1, .75)).toBeCloseTo(displayHeading(2,1.75));
  expect(displayHeading(1.5,.75)).toBeCloseTo(.75);
 });
});

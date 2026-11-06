import { useMemo, useEffect, type MutableRefObject } from 'react';
import { Box3, Color, Group, OrthographicCamera, Vector3 } from 'three';
import { useFrame, useThree } from '@/lib/r3fCanvas';
import { armOverviewRect } from '@/lib/armEeGizmo';

export function ArmRobotOverview({robot}:{robot:MutableRefObject<Group|null>}) {
  const {gl,scene,camera,size}=useThree();
  const scratch=useMemo(()=>({box:new Box3(),center:new Vector3(),extent:new Vector3(),direction:new Vector3(1,.65,1).normalize(),color:new Color(),
    mini:new OrthographicCamera(-1,1,1,-1,.01,100)}),[]);
  useEffect(()=>()=>{gl.setScissorTest(false);gl.setViewport(0,0,size.width,size.height);},[gl,size]);
  useFrame(()=>{
    const {width,height}=size;
    if(width<=0||height<=0)return;
    const {mini,box,center,extent,color}=scratch;
    const rect=armOverviewRect(width,height);
    const autoClear=gl.autoClear;
    gl.autoClear=true;gl.setScissorTest(false);gl.setViewport(0,0,width,height);
    gl.render(scene,camera);
    if(robot.current){
      box.setFromObject(robot.current);
      if(!box.isEmpty()){
        box.getCenter(center);box.getSize(extent);
        const radius=Math.max(.3,extent.length()/2)*1.14;
        const aspect=rect.width/rect.height;
        mini.left=-radius*Math.max(1,aspect);mini.right=-mini.left;
        mini.top=radius*Math.max(1,1/aspect);mini.bottom=-mini.top;
        mini.position.copy(center).addScaledVector(scratch.direction,radius*3);
        mini.lookAt(center);mini.layers.set(1);mini.updateProjectionMatrix();mini.updateMatrixWorld();
        const alpha=gl.getClearAlpha();gl.getClearColor(color);
        const background=scene.background;
        scene.background=null;gl.setClearColor(background instanceof Color?background:color,1);
        gl.setScissorTest(true);gl.setViewport(rect.left,height-rect.top-rect.height,rect.width,rect.height);
        gl.setScissor(rect.left,height-rect.top-rect.height,rect.width,rect.height);
        gl.render(scene,mini);
        scene.background=background;gl.setClearColor(color,alpha);
      }
    }
    gl.autoClear=autoClear;gl.setScissorTest(false);gl.setViewport(0,0,width,height);
  },1);
  return null;
}

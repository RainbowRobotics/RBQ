import type { EeDragProjection } from './armEeGizmo';

export type EeFeedbackStyle = 'glow'|'touch'|'ribbon'|'both';
export type EeFeedbackState = {grab:EeDragProjection;point:{x:number;y:number};sign:-1|0|1};

export function eeFeedbackPath(grab:EeDragProjection,sign:-1|1,width:number,height:number) {
  const {center,origin,direction}=grab;
  if(![width,height,center.x,center.y,origin.x,origin.y,direction.x,direction.y,grab.facing].every(Number.isFinite)||width<=0||height<=0)return null;
  let from=grab.mode===2&&grab.vertical?origin:center;
  let tip:{x:number;y:number},tangent:{x:number;y:number},path:string;
  if(grab.mode===2&&!grab.vertical){
    const radius=Math.min(80,Math.max(24,Math.hypot(origin.x-center.x,origin.y-center.y)));
    const angle=Math.atan2(origin.y-center.y,origin.x-center.x);
    const turn=-sign*Math.sign(grab.facing||1),end=angle+turn*Math.PI*.65;
    from={x:center.x+Math.cos(angle)*radius,y:center.y+Math.sin(angle)*radius};
    tip={x:center.x+Math.cos(end)*radius,y:center.y+Math.sin(end)*radius};
    tangent={x:-Math.sin(end)*turn,y:Math.cos(end)*turn};
    path=`M${from.x} ${from.y} A${radius} ${radius} 0 0 ${turn>0?1:0} ${tip.x} ${tip.y}`;
  }else{
    const n=Math.hypot(direction.x,direction.y);
    if(n<1e-6)return null;
    tangent={x:direction.x/n*sign,y:direction.y/n*sign};
    let length=Math.min(110,Math.min(width,height)*.28);
    const {x,y}=tangent;
    if(x>1e-6)length=Math.min(length,(width-16-from.x)/x);
    if(x<-1e-6)length=Math.min(length,(16-from.x)/x);
    if(y>1e-6)length=Math.min(length,(height-16-from.y)/y);
    if(y<-1e-6)length=Math.min(length,(16-from.y)/y);
    if(length<18)return null;
    tip={x:from.x+x*length,y:from.y+y*length};
    path=`M${from.x} ${from.y} L${tip.x} ${tip.y}`;
  }
  const head=`${tip.x},${tip.y} ${tip.x-tangent.x*19-tangent.y*12},${tip.y-tangent.y*19+tangent.x*12} ${tip.x-tangent.x*19+tangent.y*12},${tip.y-tangent.y*19-tangent.x*12}`;
  return {path,head,tip};
}

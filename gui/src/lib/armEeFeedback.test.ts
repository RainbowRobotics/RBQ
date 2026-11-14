import { describe,it,expect } from 'vitest';
import type { EeDragProjection } from './armEeGizmo';
import { eeFeedbackPath } from './armEeFeedback';
const grab:EeDragProjection={axis:1,mode:1,vertical:false,center:{x:150,y:150},origin:{x:210,y:150},direction:{x:1,y:0},facing:1,project:()=>0};
describe('EE input direction graphics',()=>{
  it('reverses the screen direction without using drag distance as travel distance',()=>{
    const plus=eeFeedbackPath(grab,1,400,300)!,minus=eeFeedbackPath(grab,-1,400,300)!;
    expect(plus.tip.x).toBeGreaterThan(grab.center.x);expect(minus.tip.x).toBeLessThan(grab.center.x);
    expect(plus.tip.y).toBe(grab.center.y);
    expect(eeFeedbackPath({...grab,origin:{x:290,y:150}},1,400,300)!.tip).toEqual(plus.tip);
  });
  it('fits a straight indicator near the edge, and rejects invalid or too short views',()=>{
    const edge=eeFeedbackPath({...grab,center:{x:350,y:150}},1,400,300)!;
    expect(edge.tip.x).toBe(384);
    expect(eeFeedbackPath(grab,1,NaN,300)).toBeNull();
    expect(eeFeedbackPath({...grab,direction:{x:0,y:0}},1,400,300)).toBeNull();
    expect(eeFeedbackPath({...grab,center:{x:390,y:150}},1,400,300)).toBeNull();
  });
  it('matches the positive rotation sign on each side of the rotation axis',()=>{
    const rotate={...grab,mode:2 as const};
    const positive=eeFeedbackPath(rotate,1,400,300)!,negative=eeFeedbackPath(rotate,-1,400,300)!;
    expect(positive.path).toContain(' A');expect(positive.tip.y).toBeLessThan(grab.center.y);
    expect(negative.tip.y).toBeGreaterThan(grab.center.y);
    expect(eeFeedbackPath({...rotate,facing:-1},1,400,300)!.tip).toEqual(negative.tip);
  });
  it('shows the same vertical fallback as the actual gesture interpreter',()=>{
    const fallback={...grab,mode:2 as const,vertical:true,direction:{x:0,y:-1}};
    expect(eeFeedbackPath(fallback,1,400,300)!.tip.y).toBeLessThan(grab.origin.y);
    expect(eeFeedbackPath(fallback,-1,400,300)!.tip.y).toBeGreaterThan(grab.origin.y);
  });
});

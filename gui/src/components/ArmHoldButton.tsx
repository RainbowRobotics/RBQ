import { useEffect, useRef } from 'react';
import { Platform, Pressable, View, type StyleProp, type ViewStyle } from 'react-native';
import type { ReactNode } from 'react';

export function ArmHoldButton({label,disabled,held,onStart,onEnd,style,children}:{
  label:string;disabled:boolean;held:boolean;onStart:()=>void;onEnd:()=>void;
  style:StyleProp<ViewStyle>;children:ReactNode;
}) {
  const node=useRef<View>(null);
  const callbacks=useRef({disabled,onStart,onEnd});callbacks.current={disabled,onStart,onEnd};
  const cancel=useRef<(()=>void)|null>(null);
  useEffect(()=>{
    if(Platform.OS!=='web')return;
    const el=node.current as unknown as HTMLElement;
    if(!el)return;
    let pointer:number|null=null,key:string|null=null;
    const end=()=>{
      if(pointer===null&&key===null)return;
      const id=pointer;pointer=null;key=null;
      callbacks.current.onEnd();
      if(id!==null&&el.hasPointerCapture(id))el.releasePointerCapture(id);
    };
    cancel.current=end;
    const down=(e:PointerEvent)=>{
      if(!e.isPrimary){end();return;}
      if(callbacks.current.disabled||e.button!==0||pointer!==null||key!==null)return;
      e.preventDefault();
      try{el.setPointerCapture(e.pointerId);}catch{return;}
      pointer=e.pointerId;el.focus({preventScroll:true});callbacks.current.onStart();
    };
    const up=(e:PointerEvent)=>{if(e.pointerId===pointer)end();};
    const otherPointer=(e:PointerEvent)=>{if(pointer!==null&&e.pointerId!==pointer)end();};
    const hidden=()=>{if(document.hidden)end();};
    const context=(e:Event)=>e.preventDefault();
    const keydown=(e:KeyboardEvent)=>{
      if(e.key==='Escape'){end();return;}
      if(e.key!==' '&&e.key!=='Enter')return;
      e.preventDefault();
      if(e.repeat||callbacks.current.disabled||pointer!==null||key!==null)return;
      key=e.key;callbacks.current.onStart();
    };
    const keyup=(e:KeyboardEvent)=>{if(e.key===key){e.preventDefault();end();}};
    el.addEventListener('pointerdown',down);
    el.addEventListener('pointerup',up);
    el.addEventListener('pointercancel',up);
    el.addEventListener('lostpointercapture',up);
    el.addEventListener('keydown',keydown);el.addEventListener('keyup',keyup);
    el.addEventListener('blur',end);el.addEventListener('contextmenu',context);
    document.addEventListener('pointerdown',otherPointer,true);
    document.addEventListener('visibilitychange',hidden);window.addEventListener('blur',end);
    return()=>{
      end();cancel.current=null;
      el.removeEventListener('pointerdown',down);el.removeEventListener('pointerup',up);
      el.removeEventListener('pointercancel',up);el.removeEventListener('lostpointercapture',up);
      el.removeEventListener('keydown',keydown);el.removeEventListener('keyup',keyup);
      el.removeEventListener('blur',end);el.removeEventListener('contextmenu',context);
      document.removeEventListener('pointerdown',otherPointer,true);
      document.removeEventListener('visibilitychange',hidden);window.removeEventListener('blur',end);
    };
  },[]);
  useEffect(()=>{if(disabled||!held)cancel.current?.();},[disabled,held]);
  const accessibility={accessibilityRole:'button' as const,accessibilityLabel:label,accessibilityState:{disabled}};
  return Platform.OS==='web'
    ?<View ref={node} {...accessibility} aria-disabled={disabled} aria-pressed={held} tabIndex={disabled?-1:0} style={style}>{children}</View>
    :<Pressable {...accessibility} disabled={disabled} onPressIn={onStart} onPressOut={onEnd} style={style}>{children}</Pressable>;
}

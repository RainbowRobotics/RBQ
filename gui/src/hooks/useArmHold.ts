import { useCallback, useEffect, useMemo, useState } from 'react';
import { AppState, Platform } from 'react-native';
import { useFocusEffect } from 'expo-router';
import { useRobot } from '@/store/robot';
import { createArmHold } from '@/lib/armManual';
import type { CommandOptions } from '@/lib/commandBus';

export function useArmHold<T>(send: (command:T, options?:CommandOptions)=>Promise<unknown>, enabled:boolean) {
  const [error,setError]=useState('');
  const hold=useMemo(()=>createArmHold(send,e=>setError(e instanceof Error?e.message:String(e))),[send]);
  useFocusEffect(useCallback(()=>()=>hold.stop(),[hold]));
  useEffect(()=>{
    const stop=()=>hold.stop();
    const hidden=()=>{if(document.hidden)stop();};
    const app=AppState.addEventListener('change',state=>{if(state!=='active')stop();});
    const unsubscribe=useRobot.subscribe(s=>{if(s.conn!=='connected')hold.cancel();});
    if(Platform.OS==='web'){window.addEventListener('blur',stop);document.addEventListener('visibilitychange',hidden);}
    return ()=>{
      stop(); app.remove(); unsubscribe();
      if(Platform.OS==='web'){window.removeEventListener('blur',stop);document.removeEventListener('visibilitychange',hidden);}
    };
  },[hold]);
  useEffect(()=>{if(!enabled)hold.stop();},[enabled,hold]);
  return {hold,error};
}

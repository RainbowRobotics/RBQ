import { useEffect, useRef, useState } from 'react';
import { useRouter, useNavigation } from 'expo-router';
import { usePreventRemove } from 'expo-router/react-navigation';
import { Screen } from '@/components/Screen';
import { HubHeader } from '@/components/hub/HubHeader';
import { ArmManualWorkspace, type ArmManualHandle } from '@/components/panels/ArmManualWorkspace';
import { t } from '@/lib/i18n';
import { useRobot } from '@/store/robot';
export default function ArmManual() {
  const router=useRouter(), navigation=useNavigation();
  const workspace=useRef<ArmManualHandle>(null), leaving=useRef(false);
  const [next,setNext]=useState<(()=>void)|null>(null);
  const disconnected=useRef(false);
  useEffect(()=>{
    const check=()=>{
      if(useRobot.getState().conn==='connected' || disconnected.current)return;
      disconnected.current=true;
      leaving.current=true;
      setNext(()=>()=>router.replace({pathname:'/hub',params:{armInterrupted:'1'}}));
    };
    const unsubscribe=useRobot.subscribe(check);
    check();
    return unsubscribe;
  },[router]);
  const leave=async(action:()=>void)=>{
    if(leaving.current)return;
    leaving.current=true;
    const ready=await workspace.current?.prepareExit();
    if(disconnected.current)return;
    if(ready)setNext(()=>action);
    else leaving.current=false;
  };
  usePreventRemove(!next,({data})=>{void leave(()=>navigation.dispatch(data.action));});
  useEffect(()=>{next?.();},[next]);
  return <Screen><HubHeader title={t('수동제어')} subtitle="" beforeNavigate={action=>void leave(action)}/>
    <ArmManualWorkspace ref={workspace} onExit={()=>void leave(()=>router.replace('/'))}/></Screen>;
}

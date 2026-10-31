import { useCallback, useEffect, useImperativeHandle, useMemo, useRef, useState, type Ref } from 'react';
import { AppState, PanResponder, Platform, ScrollView, StyleSheet, Text, View, useWindowDimensions } from 'react-native';
import { useFocusEffect } from 'expo-router';
import { useTheme } from '@/theme';
import { Segmented, Slider } from '@/components/ui/controls';
import { web } from '@/rb/components/web';
import { Tappable } from '@/components/anim';
import { CameraView } from '@/components/CameraView';
import { Popover } from '@/components/ui/overlays';
import { useSettings } from '@/store/settings';
import { ArmHoldButton } from '@/components/ArmHoldButton';
import { RobotModel3D } from '@/components/RobotModel3D';
import { useRobot } from '@/store/robot';
import { useTelemetry } from '@/store/telemetry';
import { armAvailable } from '@/lib/armAvailability';
import { simulatedRobot } from '@/lib/simulatedRobot';
import { useArmAvailable } from '@/lib/useArmAvailable';
import { armManualReady, createArmDrag, createArmHold, manualCommands as cmd, sendArmManual, type ArmManualCommand, type EeMoveFrame } from '@/lib/armManual';
import { ARM_MANUAL_STREAM_ID } from '@/lib/visionSources';
import { EE_AXES, type EeAxis, type EeMode } from '@/lib/armEeGizmo';
import { EE_SPEED_DEFAULTS, eeSpeedPercent } from '@/lib/armEeSpeed';
import { MC } from '@/lib/arm';
import { t } from '@/lib/i18n';

export type ArmManualHandle = { prepareExit: () => Promise<boolean> };
export function ArmManualWorkspace({onExit, ref}:{onExit:()=>void; ref?:Ref<ArmManualHandle>}) {
  const {c,radius}=useTheme();
  const {width,height}=useWindowDimensions();
  const rootRef=useRef<View>(null),speedAnchor=useRef<View>(null);
  const [speedPosition,setSpeedPosition]=useState({left:12,top:64,maxHeight:300});
  const compact=height<600, narrow=width<720;
  const [mainShare,setMainShare]=useState(.46);
  const [viewSize,setViewSize]=useState({width:0,height:0});
  const [mainView,setMainView]=useState<'camera'|'model'>('camera');
  const [eeView,setEeView]=useState<'close'|'full'>('close');
  const [axisSettings,setAxisSettings]=useState(false),[speedSettings,setSpeedSettings]=useState(false);
  const savedMoveSpeed=useSettings(s=>s.armEeMoveSpeed),savedRotateSpeed=useSettings(s=>s.armEeRotateSpeed);
  const setEeSpeed=useSettings(s=>s.setArmEeSpeed);
  const moveSpeed=eeSpeedPercent(savedMoveSpeed,EE_SPEED_DEFAULTS.move),rotateSpeed=eeSpeedPercent(savedRotateSpeed,EE_SPEED_DEFAULTS.rotate);
  const savedEeScale=useSettings(s=>s.armEeScale),setEeScale=useSettings(s=>s.setArmEeScale);
  const eeFeedbackStyle=useSettings(s=>s.armEeFeedback),setEeFeedbackStyle=useSettings(s=>s.setArmEeFeedback);
  const eeScale=Number.isFinite(savedEeScale)?Math.max(100,Math.min(200,savedEeScale)):140;
  const availableWidth=Math.max(1,viewSize.width-30);
  const minShare=.32;
  const maxShare=Math.max(minShare,Math.min(.68,1-320/availableWidth));
  const share=Math.max(minShare,Math.min(maxShare,mainShare));
  const split=useRef({width:0,share,start:share,min:minShare,max:maxShare,active:false});
  Object.assign(split.current,{width:availableWidth,share,min:minShare,max:maxShare});
  const controlHeight=44,jogHeight=compact?44:56;
  const panelHeight=controlHeight*(narrow?2:1)+jogHeight+(narrow?12:6)+18;
  const viewsHeight=Math.max(0,viewSize.height-panelHeight-8);
  const mainWidth=narrow?viewSize.width:availableWidth*share;
  const sideLeft=narrow?0:mainWidth+30;
  const sideWidth=narrow?viewSize.width:availableWidth-mainWidth;
  const mainRect={left:0,top:0,width:mainWidth,height:narrow?Math.max(0,(viewsHeight-8)*.46):viewsHeight};
  const insetRect=narrow
    ?{left:0,top:mainRect.height+8,width:sideWidth,height:Math.max(0,viewsHeight-mainRect.height-8)}
    :{left:sideLeft,top:0,width:sideWidth,height:viewsHeight};
  const panelRect={left:0,top:viewsHeight+8,width:viewSize.width,height:panelHeight};
  const connected=useRobot(s=>s.conn==='connected');
  const hasArm=useArmAvailable();
  const available=useRef(hasArm);available.current=hasArm;
  const stat=useTelemetry(s=>s.robot?.armStat);
  const hasPose=useTelemetry(s=>(s.robot?.joints.length??0)>=18);
  const hasFc=useTelemetry(s=>Number.isFinite(s.robot?.fcYaw));
  const [focused,setFocused]=useState(false),[entered,setEntered]=useState(false);
  const [busy,setBusy]=useState(false),[error,setError]=useState('');
  const [held,setHeld]=useState<string|null>(null);
  const [dragging,setDragging]=useState(false),[inputMode,setInputMode]=useState<'drag'|'buttons'>('drag');
  const dragReady=useRef(false),gestureCancel=useRef<(()=>void)|null>(null);
  const [moveFrame,setMoveFrame]=useState<EeMoveFrame>('robot');
  const [eeMode,setEeMode]=useState<EeMode>(1),[eeAxis,setEeAxis]=useState<EeAxis>(3);
  const alive=useRef(true),generation=useRef(0);
  const report=useCallback((e:unknown)=>{if(alive.current)setError(e instanceof Error?e.message:String(e));},[]);
  const hold=useMemo(()=>createArmHold(sendArmManual,report),[report]);
  const drag=useMemo(()=>createArmDrag(hold,()=>{
    const state=useTelemetry.getState();
    return alive.current&&dragReady.current&&state.robot?.armStat.eeSpeedControl===true&&useRobot.getState().conn==='connected'
      &&armAvailable(state.robot,state.pdu,simulatedRobot())&&armManualReady(state.robot?.armStat);
  },setHeld,report),[hold,report]);
  const stop=useCallback(()=>{gestureCancel.current?.();drag.cancel();hold.stop();setHeld(null);setDragging(false);},[drag,hold]);
  useEffect(()=>{stop();},[moveSpeed,rotateSpeed,hasFc,stop]);
  const resizeViews=useCallback((next:number)=>setMainShare(Math.max(split.current.min,Math.min(split.current.max,next))),[]);
  const selectMainView=(next:'camera'|'model')=>{stop();setMainView(next);};
  useEffect(()=>{stop();setSpeedSettings(false);split.current.active=false;},[width,height,stop]);
  const divider=useMemo(()=>PanResponder.create({
    onStartShouldSetPanResponder:()=>true,
    onPanResponderGrant:e=>{if(Platform.OS==='web')e.preventDefault();stop();split.current.start=split.current.share;split.current.active=true;},
    onPanResponderStart:(_,g)=>{if(g.numberActiveTouches>1)split.current.active=false;},
    onPanResponderMove:(_,g)=>{
      const frame=split.current;
      if(frame.active&&frame.width>0&&g.numberActiveTouches===1)resizeViews(frame.start+g.dx/frame.width);
    },
    onPanResponderRelease:()=>{split.current.active=false;},
    onPanResponderTerminate:()=>{split.current.active=false;},
    onPanResponderTerminationRequest:()=>false,
  }),[resizeViews,stop]);
  useEffect(()=>{
    const check=()=>{
      if(useRobot.getState().conn==='connected')return;
      generation.current++;
      dragReady.current=false;gestureCancel.current?.();drag.cancel();hold.cancel();setDragging(false);
      setHeld(null);setEntered(false);
    };
    const unsubscribe=useRobot.subscribe(check);
    check();
    return unsubscribe;
  },[drag,hold]);
  useEffect(()=>{alive.current=true;return()=>{alive.current=false;drag.cancel();hold.stop();};},[drag,hold]);
  useFocusEffect(useCallback(()=>{setFocused(true);return()=>{generation.current++;setFocused(false);stop();};},[stop]));
  useEffect(()=>{
    const sub=AppState.addEventListener('change',s=>{if(s!=='active')stop();});
    const blur=()=>stop();const hidden=()=>{if(document.hidden)stop();};
    if(Platform.OS==='web'){window.addEventListener('blur',blur);document.addEventListener('visibilitychange',hidden);}
    return()=>{sub.remove();if(Platform.OS==='web'){window.removeEventListener('blur',blur);document.removeEventListener('visibilitychange',hidden);}};
  },[stop]);
  const run=async(command:ArmManualCommand)=>{
    const attempt=generation.current;
    stop();setBusy(true);setError('');
    try {
      await hold.settled();
      const commands=command===cmd.home?[cmd.jog(1,1,0),cmd.jog(2,1,0),cmd.gripStop,command]:[command];
      for(const next of commands){
        if(!alive.current || attempt!==generation.current || !available.current || useRobot.getState().conn!=='connected')return false;
        await sendArmManual(next);
      }
      return true;
    }
    catch(e){report(e);return false;}finally{if(alive.current)setBusy(false);}
  };
  useImperativeHandle(ref,()=>({prepareExit:async()=>{
    stop();
    if(!connected || !hasArm){
      if(!connected)hold.cancel();
      else try{await hold.settled();}catch(e){report(e);return false;}
      setEntered(false);return true;
    }
    if(busy)return false;
    return run(cmd.home);
  }}));
  const enter=async()=>{
    stop();const attempt=++generation.current;
    setBusy(true);setError('');
    try {
      if(!alive.current || generation.current!==attempt || !available.current)return;
      await sendArmManual(cmd.gripStop);
      await sendArmManual(cmd.enter);
      if(alive.current && generation.current===attempt)setEntered(true);
    }catch(e){report(e);}finally{if(alive.current)setBusy(false);}
  };
  useEffect(()=>{if(!hasArm){generation.current++;drag.cancel();setDragging(false);if(connected)hold.stop();else hold.cancel();setHeld(null);setEntered(false);}},[hasArm,connected,hold,drag]);
  const canMove=focused&&connected&&hasArm&&entered&&armManualReady(stat)&&!busy;
  const canJog=canMove&&stat?.eeSpeedControl===true&&!speedSettings&&!axisSettings;
  const speed=eeMode===1?moveSpeed:rotateSpeed;
  dragReady.current=canJog;
  useEffect(()=>{if(!canMove)stop();},[canMove,stop]);
  useEffect(()=>{if(!canJog)stop();},[canJog,stop]);
  const disabled=!connected||!hasArm||busy;
  const selectAxis=useCallback((axis:EeAxis)=>{stop();setEeAxis(axis);},[stop]);
  const selectMode=(mode:EeMode)=>{stop();setEeMode(mode);};
  const dragInput=useCallback((mode:EeMode,axis:EeAxis,sign:-1|0|1)=>{
    if(!dragReady.current){stop();return;}
    setDragging(true);void drag.move(mode,axis,sign,moveFrame,mode===1?moveSpeed:rotateSpeed);
  },[drag,stop,moveFrame,moveSpeed,rotateSpeed]);
  const button=(label:string,action:()=>void,off=disabled,selected=false,minHeight?:number)=><Tappable disabled={off} onPress={action}
    accessibilityLabel={t(label)} style={[styles.button,{flex:label==='EE 조작 정지'?1:undefined,minHeight:label==='EE 조작 정지'?(narrow?controlHeight:jogHeight):minHeight,borderRadius:radius.md,borderColor:label==='EE 조작 정지'?c.dangerLine:selected?c.accent2:c.line,backgroundColor:label==='EE 조작 정지'?c.panel:selected?'rgba(77,156,245,0.12)':c.elev,opacity:off?.4:1}]}>
      <Text numberOfLines={1} style={{color:label==='EE 조작 정지'?c.redTx:selected?c.accent2:c.text,fontSize:11,fontWeight:'600'}}>{t(label)}</Text>
    </Tappable>;
  const jog=(id:string,label:string,start:ArmManualCommand,end:ArmManualCommand)=><ArmHoldButton key={id} label={t(label)} disabled={id.startsWith('ee')?!canJog:!canMove||speedSettings||axisSettings} held={held===id}
    onStart={()=>{drag.cancel();setDragging(false);if(hold.start(id,start,end))setHeld(id);}}
    onEnd={()=>{hold.stop(id);setHeld(v=>v===id?null:v);}}
    style={[styles.jog,{height:jogHeight,borderRadius:radius.md,borderColor:held===id?c.accent2:c.line,backgroundColor:held===id?c.accent2:c.elev,opacity:(id.startsWith('ee')?canJog:canMove)&&!speedSettings&&!axisSettings?1:.4}]}>
      <Text style={{color:held===id?c.onAccent:c.text,fontSize:label==='+'||label==='−'?24:13,fontWeight:'600'}}>{t(label)}</Text>
    </ArmHoldButton>;
  const speedButton=<View ref={speedAnchor} collapsable={false}>
    <Tappable accessibilityLabel={t('EE 속도')} onPress={()=>{
      stop();setAxisSettings(false);
      speedAnchor.current?.measureInWindow((x,y,w,h)=>rootRef.current?.measureInWindow((rx,ry,rw,rh)=>{
        const top=y+h-ry+6;
        setSpeedPosition({left:Math.max(12,Math.min(x-rx,rw-Math.min(300,width-24)-12)),top,maxHeight:Math.max(0,rh-top-12)});
        setSpeedSettings(true);
      }));
    }} style={[styles.button,{minHeight:28,minWidth:68,borderRadius:radius.md,borderColor:c.line,backgroundColor:c.elev}]}>
      <Text style={{color:c.text,fontSize:11,fontWeight:'700'}}>{t('속도')} {speed}%</Text>
    </Tappable>
  </View>;
  const status=!connected?'연결 확인 필요':!hasArm?'팔 전원 및 통신 확인 필요':busy?'수동제어 진입 중':!entered?'수동제어 대기':!stat?.conStart?'팔 제어 시작을 확인하세요.':stat.lockPosition?'팔 위치 고정':!armManualReady(stat)?'수동제어 응답 대기':'Control Mode';
  return <View ref={rootRef} testID="arm-manual-workspace" style={[styles.root,{gap:compact?6:10,padding:compact?8:12}]}>
    <ScrollView horizontal showsHorizontalScrollIndicator={false} style={{flexGrow:0,flexShrink:0}} contentContainerStyle={[styles.toolbar,{borderColor:c.line,flexWrap:'nowrap'}]}>
      <View style={{flex:1,minWidth:110}}><Text style={{color:c.text,fontSize:16,fontWeight:'700'}}>{t('수동제어')}</Text>
        <Text style={{color:canMove?c.greenTx:c.muted,fontSize:11,marginTop:3}}>{t(status)}</Text></View>
      {button(entered?'초기 위치 (Reset)':'수동제어 시작',()=>void enter())}
      {button('Fix',()=>void run(cmd.fix),disabled||!entered||!armManualReady(stat),!!stat?.lockPosition)}
      {button('Control',()=>void run(cmd.control),disabled||!entered||!stat?.lockPosition,canMove)}
      {button('Robot Control Mode',()=>void run({command:MC.DEMO_ROBOT_CONTROL_MODE}),disabled||!entered||!stat?.conStart||!!stat.lockPosition)}
      {button('Chicken Head Mode',()=>void run({command:MC.DEMO_CHICKEN_HEAD_MODE}),disabled||!entered||!stat?.conStart||!!stat.lockPosition)}
      {button(hasArm?'팔 접고 나가기':'나가기',onExit,hasArm&&busy)}
    </ScrollView>
    {connected&&hasArm&&stat?.eeSpeedControl!==true&&<Text testID="arm-ee-speed-unsupported" style={{color:c.amberTx,fontSize:11}}>{t('EE 속도 제어를 위해 로봇 소프트웨어를 업데이트하세요.')}</Text>}
    {!!error&&<Text accessibilityRole="alert" style={{color:c.redTx,fontSize:11}}>{error}</Text>}
    <View testID="arm-manual-views" onLayout={e=>{const {width:w,height:h}=e.nativeEvent.layout;setViewSize(old=>old.width===w&&old.height===h?old:{width:w,height:h});}}
      style={{flex:1,minHeight:0}}>
      <View testID="arm-manual-camera" style={[styles.pane,mainView==='camera'?mainRect:insetRect,{borderColor:c.line,borderRadius:radius.lg,backgroundColor:c.bg}]}>
        <View style={[styles.paneHead,{backgroundColor:c.panel}]}>
          <Text style={{color:c.text,fontWeight:'700',fontSize:12}}>{t('몸통 전방')}</Text>
          {mainView==='camera'&&button('3D 크게 보기',()=>selectMainView('model'),false)}
        </View>
        <View style={{flex:1,overflow:'hidden'}}><CameraView streamId={ARM_MANUAL_STREAM_ID} active={focused} fit="contain"/></View>
      </View>
      {!narrow&&<View testID="arm-view-divider" {...divider.panHandlers} accessible accessibilityRole="adjustable"
        accessibilityLabel={t('주 화면과 3D 패널 너비 조절')} accessibilityValue={{min:Math.round(minShare*100),max:Math.round(maxShare*100),now:Math.round(share*100)}}
        accessibilityActions={[{name:'increment'},{name:'decrement'}]}
        onAccessibilityAction={e=>{stop();resizeViews(share+(e.nativeEvent.actionName==='increment'?.05:-.05));}}
        {...(Platform.OS==='web'?{tabIndex:0,onKeyDown:(e:{key:string;preventDefault:()=>void})=>{
          if(e.key==='ArrowLeft'||e.key==='ArrowRight'){e.preventDefault();stop();resizeViews(share+(e.key==='ArrowRight'?.05:-.05));}
        }}:{})}
        style={[styles.divider,{position:'absolute',left:mainWidth+4,top:0,height:viewsHeight,backgroundColor:c.panel,borderRadius:radius.sm},web({cursor:'col-resize',touchAction:'none',userSelect:'none'})]}>
        <View style={{height:36,width:4,borderRadius:radius.sm,backgroundColor:c.muted}}/>
      </View>}
      <View testID="arm-manual-model" style={[styles.pane,mainView==='model'?mainRect:insetRect,{borderColor:c.line,borderRadius:radius.lg,backgroundColor:c.panel2}]}>
        <View style={[styles.paneHead,{backgroundColor:c.panel}]}>
          <Text numberOfLines={1} style={{color:c.text,fontSize:12,fontWeight:'700',flexShrink:1}}>{t('로봇 · 팔 3D')}</Text>
          <View style={{flexDirection:'row',gap:4,alignItems:'center'}}>
            {speedButton}
              {button('조작 표시',()=>{stop();setSpeedSettings(false);setAxisSettings(true);},false)}
            {button(eeView==='close'?'전체 보기':'EE 중심',()=>{stop();setEeView(eeView==='close'?'full':'close');},false)}
            {mainView==='model'&&button('카메라 크게 보기',()=>selectMainView('camera'),false)}
          </View>
        </View>
        <View style={{flex:1}}><RobotModel3D eeGizmo={{mode:eeMode,axis:eeAxis,moving:!!held,frame:moveFrame,scale:eeScale/100,onSelect:selectAxis}}
          followFc eeView={eeView} eeFeedbackStyle={eeFeedbackStyle} onEeViewChange={view=>{stop();setEeView(view);}}
          eeInteraction={{enabled:canJog&&hasPose&&(eeMode===2||moveFrame==='ee'||hasFc)&&inputMode==='drag',cancelRef:gestureCancel,onDrag:dragInput,onDragEnd:stop}}
          active={focused} controls listenPresets={false} showPresetRow={false} initialDist={.65} gridSize={8} gridDivisions={16} backgroundColor={c.panel2}/>
          {(!hasPose||(eeMode===1&&moveFrame==='robot'&&!hasFc))&&<View pointerEvents="none" style={{position:'absolute',left:8,bottom:8}}>
            <Text style={{color:c.muted,fontSize:11}}>{t(!hasPose?'팔 자세 수신 대기':'FC 좌표계 수신 대기 · 로봇 소프트웨어 확인')}</Text>
          </View>}
        </View>
      </View>
      <View testID="arm-manual-controls" style={[panelRect,{position:'absolute',padding:8,gap:6,borderWidth:1,borderRadius:radius.lg,borderColor:c.line,backgroundColor:c.panel}]}>
        <View style={{height:controlHeight,flexDirection:'row',gap:8}}>
          <View style={{flex:1,justifyContent:'center'}}><Segmented options={[{key:'move',label:t('이동')},{key:'rotate',label:t('회전')}]} value={eeMode===1?'move':'rotate'}
            onChange={mode=>selectMode(mode==='move'?1:2)}/></View>
          <View style={{flex:1,justifyContent:'center'}}>
            {eeMode===1?<Segmented options={[{key:'robot',label:t('FC 기준')},{key:'ee',label:t('EE 기준')}]} value={moveFrame}
              onChange={frame=>{stop();setMoveFrame(frame);}}/>:button('EE 기준',()=>{},true,true)}
          </View>
          {!narrow&&<View style={{flex:1,justifyContent:'center'}}><Segmented options={[{key:'drag',label:t('축 드래그')},{key:'buttons',label:t('버튼 조작')}]} value={inputMode}
            onChange={mode=>{stop();setInputMode(mode);}}/></View>}
        </View>
        {narrow&&<View style={{height:controlHeight,flexDirection:'row',gap:8}}>
          <Tappable accessibilityLabel={t(inputMode==='drag'?'버튼 조작':'축 드래그')} onPress={()=>{stop();setInputMode(inputMode==='drag'?'buttons':'drag');}}
            style={[styles.button,{flex:1,borderRadius:radius.md,borderColor:c.line,backgroundColor:c.elev}]}>
            <Text style={{fontSize:11,fontWeight:'600',color:c.text}}>{t(inputMode==='drag'?'버튼 조작':'축 드래그')}</Text>
          </Tappable>
          <View style={{flex:1}}>{button('EE 조작 정지',stop,!held&&!dragging)}</View>
        </View>}
        <View style={{height:jogHeight,flexDirection:'row',gap:8}}>
          {inputMode==='buttons'&&<View testID="arm-ee-buttons" style={{flex:narrow?2.7:2,flexDirection:'row',gap:4}}>
            {EE_AXES.map(a=><Tappable key={a.axis} accessibilityLabel={eeMode===1?a.label:a.rotation} onPress={()=>selectAxis(a.axis)}
              style={[styles.button,{flex:1,paddingHorizontal:0,minWidth:36,borderRadius:radius.sm,borderColor:eeAxis===a.axis?a.color:c.line,backgroundColor:eeAxis===a.axis?c.panel2:c.elev}]}>
              <Text style={{color:a.color,fontSize:12,fontWeight:'700'}}>{a.label}</Text>
            </Tappable>)}
            {jog(`ee${eeMode}${eeAxis}-`,'−',cmd.jog(eeMode,eeAxis,-1,moveFrame,speed),cmd.jog(eeMode,eeAxis,0,moveFrame))}
            {jog(`ee${eeMode}${eeAxis}+`,'+',cmd.jog(eeMode,eeAxis,1,moveFrame,speed),cmd.jog(eeMode,eeAxis,0,moveFrame))}
          </View>}
          <View style={{flex:1,flexDirection:'row',gap:6}}>
            {jog('open','열기',cmd.grip(true),cmd.gripStop)}{jog('close','닫기',cmd.grip(false),cmd.gripStop)}
          </View>
          {!narrow&&<View style={{flex:1}}>{button('EE 조작 정지',stop,!held&&!dragging)}</View>}
        </View>
      </View>
    </View>
    {speedSettings&&<Popover onClose={()=>setSpeedSettings(false)} portal={false} scroll={false}
      style={{...speedPosition,width:Math.min(300,width-24),borderRadius:radius.lg}}>
      <View testID="arm-ee-speed-settings" style={{gap:compact?6:12}}>
        <View style={{flexDirection:'row',alignItems:'center',justifyContent:'space-between'}}>
          <Text style={{color:c.text,fontSize:13,fontWeight:'700'}}>{t('EE 속도')}</Text>
          {button('닫기',()=>setSpeedSettings(false),false,false,44)}
        </View>
        {(['move','rotate'] as const).map(kind=>{
          const value=kind==='move'?moveSpeed:rotateSpeed;
          return <View key={kind} style={{gap:compact?4:8}}>
            <View style={{flexDirection:'row',alignItems:'center',justifyContent:'space-between'}}>
              <Text style={{color:c.text,fontSize:12,fontWeight:'600'}}>{t(kind==='move'?'이동':'회전')}</Text>
              <Text testID={`arm-ee-speed-${kind}-value`} style={{color:c.text,fontSize:13,fontWeight:'700'}}>{value}%</Text>
            </View>
            <View testID={`arm-ee-speed-${kind}-slider`}><Slider value={(value-10)/.9} width="100%" onChange={v=>{stop();setEeSpeed(kind,10+v*.9);}}/></View>
          </View>;
        })}
        <View style={{flexDirection:'row',gap:8,minHeight:44}}>
          {([[15,10,'정밀'],[30,20,'보통'],[60,40,'빠르게']] as const).map(([move,rotate,label])=><View key={label} style={{flex:1}}>
            {button(label,()=>{stop();setEeSpeed('move',move);setEeSpeed('rotate',rotate);},false,moveSpeed===move&&rotateSpeed===rotate,44)}
          </View>)}
        </View>
      </View>
    </Popover>}
    {axisSettings&&<Popover onClose={()=>setAxisSettings(false)} portal={false} scroll={false}
      style={{right:12,top:compact?54:64,width:Math.min(300,width-48),borderRadius:radius.lg}}>
      <View testID="arm-axis-size-settings" style={{gap:12}}>
        <Text style={{color:c.text,fontSize:13,fontWeight:'700'}}>{t('조작 표시')}</Text>
        <View testID="arm-feedback-options"><Segmented value={eeFeedbackStyle} options={[
          {key:'glow',label:'A'},{key:'touch',label:'B'},{key:'ribbon',label:'C'},{key:'both',label:'B+C'},
        ]} onChange={value=>{stop();setEeFeedbackStyle(value);}}/></View>
        <Text style={{color:c.muted,fontSize:11}}>{t('A 발광 · B 연결선 · C 리본')}</Text>
        <View style={{flexDirection:'row',alignItems:'center',justifyContent:'space-between'}}>
          <Text style={{color:c.text,fontSize:13,fontWeight:'700'}}>{t('축 크기')}</Text>
          <Text testID="arm-axis-size-value" style={{color:c.text,fontSize:13,fontWeight:'700'}}>{eeScale}%</Text>
        </View>
        <View testID="arm-axis-size-slider"><Slider value={eeScale-100} onChange={v=>setEeScale(100+v)} width="100%"/></View>
        <View style={{flexDirection:'row',gap:8,minHeight:44}}>
          {button('축 작게',()=>setEeScale(eeScale-10),eeScale<=100)}
          {button('축 크게',()=>setEeScale(eeScale+10),eeScale>=200)}
        </View>
        <View style={{flexDirection:'row',gap:8,minHeight:44}}>
          {button('기본값',()=>setEeScale(140),false)}
          {button('닫기',()=>setAxisSettings(false),false)}
        </View>
      </View>
    </Popover>}
  </View>;
}
const styles=StyleSheet.create({
 root:{flex:1,minHeight:0},toolbar:{flexDirection:'row',alignItems:'center',gap:6,flexWrap:'wrap',paddingBottom:8,borderBottomWidth:1},
 button:{paddingHorizontal:10,paddingVertical:6,borderWidth:1,alignItems:'center',justifyContent:'center'},
 pane:{position:'absolute',overflow:'hidden',borderWidth:1},paneHead:{height:38,padding:5,flexDirection:'row',alignItems:'center',justifyContent:'space-between',gap:8},
 divider:{width:22,alignItems:'center',justifyContent:'center'},
 jog:{flex:1,alignItems:'center',justifyContent:'center',borderWidth:1,minWidth:40,...(Platform.OS==='web'?{touchAction:'none' as const}:{})},
});

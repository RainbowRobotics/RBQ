import { useEffect } from 'react';
import { StyleSheet, Text, View } from 'react-native';
import Animated, { Easing, cancelAnimation, useAnimatedProps, useReducedMotion, useSharedValue, withRepeat, withTiming } from 'react-native-reanimated';
import Svg, { Circle, Line, Path, Polygon } from 'react-native-svg';
import { EE_AXES } from '@/lib/armEeGizmo';
import { eeFeedbackPath, type EeFeedbackState, type EeFeedbackStyle } from '@/lib/armEeFeedback';
import { useTheme } from '@/theme';
import { t } from '@/lib/i18n';

const AnimatedPath=Animated.createAnimatedComponent(Path);
export function ArmEeFeedback({state,variant,width,height}:{state:EeFeedbackState;variant:EeFeedbackStyle;width:number;height:number}) {
  const {c,radius}=useTheme(),{grab,point,sign}=state;
  const axis=EE_AXES[grab.axis-1],color=axis.color;
  const dash=useSharedValue(0),reduceMotion=useReducedMotion();
  const moving=sign!==0&&variant!=='touch';
  const flow=useAnimatedProps(()=>({strokeDashoffset:dash.value}));
  useEffect(()=>{
    dash.value=0;
    if(!moving||reduceMotion)return;
    dash.value=withRepeat(withTiming(-40,{duration:700,easing:Easing.linear}),-1,false);
    return()=>cancelAnimation(dash);
  },[moving,reduceMotion,dash]);
  const ribbon=sign?eeFeedbackPath(grab,sign,width,height):null;
  const touch=variant==='touch'||variant==='both';
  const title=grab.mode===1?axis.label:axis.rotation;
  return <View testID="arm-ee-feedback" pointerEvents="none" style={StyleSheet.absoluteFill}>
    <Svg testID="arm-ee-feedback-svg" width={width} height={height} viewBox={`0 0 ${width} ${height}`}>
      {ribbon&&variant!=='touch'&&<>
        <Path testID="arm-ee-feedback-direction" d={ribbon.path} fill="none" stroke={color} strokeWidth={variant==='glow'?18:24} strokeOpacity={variant==='glow'?.25:.65} strokeLinecap="round"/>
        <AnimatedPath d={ribbon.path} fill="none" stroke={variant==='glow'?color:c.panel} strokeWidth={variant==='glow'?5:4} strokeDasharray="8 12" animatedProps={flow} strokeLinecap="round"/>
        <Polygon points={ribbon.head} fill={color} stroke={c.panel} strokeWidth={2}/>
      </>}
      {(touch||variant==='glow')&&<>
        {touch&&<>
          <Line testID="arm-ee-feedback-tether" x1={grab.origin.x} y1={grab.origin.y} x2={point.x} y2={point.y} stroke={c.panel} strokeWidth={8}/>
          <Line x1={grab.origin.x} y1={grab.origin.y} x2={point.x} y2={point.y} stroke={color} strokeWidth={3} strokeDasharray="5 7"/>
        </>}
        <Circle cx={grab.origin.x} cy={grab.origin.y} r={16} fill={c.panel} fillOpacity={.65} stroke={color} strokeWidth={3}/>
        {touch&&<><Circle cx={point.x} cy={point.y} r={24} fill={color} fillOpacity={.12} stroke={color} strokeWidth={3}/><Circle cx={point.x} cy={point.y} r={4} fill={color}/></>}
      </>}
    </Svg>
    <View testID="arm-ee-feedback-status" accessibilityLiveRegion="polite" style={{position:'absolute',top:8,left:8,paddingHorizontal:10,paddingVertical:6,borderRadius:radius.md,borderWidth:1,borderColor:color,backgroundColor:c.panel}}>
      <Text style={{color:c.text,fontSize:12,fontWeight:'700'}}>{title}{sign>0?' +':sign<0?' −':''} · {t(sign?'입력 중':'축 잡는 중')}</Text>
    </View>
  </View>;
}

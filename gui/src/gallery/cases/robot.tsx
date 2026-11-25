import { useState } from 'react';
import { View, Text } from 'react-native';
import { useTheme } from '@/theme';
import { Joystick } from '@/components/Joystick';
import { MotionGridModal } from '@/components/control/overlays';
import { Tappable } from '@/components/anim';
import { demoBtn } from '../Frame';
import { GamepadDiagram } from '@/components/GamepadDiagram';
import type { WidgetCase } from '../types';

function JoystickDemo() {
  const { c, fonts } = useTheme();
  const [xy, setXy] = useState({ x: 0, y: 0 });
  return (
    <View style={{ alignItems: 'center', gap: 8 }}>
      <Joystick size={140} onMove={(x, y) => setXy({ x, y })} />
      <Text style={{ color: c.muted, fontSize: 11, fontFamily: fonts.mono }}>
        x {xy.x.toFixed(2)}  y {xy.y.toFixed(2)}
      </Text>
    </View>
  );
}

function MotionGridDemo() {
  const { c } = useTheme();
  const [open, setOpen] = useState(false);
  return (
    <View>
      <Tappable onPress={() => setOpen(true)} style={[demoBtn, { backgroundColor: c.elev, borderWidth: 1, borderColor: c.line }]}>
        <Text style={{ color: c.text, fontSize: 12, fontWeight: '600' }}>모션 그리드 열기</Text>
      </Tappable>
      {open && <MotionGridModal onClose={() => setOpen(false)} onPick={() => setOpen(false)} />}
    </View>
  );
}

function GamepadDemo() {
  return (
    <View style={{ alignItems: 'center' }}>
      <GamepadDiagram deviceName="데모 패드" pressed={[96, 19]} bound={[96, 97, 102]} />
    </View>
  );
}

export const robot: WidgetCase[] = [
  { name: 'Joystick',
    code: `import { Joystick } from '@/components/Joystick';

<Joystick size={158} onMove={(x, y) => sendJoy(x, y)} />
// onMove 는 -1..1 정규화 좌표. 하드웨어 패드가 붙으면 호출부가 이걸 숨긴다.`, from: '@/components/Joystick', Demo: JoystickDemo,
    when: '가상 조이스틱. onMove 는 -1..1 정규화 좌표 — 하드웨어 패드가 붙으면 호출부가 이걸 숨긴다.' },
  { name: 'MotionGridModal',
    code: `import { MotionGridModal } from '@/components/control/overlays';

{open && <MotionGridModal onClose={close} onPick={(m) => runMotion(m)} />}
// 어떤 타일이 눌리는지는 현재 gait 가 정한다.`, from: '@/components/control/overlays', Demo: MotionGridDemo,
    when: '모션 선택 그리드(앉기·서기·보행…). 어떤 타일이 눌리는지는 현재 gait 가 정하므로, 로봇이 없으면 대부분 비활성인 게 정상.' },
  { name: 'GamepadDiagram',
    code: `import { GamepadDiagram } from '@/components/GamepadDiagram';

<GamepadDiagram deviceName={pad.name} pressed={pressedKeys} bound={boundKeys}
  onPressKey={(code, label) => bind(code)} />
// compact 는 0.8배 축소. 리맵 마법사와 입력 확인 화면이 이걸 공유한다.`, from: '@/components/GamepadDiagram', Demo: GamepadDemo,
    when: '패드 배치도. 리맵 마법사와 입력 확인 화면이 공유한다 — pressed 로 눌린 키가 하이라이트된다.' },
];

import { useState } from 'react';
import { View, Text } from 'react-native';
import { useTheme } from '@/theme';
import { Tappable } from '@/components/anim';
import { RobotModel3D } from '@/components/RobotModel3D';
import { PayloadPreview3D } from '@/components/PayloadPreview3D';
import type { PayloadRow } from '@/lib/payload';
import { demoBtn } from '../Frame';
import type { WidgetCase } from '../types';

function OnDemand({ children }: { children: React.ReactNode }) {
  const { c } = useTheme();
  const [on, setOn] = useState(false);
  return (
    <View style={{ gap: 10 }}>
      <Tappable onPress={() => setOn((v) => !v)} style={[demoBtn, { backgroundColor: c.elev, borderWidth: 1, borderColor: c.line }]}>
        <Text style={{ color: c.text, fontSize: 12, fontWeight: '600' }}>{on ? '3D 끄기' : '3D 켜기'}</Text>
      </Tappable>
      {on && children}
    </View>
  );
}

function RobotDemo() {
  return (
    <OnDemand>
      <View style={{ height: 240 }}>
        <RobotModel3D listenPresets={false} showPresetRow={false} />
      </View>
    </OnDemand>
  );
}

const row = (id: number, label: string, mass: number, x: number, y: number, z: number): PayloadRow => ({
  id, name: `slot${id}`, label, isCustom: false, enabled: true, mass, x, y, z,
  defaultMass: mass, defaultX: x, defaultY: y, defaultZ: z,
});

const DEMO_ROWS = [row(0, '배터리', 4.2, 0.05, 0, 0.12), row(1, '센서', 1.1, -0.18, 0.06, 0.2)];

function PayloadDemo() {
  const { c } = useTheme();
  const [sel, setSel] = useState(-1);
  return (
    <OnDemand>
      <View style={{ gap: 6 }}>
        <PayloadPreview3D rows={DEMO_ROWS} total={{ mass: 5.3, x: 0.0, y: 0.01, z: 0.14 }}
          selectedId={sel} onSelect={setSel} height={220} />
        <Text style={{ color: c.muted, fontSize: 11 }}>선택된 슬롯: {sel < 0 ? '없음' : DEMO_ROWS[sel]?.label}</Text>
      </View>
    </OnDemand>
  );
}

export const three: WidgetCase[] = [
  { name: 'RobotModel3D',
    code: `import { RobotModel3D } from '@/components/RobotModel3D';

<View style={{ height: 240 }}>
  <RobotModel3D active={visible} listenPresets={false} showPresetRow={false} />
</View>
// ⚠ active={false} 로 렌더 루프를 멈춘다(마운트는 유지) — 숨겨진 3D 가 GPU 를 먹는 걸 막는다.
//   pose 를 주면 실시간 텔레메트리 대신 그 값을 그린다(블랙박스 재생).`, from: '@/components/RobotModel3D', Demo: RobotDemo,
    when: '대시보드의 3D 로봇 뷰. 로봇이 없으면 기본 포즈로 선다 — 끌어서 궤도 회전, 휠로 줌.' },
  { name: 'PayloadPreview3D',
    code: `import { PayloadPreview3D } from '@/components/PayloadPreview3D';

<PayloadPreview3D rows={rows} total={com} selectedId={sel}
  onSelect={setSel} onMove={(id, axis, v) => setCoord(id, axis, v)} height={260} />
// onMove 는 클램프까지 끝난 값이 온다 — 호출부가 다시 범위 검사할 필요 없다.`, from: '@/components/PayloadPreview3D', Demo: PayloadDemo,
    when: '적재물 배치·무게중심 미리보기. 마커를 눌러 슬롯을 고르고 기즈모 축을 끌어 좌표를 바꾼다.' },
];

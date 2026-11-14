import { useState } from 'react';
import { View, Text } from 'react-native';
import { useTheme } from '@/theme';
import { Battery } from '@/components/Battery';
import { AttitudeDial } from '@/components/AttitudeDial';
import { SpeedHud } from '@/components/SpeedHud';
import { Slider } from '@/components/ui/controls';
import type { WidgetCase } from '../types';

function BatteryDemo() {
  return (
    <View style={{ flexDirection: 'row', gap: 16, flexWrap: 'wrap', alignItems: 'center' }}>
      <Battery pct={88} label="로봇" />
      <Battery pct={31} label="로봇" />
      <Battery pct={8} label="로봇" />
      <Battery pct={64} label="패드" charging />
      <Battery pct={null} label="로봇" />
    </View>
  );
}

function AttitudeDemo() {
  const { c } = useTheme();
  const [roll, setRoll] = useState(50);
  const [pitch, setPitch] = useState(50);
  const r = (roll - 50) * 1.2;
  const p = (pitch - 50) * 0.8;
  return (
    <View style={{ alignItems: 'center', gap: 10, width: '100%' }}>
      <AttitudeDial size={110} roll={r} pitch={p}
        colors={{ sky: c.accent2, ground: c.elev, line: c.line, cross: c.text, border: c.line }} />
      <Text style={{ color: c.muted, fontSize: 11 }}>roll {r.toFixed(0)}° · pitch {p.toFixed(0)}°</Text>
      <Slider value={roll} onChange={setRoll} width="100%" />
      <Slider value={pitch} onChange={setPitch} width="100%" />
    </View>
  );
}

function SpeedDemo() {
  return (
    <View style={{ gap: 10, alignItems: 'flex-start' }}>
      <SpeedHud />
      <SpeedHud dense />
    </View>
  );
}

export const status: WidgetCase[] = [
  { name: 'Battery',
    code: `import { Battery } from '@/components/Battery';

<Battery pct={88} label="로봇" />
<Battery pct={64} label="패드" charging />
<Battery pct={null} label="로봇" />     // 값 없음 → '—'
// 색은 값에서 자동으로 갈린다(>40 초록 / ≥15 호박 / 그 아래 빨강). 직접 칠하지 않는다.`, from: '@/components/Battery', Demo: BatteryDemo,
    when: '잔량 배지. 색은 값에서 자동으로 갈린다(>40 초록 / ≥15 호박 / 그 아래 빨강). null 이면 "—".' },
  { name: 'AttitudeDial',
    code: `import { AttitudeDial } from '@/components/AttitudeDial';

<AttitudeDial size={110} roll={roll} pitch={pitch}
  colors={{ sky: c.accent2, ground: c.elev, line: c.line, cross: c.text, border: c.line }} />
// 색을 호출부가 넘기는 순수 표시 위젯 — 테마를 직접 읽지 않아 어디든 얹을 수 있다.`, from: '@/components/AttitudeDial', Demo: AttitudeDemo,
    when: '자세계(롤·피치). 색을 호출부가 넘기는 순수 표시 위젯이라 테마와 무관하게 어디든 얹을 수 있다.' },
  { name: 'SpeedHud',
    code: `import { SpeedHud } from '@/components/SpeedHud';

<SpeedHud />          // 알약 — 제 폭을 갖는다. 늘리지 말 것
<SpeedHud dense />    // 좁은 자리용
// 스토어를 직접 구독한다. 로봇이 없으면 '—' 가 정상.`, from: '@/components/SpeedHud', Demo: SpeedDemo,
    when: '주행 속도·요레이트 HUD. 스토어를 직접 구독하므로 로봇이 없으면 "—"로 뜨는 게 정상이다.' },
];

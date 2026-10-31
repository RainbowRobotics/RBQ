import { useState } from 'react';
import { View, Text } from 'react-native';
import { useTheme } from '@/theme';
import { Icon, ICONS, type IconName } from '@/components/Icon';
import { Tappable, Pulse } from '@/components/anim';
import { LongPressButton } from '@/components/ui/LongPressButton';
import { demoBtn } from '../Frame';
import type { WidgetCase } from '../types';

const ICON_NAMES = Object.keys(ICONS) as IconName[];

function IconDemo() {
  const { c } = useTheme();
  return (
    <View style={{ flexDirection: 'row', flexWrap: 'wrap', gap: 8 }}>
      {ICON_NAMES.map((n) => (
        <View key={n} style={{ width: 46, alignItems: 'center' }}>
          <Icon name={n} size={19} color={c.text} />
          <Text numberOfLines={1} style={{ color: c.dim, fontSize: 8, marginTop: 3 }}>{n}</Text>
        </View>
      ))}
    </View>
  );
}

function TappableDemo() {
  const { c } = useTheme();
  const [n, setN] = useState(0);
  return (
    <Tappable onPress={() => setN((v) => v + 1)} style={[demoBtn, { backgroundColor: c.accent }]}>
      <Text style={{ color: c.onAccent, fontWeight: '700', fontSize: 12 }}>눌러보기 ({n})</Text>
    </Tappable>
  );
}

function PulseDemo() {
  const { c } = useTheme();
  return (
    <View style={{ flexDirection: 'row', gap: 22, alignItems: 'center' }}>
      <Pulse color={c.green} />
      <Pulse color={c.amber} />
      <Pulse color={c.redbright} size={12} halo={24} />
    </View>
  );
}

function LongPressDemo() {
  const { c } = useTheme();
  const [n, setN] = useState(0);
  return (
    <LongPressButton onLongPress={() => setN((v) => v + 1)}
      style={[demoBtn, { backgroundColor: c.elev, borderWidth: 1, borderColor: c.line }]}>
      <Text style={{ color: c.text, fontSize: 12, fontWeight: '600' }}>0.5초 꾹 눌러 실행 ({n})</Text>
    </LongPressButton>
  );
}

export const primitives: WidgetCase[] = [
  { name: 'Icon',
    code: `import { Icon } from '@/components/Icon';

<Icon name="gauge" size={18} color={c.text} />
// name 은 ICONS 의 키. 새 아이콘은 Icon.tsx 의 ICONS 에만 추가한다.`, from: '@/components/Icon', Demo: IconDemo,
    when: `Lucide 패스를 SvgXml 로 렌더. ${ICON_NAMES.length}종 — 새 아이콘은 ICONS 에만 추가하면 전 화면에서 쓸 수 있다.` },
  { name: 'Tappable',
    code: `import { Tappable } from '@/components/anim';

<Tappable onPress={onSave} style={[s.btn, { backgroundColor: c.accent }]}>
  <Text style={{ color: c.onAccent, fontWeight: '700', fontSize: 12 }}>저장</Text>
</Tappable>
// 신UI 의 탭 요소는 전부 이것. 맨 Pressable 은 눌림 피드백이 없다.`, from: '@/components/anim', Demo: TappableDemo,
    when: '눌림 스케일이 붙은 Pressable. 신UI 의 탭 요소는 전부 이걸 쓴다 — 맨 Pressable 은 피드백이 없다.' },
  { name: 'Pulse',
    code: `import { Pulse } from '@/components/anim';

<Pulse color={c.green} />               // 살아있음
<Pulse color={c.redbright} size={12} halo={24} />`, from: '@/components/anim', Demo: PulseDemo,
    when: '살아있음(라이브 텔레메트리·녹화 중)을 알리는 맥동 점. 정적 상태에는 쓰지 않는다.' },
  { name: 'LongPressButton',
    code: `import { LongPressButton } from '@/components/ui/LongPressButton';

<LongPressButton onLongPress={fire} style={[s.btn, { borderColor: c.line }]}>
  <Text style={{ color: c.text, fontSize: 12, fontWeight: '600' }}>0.5초 홀드</Text>
</LongPressButton>
// onPress = 짧은 탭, onLongPress = 500ms 홀드(게이지가 찬다). 위험 동작 전용.`, from: '@/components/ui/LongPressButton', Demo: LongPressDemo,
    when: '오발 방지 홀드 버튼(500ms, Qt DelayButton 대응). 웹은 DOM 포인터·네이티브는 제스처로 구현이 갈린다 — 되돌릴 수 없는 위험 동작의 유일 안전장치라 전원·투입 같은 위험 동작 전용.' },
];

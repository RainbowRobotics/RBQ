import { useState } from 'react';
import { View, Text } from 'react-native';
import { useTheme } from '@/theme';
import { Toggle, Segmented, Slider, Select } from '@/components/ui/controls';
import { HTabs } from '@/components/ui/HTabs';
import { H2, Desc, Field, EditField, TRow, SBtn, Group, LegibleText } from '@/components/panels/settings/common';
import type { WidgetCase } from '../types';

function ToggleDemo() {
  const { c } = useTheme();
  const [on, setOn] = useState(true);
  return (
    <View style={{ flexDirection: 'row', gap: 18, alignItems: 'center' }}>
      <Toggle value={on} onChange={setOn} />
      <Toggle value={!on} onChange={(v) => setOn(!v)} />
      <Toggle value disabled />
      <Text style={{ color: c.muted, fontSize: 11 }}>{on ? 'ON' : 'OFF'} · disabled</Text>
    </View>
  );
}

function SegmentedDemo() {
  const [v, setV] = useState('walk');
  return (
    <Segmented options={[{ key: 'walk', label: '보행' }, { key: 'trot', label: '속보' }, { key: 'stair', label: '계단' }]}
      value={v} onChange={setV} />
  );
}

function SelectDemo() {
  const { c } = useTheme();
  const [v, setV] = useState('front');
  const opts = [{ key: 'front', label: '전방 카메라' }, { key: 'rear', label: '후방 카메라' }, { key: 'thermal', label: '열화상' }];
  return (
    <View style={{ gap: 10 }}>
      <Select options={opts} value={v} onChange={setV} label="영상 소스" />
      <View style={{ width: 92 }}><Select options={opts} value={v} onChange={setV} dense /></View>
      <Text style={{ color: c.muted, fontSize: 11 }}>선택: {v}</Text>
    </View>
  );
}

function SliderDemo() {
  const { c } = useTheme();
  const [v, setV] = useState(45);
  const [committed, setCommitted] = useState(45);
  return (
    <View style={{ gap: 6 }}>
      <Slider value={v} onChange={setV} onCommit={setCommitted} width="100%" />
      <Text style={{ color: c.muted, fontSize: 11 }}>드래그 중 {Math.round(v)} · 적용됨 {committed}</Text>
    </View>
  );
}

function FieldDemo() {
  const [ip, setIp] = useState('192.168.0.10');
  return (
    <View style={{ flexDirection: 'row', gap: 12 }}>
      <Field label="로봇 시리얼" value="RBQ1000000001" muted />
      <EditField label="로봇 IP" value={ip} onChangeText={setIp} />
    </View>
  );
}

function StructureDemo() {
  const [a, setA] = useState(true);
  const [b, setB] = useState(false);
  return (
    <View style={{ width: '100%' }}>
      <H2>보행 파라미터</H2>
      <Desc>로봇에 즉시 반영됩니다. 주행 중에도 바꿀 수 있습니다.</Desc>
      <TRow nm="장애물 회피" sub="전방 라이다 기준" right={<Toggle value={a} onChange={setA} />} />
      <TRow nm="자동 복구" sub="넘어짐 감지 시" right={<Toggle value={b} onChange={setB} />} />
    </View>
  );
}

function GroupDemo() {
  const [a, setA] = useState(true);
  return (
    <View style={{ width: '100%' }}>
      <LegibleText.Provider value>
        <H2>🎯 IMU 캘리브레이션</H2>
        <Desc>로봇을 평지에 정지 상태로 두고 실행합니다.</Desc>
        <Group>
          <TRow nm="가속도계 보정 (ACC)" sub="앉은 상태에서 실행 — 약 2초" right={<Toggle value={a} onChange={setA} />} />
          <TRow nm="자이로 바이어스 보정" sub="앉은 상태에서 실행 — 약 5초" right={<Toggle value={!a} onChange={(v) => setA(!v)} />} />
        </Group>
      </LegibleText.Provider>
    </View>
  );
}

function SBtnDemo() {
  return (
    <View style={{ flexDirection: 'row', gap: 10, flexWrap: 'wrap' }}>
      <SBtn kind="primary" icon="save" label="저장" onPress={() => {}} />
      <SBtn kind="ghost" icon="recover" label="되돌리기" onPress={() => {}} />
      <SBtn kind="danger" icon="trash" label="삭제" onPress={() => {}} />
      <SBtn kind="ghost" icon="save" label="비활성" disabled />
    </View>
  );
}

function HTabsDemo() {
  const [k, setK] = useState('a');
  return (
    <HTabs value={k} onChange={setK}
      items={[{ key: 'a', label: '개요', icon: 'gauge' }, { key: 'b', label: '연결', icon: 'wifi' },
        { key: 'c', label: '로그', icon: 'log' }]} />
  );
}

export const forms: WidgetCase[] = [
  { name: 'Toggle',
    code: `import { Toggle } from '@/components/ui/controls';

const [on, setOn] = useState(false);
<Toggle value={on} onChange={setOn} />
<Toggle value disabled />               // 조작 불가 상태`, from: '@/components/ui/controls', Demo: ToggleDemo,
    when: 'on/off 단일 스위치. 값이 즉시 적용되는 설정에만 — 확인이 필요한 동작은 버튼으로 간다.' },
  { name: 'Segmented',
    code: `import { Segmented } from '@/components/ui/controls';

const [gait, setGait] = useState('walk');
<Segmented
  options={[{ key: 'walk', label: '보행' }, { key: 'trot', label: '속보' }]}
  value={gait} onChange={setGait} />
// 라벨은 한 줄로 잘린다(numberOfLines=1). 5개 넘으면 Select 로.`, from: '@/components/ui/controls', Demo: SegmentedDemo,
    when: '3~4개 배타 선택. 라벨이 길면 한 줄로 잘리므로(numberOfLines=1) 짧게 쓴다. 5개 넘으면 Select.' },
  { name: 'Select',
    code: `import { Select } from '@/components/ui/controls';

<Select label="영상 소스" value={src} onChange={setSrc}
  options={[{ key: 'front', label: '전방 카메라' }, { key: 'rear', label: '후방 카메라' }]} />
// dense 는 92dp 같은 좁은 카드용. label 은 값이 목록에 없을 때 보이는 자리표시 글자.
// 속은 머스캣 RBSelectBox — 웹은 버튼 아래 드롭다운, 네이티브는 가운데 모달 목록.`, from: '@/components/ui/controls', Demo: SelectDemo,
    when: '항목이 많은 드롭다운(머스캣 RBSelectBox). 웹은 버튼 아래, 네이티브는 가운데 모달 목록. dense 는 92dp 카드용.' },
  { name: 'Slider',
    code: `import { Slider } from '@/components/ui/controls';

const [v, setV] = useState(50);
<Slider value={v} onChange={setV} onCommit={(n) => applyToRobot(n)} width="100%" />
// ⚠ onChange = 드래그 중(표시용), onCommit = 손 뗄 때.
//   로봇에 보내는 건 반드시 onCommit 에서만 — 드래그마다 보내면 명령이 폭주한다.
// ⚠속은 머스캣 RBSlider — 트랙을 **탭만 해도**(끌지 않아도) 놓는 순간 onCommit 이 한 번 온다.`, from: '@/components/ui/controls', Demo: SliderDemo,
    when: '0~100 연속값(머스캣 RBSlider). onChange=드래그 중 표시, onCommit=놓을 때(탭 포함) 적용 — 로봇에 보내는 건 반드시 onCommit 에서만.' },
  { name: 'Field / EditField',
    code: `import { Field, EditField } from '@/components/panels/settings/common';

<Field label="로봇 시리얼" value={serial} muted />          // 읽기 전용
<EditField label="로봇 IP" value={ip} onChangeText={setIp} onSubmit={save} />
// 속은 머스캣 RBTextField+RBTextInput — 안드로이드 세로 잘림 보정(androidInput)이 이미 들어 있다. 직접 TextInput 을 쓰면 다시 겪는다.`, from: '@/components/panels/settings/common', Demo: FieldDemo,
    when: '설정 패널의 읽기 전용 값 / 편집 값(머스캣 RBTextField+RBTextInput). mono 폰트 + 안드로이드 세로 잘림 보정이 들어 있다.' },
  { name: 'H2 / Desc / TRow',
    code: `import { H2, Desc, TRow } from '@/components/panels/settings/common';

<H2>보행 파라미터</H2>
<Desc>로봇에 즉시 반영됩니다.</Desc>
<TRow nm="장애물 회피" sub="전방 라이다 기준" right={<Toggle value={on} onChange={setOn} />} />
// 설정 패널은 제목 → 설명 → 항목 행 순서를 지킨다.`, from: '@/components/panels/settings/common', Demo: StructureDemo,
    when: '설정 패널의 문서 구조. 제목 → 설명 → 항목 행 순서를 지키면 패널끼리 결이 맞는다.' },
  { name: 'Group',
    code: `import { H2, Desc, TRow, Group, LegibleText } from '@/components/panels/settings/common';

<LegibleText.Provider value>
  <H2>🎯 IMU 캘리브레이션</H2>
  <Desc>로봇을 평지에 정지 상태로 두고 실행합니다.</Desc>
  <Group>
    <TRow nm="가속도계 보정 (ACC)" sub="앉은 상태에서 실행" right={…} />
  </Group>
</LegibleText.Provider>
// 가시성 모드(캘리브레이션·로봇 점검)에서만 들여쓴 카드를 그린다. 그 밖의 화면에선 children 을 그대로 둔다.`,
    from: '@/components/panels/settings/common', Demo: GroupDemo,
    when: '가시성 모드 화면의 섹션 본문 카드. 제목·설명 아래 행을 들여쓴 카드로 묶어 섹션과 항목을 가른다.' },
  { name: 'SBtn',
    code: `import { SBtn } from '@/components/panels/settings/common';

<SBtn kind="primary" icon="save"    label="저장"   onPress={save} />
<SBtn kind="ghost"   icon="recover" label="되돌리기" onPress={undo} />
<SBtn kind="danger"  icon="trash"   label="삭제"   onPress={remove} />
// danger 는 되돌릴 수 없는 동작에만. ConfirmModal 과 짝지어 쓴다.`, from: '@/components/panels/settings/common', Demo: SBtnDemo,
    when: '설정 패널 액션 버튼 3종(속은 머스캣 RBLabelButton — brand/outlined/danger). danger 는 빨간 면 — 되돌릴 수 없는 동작에만 쓴다.' },
  { name: 'HTabs',
    code: `import { HTabs } from '@/components/ui/HTabs';

<HTabs value={tab} onChange={setTab}
  items={[{ key: 'a', label: '개요', icon: 'gauge' }, { key: 'b', label: '연결', icon: 'wifi' }]} />
// 폰 가로(높이<500dp)에서 세로 내비 대신 쓴다. useCompactH 분기는 호출부 몫.`, from: '@/components/ui/HTabs', Demo: HTabsDemo,
    when: '폰 가로(높이<500dp)에서 세로 내비를 대체하는 가로 칩 탭. 넘치면 가로 스크롤.' },
];

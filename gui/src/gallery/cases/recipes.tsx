import { useState } from 'react';
import { View, Text } from 'react-native';
import { useTheme } from '@/theme';
import { Icon } from '@/components/Icon';
import { Toggle, Slider } from '@/components/ui/controls';
import { ConfirmModal } from '@/components/ui/ConfirmModal';
import { H2, Desc, Field, EditField, TRow, SBtn } from '@/components/panels/settings/common';
import { LEFT_X, LEFT_W } from '@/components/control/LeftPanel';
import { RIGHT_X, RIGHT_W } from '@/components/control/RightPanel';
import type { WidgetCase } from '../types';

function SectionDemo() {
  const [a, setA] = useState(true);
  const [b, setB] = useState(false);
  const [sp, setSp] = useState(60);
  return (
    <View style={{ width: '100%' }}>
      <H2>보행</H2>
      <Desc>로봇에 즉시 반영됩니다. 주행 중에도 바꿀 수 있습니다.</Desc>
      <TRow nm="장애물 회피" sub="전방 라이다 기준" right={<Toggle value={a} onChange={setA} />} />
      <TRow nm="자동 복구" sub="넘어짐 감지 시" right={<Toggle value={b} onChange={setB} />} />
      <TRow nm="최고 속도" sub={`${sp}%`} right={<View style={{ width: 140 }}><Slider value={sp} onChange={setSp} width="100%" /></View>} />
    </View>
  );
}

function DangerDemo() {
  const { c } = useTheme();
  const [ask, setAsk] = useState(false);
  const [done, setDone] = useState(0);
  return (
    <View style={{ gap: 10, alignItems: 'flex-start' }}>
      <SBtn kind="danger" icon="power" label="전원 차단" onPress={() => setAsk(true)} />
      <Text style={{ color: c.dim, fontSize: 11 }}>실행 {done}회</Text>
      {ask && (
        <ConfirmModal title="전원 차단" message="로봇 전원을 차단합니다." confirmLabel="차단" skipKey="demo-pdu"
          onConfirm={() => { setDone((v) => v + 1); setAsk(false); }} onClose={() => setAsk(false)} />
      )}
    </View>
  );
}

function GatedDemo() {
  const { c } = useTheme();
  const [ready, setReady] = useState(false);
  return (
    <View style={{ width: '100%' }}>
      <TRow nm="시연 모드" sub="상시 개방" right={<Toggle value={ready} onChange={setReady} />} />
      <TRow
        nm="카메라 (5V)"
        sub={ready ? '레일 정상' : '레일 상태 수신 전 — 조작 불가'}
        right={
          <View style={{ flexDirection: 'row', alignItems: 'center', gap: 8, opacity: ready ? 1 : 0.45 }}>
            {!ready && <Icon name="warn" size={12} color={c.redTx} />}
            <Toggle value={false} disabled={!ready} />
          </View>
        }
      />
    </View>
  );
}

function FormDemo() {
  const [ip, setIp] = useState('192.168.0.10');
  return (
    <View style={{ width: '100%' }}>
      <View style={{ flexDirection: 'row', gap: 12 }}>
        <Field label="로봇 시리얼" value="RBQ1000000001" muted />
        <EditField label="로봇 IP" value={ip} onChangeText={setIp} />
      </View>
      <View style={{ flexDirection: 'row', gap: 10, justifyContent: 'flex-end' }}>
        <SBtn kind="ghost" icon="recover" label="되돌리기" onPress={() => setIp('192.168.0.10')} />
        <SBtn kind="primary" icon="save" label="저장" onPress={() => {}} />
      </View>
    </View>
  );
}

function SafeAreaDemo() {
  const { c } = useTheme();
  const left = LEFT_X + LEFT_W;
  const right = RIGHT_X + RIGHT_W;
  const band = (w: number, label: string, side: 'left' | 'right') => (
    <View style={{ position: 'absolute', top: 0, bottom: 0, [side]: 0, width: w,
      backgroundColor: c.brand + '22', borderColor: c.brand + '66',
      borderLeftWidth: side === 'right' ? 1 : 0, borderRightWidth: side === 'left' ? 1 : 0,
      alignItems: 'center', justifyContent: 'center' }}>
      <Text style={{ color: c.muted, fontSize: 10, fontWeight: '700' }}>{label}</Text>
      <Text style={{ color: c.dim, fontSize: 10 }}>{w}</Text>
    </View>
  );
  return (
    <View style={{ height: 150, borderRadius: 10, borderWidth: 1, borderColor: c.line,
      backgroundColor: c.sheetB, overflow: 'hidden' }}>
      {band(left, '좌 패널', 'left')}
      {band(right, '우 패널', 'right')}
      <View style={{ position: 'absolute', left: left + 8, right: right + 8, top: 8, bottom: 8,
        borderWidth: 1, borderStyle: 'dashed', borderColor: c.accent, borderRadius: 8,
        alignItems: 'center', justifyContent: 'center' }}>
        <Text style={{ color: c.accent, fontSize: 11, fontWeight: '700' }}>새 오버레이는 여기</Text>
      </View>
    </View>
  );
}

export const recipes: WidgetCase[] = [
  {
    name: '설정 섹션', from: 'H2 + Desc + TRow', size: 'wide', Demo: SectionDemo,
    when: '설정·정비 화면의 기본 골격. 제목 → 설명 → 항목 행 순서를 지키면 화면끼리 결이 맞는다. 항목은 TRow 하나에 하나씩, 오른쪽에 컨트롤 하나.',
    code: `<H2>보행</H2>
<Desc>로봇에 즉시 반영됩니다.</Desc>
<TRow nm="장애물 회피" sub="전방 라이다 기준" right={<Toggle value={a} onChange={setA} />} />
<TRow nm="최고 속도"   sub={\`\${sp}%\`}
  right={<View style={{ width: 140 }}><Slider value={sp} onChange={setSp} width="100%" /></View>} />

// 규약
// · 한 섹션 = H2 하나. 섹션이 늘면 H2 를 늘리지 말고 화면을 나눈다
// · sub 는 "무엇을 기준으로 하는 값인지"를 적는다. 단위·기준이 없으면 사용자가 못 고른다
// · 폭이 필요한 컨트롤(Slider)은 View 로 감싸 폭을 준다 — TRow 는 폭을 정해 주지 않는다`,
  },
  {
    name: '위험 동작', from: 'SBtn(danger) + ConfirmModal', size: 'wide', Demo: DangerDemo,
    when: '되돌릴 수 없는 동작. danger 버튼 하나로 끝내지 않고 반드시 확인을 한 겹 둔다 — 전원·재부팅·펌웨어·삭제.',
    code: `const [ask, setAsk] = useState(false);

<SBtn kind="danger" icon="power" label="전원 차단" onPress={() => setAsk(true)} />
{ask && (
  <ConfirmModal title="전원 차단" message="로봇 전원을 차단합니다." confirmLabel="차단"
    skipKey="pdu-off" onConfirm={doIt} onClose={() => setAsk(false)} />
)}

// 규약
// · danger 버튼은 onPress 에서 바로 실행하지 않는다. 여는 것은 확인창뿐이다
// · confirmLabel 은 "확인"이 아니라 실제 동작 이름("차단")을 쓴다 — 무엇에 동의하는지 보이게
// · skipKey 를 주면 24시간 다시 묻지 않는다. 로봇이 물리적으로 움직이는 동작엔 주지 않는다
// · 더 위험한 것(되돌릴 수 없는 즉시 동작)은 확인창 대신 LongPressButton 홀드를 쓴다
// · SBtn 은 제 콘텐츠 폭을 갖는다. 세로 컨테이너에 넣을 땐 alignItems:'flex-start' 로
//   감싸지 않으면 stretch 되어 폭이 화면만큼 늘어난다`,
  },
  {
    name: '막힌 컨트롤', from: 'TRow + disabled + 사유', size: 'wide', Demo: GatedDemo,
    when: '지금 못 누르는 컨트롤. 흐리게만 두면 고장으로 읽힌다 — 왜 막혔는지를 그 자리에 적는다. 권한·연결·상태 미수신 전부 같은 방식.',
    code: `<TRow
  nm="카메라 (5V)"
  sub={ready ? '레일 정상' : '레일 상태 수신 전 — 조작 불가'}
  right={
    <View style={{ flexDirection: 'row', alignItems: 'center', gap: 8, opacity: ready ? 1 : 0.45 }}>
      {!ready && <Icon name="warn" size={12} color={c.redTx} />}
      <Toggle value={on} disabled={!ready} />
    </View>
  }
/>

// 규약
// · 사유는 sub 에 쓴다. 토스트나 alert 로 띄우지 않는다 — 누른 뒤에야 알게 되면 늦다
// · opacity 는 0.4~0.45. 완전히 숨기지 않는다(있다는 사실은 보여야 다음 행동을 정한다)
// · disabled 를 주면 onPress 를 굳이 undefined 로 또 막지 않는다 — 한 곳에서만 막는다`,
  },
  {
    name: '값 편집 폼', from: 'Field/EditField + SBtn', size: 'wide', Demo: FormDemo,
    when: '읽기 값과 편집 값이 섞인 폼. 액션은 항상 아래 오른쪽에 [되돌리기][저장] 순서.',
    code: `<View style={{ flexDirection: 'row', gap: 12 }}>
  <Field label="로봇 시리얼" value={serial} muted />
  <EditField label="로봇 IP" value={ip} onChangeText={setIp} />
</View>
<View style={{ flexDirection: 'row', gap: 10, justifyContent: 'flex-end' }}>
  <SBtn kind="ghost"   icon="recover" label="되돌리기" onPress={reset} />
  <SBtn kind="primary" icon="save"    label="저장"    onPress={save} />
</View>

// 규약
// · 저장이 오른쪽 끝. 되돌리기가 그 왼쪽. 순서를 화면마다 바꾸지 않는다
// · 값이 로봇에 즉시 가는 항목은 Toggle/Slider 로 두고 저장 버튼을 두지 않는다.
//   저장 버튼이 있다는 건 "누르기 전엔 반영 안 된다"는 약속이다 — 섞지 말 것
// · 읽기 전용은 muted 로. 회색 EditField 로 흉내 내면 눌러 보고 나서야 알게 된다`,
  },
  {
    name: '오버레이 안전 영역', from: 'LeftPanel/RightPanel 의 좌표 상수', size: 'wide', Demo: SafeAreaDemo,
    when: '컨트롤 화면에 무언가를 얹을 때. 좌우는 이미 임자가 있다 — 폭을 눈대중하지 말고 상수를 import 해서 비운다. 전체 좌표 맵(상단 배너·하단 HUD·z 순서)은 ui-conventions.md §4.',
    code: `import { LEFT_X, LEFT_W } from '@/components/control/LeftPanel';
import { RIGHT_X, RIGHT_W } from '@/components/control/RightPanel';

<View style={{ position: 'absolute',
  left:  LEFT_X  + LEFT_W  + 8,   // 좌 패널(모션 버튼) 다음부터
  right: RIGHT_X + RIGHT_W + 8,   // 우 패널(뷰 전환) 앞까지
  top: 54,                        // TopBar 아래
  bottom: bottomReserve }} />     // 조이스틱·속도 HUD 위

// ⚠ 104 처럼 숫자를 박지 않는다. 패널 폭이 바뀌면 새 오버레이만 조용히 어긋난다.
// ⚠ 실제로 센서 뷰가 이걸 안 지켜 좌측 모션 버튼을 통째로 덮었다(2026-09-01).
//    "화면 가운데니까 괜찮겠지" 가 정확히 그 사고의 사고방식이다.`,
  },
];

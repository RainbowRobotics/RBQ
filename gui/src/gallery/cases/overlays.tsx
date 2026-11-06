import { useState } from 'react';
import { View, Text } from 'react-native';
import { useTheme } from '@/theme';
import { Popover, Modal } from '@/components/ui/overlays';
import { ConfirmModal } from '@/components/ui/ConfirmModal';
import { AccessPrompt } from '@/components/ui/AccessPrompt';
import { Segmented } from '@/components/ui/controls';
import { SBtn } from '@/components/panels/settings/common';
import { Tappable } from '@/components/anim';
import { demoBtn } from '../Frame';
import type { WidgetCase } from '../types';

function Opener({ label, render }: { label: string; render: (close: () => void) => React.ReactNode }) {
  const { c } = useTheme();
  const [open, setOpen] = useState(false);
  return (
    <View>
      <Tappable onPress={() => setOpen(true)} style={[demoBtn, { backgroundColor: c.elev, borderWidth: 1, borderColor: c.line }]}>
        <Text style={{ color: c.text, fontSize: 12, fontWeight: '600' }}>{label}</Text>
      </Tappable>
      {open && render(() => setOpen(false))}
    </View>
  );
}

function PopoverDemo() {
  const { c } = useTheme();
  const [pick, setPick] = useState('one');
  return (
    <Opener label="팝오버 열기" render={(close) => (
      <Popover portal={false} onClose={close} dim style={{ top: 44, left: 0, width: 240 }}>
        <Text style={{ color: c.text, fontSize: 12, marginBottom: 8 }}>팝오버 내용</Text>
        <Segmented options={[{ key: 'one', label: '하나' }, { key: 'two', label: '둘' }]} value={pick} onChange={setPick} />
      </Popover>
    )} />
  );
}

function ModalDemo() {
  const { c } = useTheme();
  return (
    <Opener label="모달 열기" render={(close) => (
      <Modal onClose={close}>
        <View style={{ width: 320, borderRadius: 14, padding: 18, gap: 12, backgroundColor: c.panel, borderWidth: 1, borderColor: c.line }}>
          <Text style={{ color: c.text, fontSize: 14, fontWeight: '700' }}>모달 제목</Text>
          <Text style={{ color: c.muted, fontSize: 12 }}>바깥을 누르면 닫힙니다.</Text>
          <SBtn kind="primary" icon="x" label="닫기" onPress={close} />
        </View>
      </Modal>
    )} />
  );
}

function ConfirmDemo() {
  return (
    <Opener label="확인창 열기" render={(close) => (
      <ConfirmModal title="전원 차단" message="로봇 전원을 차단합니다." confirmLabel="차단" onConfirm={close} onClose={close} />
    )} />
  );
}

function AccessDemo() {
  return <Opener label="레벨 승격 창" render={(close) => <AccessPrompt onClose={close} />} />;
}



export const overlays: WidgetCase[] = [
  { name: 'Popover',
    code: `import { Popover } from '@/components/ui/overlays';

const [open, setOpen] = useState(false);
{open && (
  <Popover onClose={() => setOpen(false)} style={{ top: 44, left: 0, width: 240 }}>
    ...
  </Popover>
)}
// 부모가 {open && ...} 로 마운트를 제어해야 퇴장 애니메이션이 재생된다.
// 동시에 여러 개 열지 않는다 — 배경 첫 탭은 닫기로만 소비된다.`, from: '@/components/ui/overlays', Demo: PopoverDemo,
    when: '버튼 아래 드롭다운. 배경 첫 탭은 "닫기"로만 소비된다 — 팝오버를 연 채 상단바가 눌리던 사고 대응.' },
  { name: 'Modal',
    code: `import { Modal } from '@/components/ui/overlays';

{open && (
  <Modal onClose={close}>
    <View style={{ width: 320, borderRadius: 14, padding: 18, backgroundColor: c.panel }}>...</View>
  </Modal>
)}
// ⚠ RN Modal 을 직접 쓸 거면 supportedOrientations={MODAL_ORIENTATIONS} 를 반드시 넘긴다.
//   앱이 landscape 전용이라 빠뜨리면 iOS 에서 SIGABRT 로 죽는다.`, from: '@/components/ui/overlays', Demo: ModalDemo,
    when: '화면 중앙 모달. RN Modal 로 감싸 어디서 열든 전체 화면 위에 뜬다. iOS 는 supportedOrientations 없으면 SIGABRT 로 죽는다.' },
  { name: 'ConfirmModal',
    code: `import { ConfirmModal } from '@/components/ui/ConfirmModal';

{ask && (
  <ConfirmModal title="전원 차단" message="로봇 전원을 차단합니다." confirmLabel="차단"
    skipKey="pdu-off" onConfirm={doIt} onClose={() => setAsk(false)} />
)}
// skipKey 를 주면 24시간 '다시 묻지 않기'가 붙는다. 안 주면 매번 묻는다(auto_start 등).`, from: '@/components/ui/ConfirmModal', Demo: ConfirmDemo,
    when: '되돌릴 수 없는 동작 확인. skipKey 를 주면 24시간 "다시 묻지 않기"가 붙는다 — 안 주면 매번 묻는다.' },
  { name: 'P2gOverlay',
    code: `import { P2gOverlay } from '@/components/P2gOverlay';

// 뷰포트(16:9 클립 박스) 안에 절대배치. active 는 호출부가 판정한다 —
// P2G 토글이 켜져 있고 **전방 카메라**를 보는 중일 때만 true.
<P2gOverlay active={p2gAvail && p2g} />
// 좌표·거리·상태는 로봇이 vision-state DC(id 27)로 보낸다(lib/p2gState.ts).
// 로봇은 값만 보내고 그림은 앱이 그린다 — 종전엔 로봇이 영상에 구워 보냈다.`,
    from: '@/components/P2gOverlay', unavailable: '로봇이 vision-state DC 로 보내는 P2G 상태(useTelemetry)를 읽어 그리는 위젯이라, 전시하려면 그 전역 스토어에 가짜 상태를 써야 한다 — 갤러리를 열었다 나오면 그 값이 앱에 남는다(갤러리 규약, coverage.test). 실물은 전방 카메라에서 P2G 를 켜고 본다. 여기서는 아래 코드와 규약만 가져간다.',
    when: 'Point2Go 로 이동 중인 목표 지점·남은 거리 표시. 로봇이 보내는 정규화 좌표를 뷰포트 픽셀로 옮겨 그린다.' },
  { name: 'AccessPrompt',
    code: `import { AccessPrompt } from '@/components/ui/AccessPrompt';

{needLevel && <AccessPrompt onClose={() => setNeedLevel(false)} />}
// 값을 만지기 전 접근 레벨을 올릴 때만. 레벨 판정 자체는 store 에서 읽는다.`, from: '@/components/ui/AccessPrompt', Demo: AccessDemo,
    when: '개발자 레벨 진입 비밀번호. 값을 만지기 전 접근 레벨을 올릴 때만 띄운다.' },
];

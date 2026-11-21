import { View, Text } from 'react-native';
import { useTheme } from '@/theme';
import { Toggle, Segmented, Slider } from '@/components/ui/controls';
import { useGamepad } from '@/store/gamepad';
import { useTelemetry } from '@/store/telemetry';
import { useSettings, useDevMode } from '@/store/settings';
import { useRobot } from '@/store/robot';
import { useNotMine } from '@/lib/spectating';
import { actions } from '@/lib/rest';
import { refreshProfile } from '@/lib/gamepad/manager';
import { useGamepadProfiles } from '@/store/gamepadProfiles';
import { H2, Desc, TRow, SBtn, S } from './common';
import { t } from '@/lib/i18n';

export function GamepadPanel({ onWizard, onOpenDiag }: {
  onWizard?: () => void;
  onOpenDiag?: () => void;
}) {
  const { c } = useTheme();
  const st = useSettings();
  const devMode = useDevMode();
  const gpActive = useGamepad((s) => s.active);
  const gpDev = useGamepad((s) => s.devices[0] ?? null);
  const customProf = useGamepadProfiles((s) => (gpDev ? s.profiles[gpDev.descriptor] : undefined));
  const ip = useRobot((s) => s.ip);
  const notMine = useNotMine();
  const extManual = useTelemetry((s) => !!s.robot?.extJoy);
  return (
    <>
      <H2>🎮 {t('게임패드')}</H2>
      <Desc>{t('물리 컨트롤러 연결 시 가상 조이스틱은 자동으로 숨겨집니다. 기기는 자동 인식됩니다.')}</Desc>
      <View style={S.statusline}>
        <View style={[S.sdot, { backgroundColor: gpActive ? c.green : c.dim }]} />
        <Text style={{ color: gpActive ? c.green : c.muted, fontSize: 11 }}>
          {gpActive ? `${t('연결됨')} · ${gpActive.name}` : t('연결된 게임패드 없음 — 가상 조이스틱 사용 중')}
        </Text>
      </View>
      {gpActive && (
        <View style={[S.note, { backgroundColor: c.bg, borderColor: c.line2 }]}>
          <Text style={{ color: c.dim, fontSize: 10.5 }}>
            {t('적용 프로파일:')} <Text style={{ color: c.accent2, fontWeight: '700' }}>{gpActive.profileLabel}</Text> {t('(자동 인식)')}
          </Text>
        </View>
      )}
      <TRow nm={t('조이스틱 모드')} sub={t('UI 레이아웃 지정')}
        right={<View style={{ width: 330 }}><Segmented
          options={[{ key: 'auto', label: t('자동') }, { key: 'virtual', label: t('가상 조이스틱') }, { key: 'gamepad', label: t('게임패드') }]}
          value={st.gpUiMode} onChange={st.setGpUiMode} /></View>} />
      {devMode && (
        <>
          <TRow nm={t('게임패드 감도')} sub={t('입력 응답 곡선 (가상 조이스틱과 별개)')}
            right={<View style={{ width: 300 }}><Slider value={st.gpSensitivity} width="100%" onChange={st.setGpSensitivity} /></View>} />
          <TRow nm={t('데드존')} sub={t('중심 무시 범위 · 기기 자체 데드존에 추가 적용')}
            right={<View style={{ width: 300 }}><Slider value={st.gpDeadzone} width="100%" onChange={st.setGpDeadzone} /></View>} />
          <TRow nm={t('외부 조종 (SLAM·SDK)')} sub={t('외부(SLAM·SDK)에 로봇 조종권을 넘김 — Qt SLAM 스위치와 같은 것')}
            right={<Toggle value={extManual} disabled={notMine} onChange={(v) => { actions.gamepadExternal(ip, v).catch(() => {}); }} />} />
        </>
      )}
      {devMode && (
        <>
          <Text style={{ color: c.dim, fontSize: 11, fontWeight: '700', marginTop: 20, marginBottom: 4 }}>{t('개발자 옵션')}</Text>
          <TRow nm={t('전체 버튼 전송')} sub={t('OFF = 키매핑 미사용 · ON = 키매핑 사용')}
            right={<Toggle value={st.gpAllButtons} onChange={st.setGpAllButtons} />} />
          <TRow nm={t('원 스틱 모드')} sub={t('한 손 조작 — 좌우 스틱 역할 스왑')} right={<Toggle value={st.gpOneStick} onChange={st.setGpOneStick} />} />
          <TRow nm={t('크루즈 컨트롤')} sub={t('R3=크루즈 시작 · 방향키 위/아래=속도 증감 (TROT 계열 보행 중에만)')} right={<Toggle value={st.gpCruise} onChange={st.setGpCruise} />} />
          <TRow nm={t('발행 주기 — {hz}Hz').replace('{hz}', String(st.gpPublishHz))} sub={t('조이스틱 프레임 전송 주기 (원본 기본 40Hz — 낮추면 지연↑, 높이면 트래픽↑)')}
            right={<View style={{ width: 300 }}><Slider value={(st.gpPublishHz - 10) / 90 * 100} width="100%"
              onChange={(v) => st.setGpPublishHz(10 + Math.round(v / 100 * 9) * 10)} /></View>} />
          {onOpenDiag && (
            <TRow nm={t('입력 진단')} sub={t('축/버튼 원시값 실시간 확인 (매핑 실측용)')}
              right={<SBtn kind="ghost" icon="gamepad" label={t('진단 열기')} onPress={onOpenDiag} />} />
          )}
          {onWizard && (
            <TRow nm={t('매핑 마법사')} sub={gpDev ? t('안내대로 조작하면 이 기기 전용 프로파일 생성 (미지 기기·레이아웃 이상 시)') : t('컨트롤러가 연결돼 있어야 실행할 수 있습니다')}
              right={<SBtn kind="ghost" icon="gamepad" label={t('마법사 시작')} disabled={!gpDev} onPress={() => onWizard()} />} />
          )}
          {gpDev && customProf && (
            <TRow nm={t('마법사 프로파일 삭제')} sub={t('이 기기의 마법사 프로파일을 지우고 자동 인식으로 되돌립니다')}
              right={<SBtn kind="ghost" icon="trash" label={t('자동 인식 복귀')} onPress={() => {
                useGamepadProfiles.getState().removeProfile(gpDev.descriptor);
                refreshProfile();
              }} />} />
          )}
        </>
      )}
    </>
  );
}

import { View, Text, StyleSheet, useWindowDimensions } from 'react-native';
import { useSafeAreaInsets } from 'react-native-safe-area-context';
import { useTheme } from '@/theme';
import { Popover } from '@/components/ui/overlays';
import { Toggle, Segmented } from '@/components/ui/controls';
import { railAnchor } from '@/components/control/ToolRail';
import { useVisionToggles } from '@/store/visionToggles';
import { useViewport } from '@/store/viewport';
import { useRobot } from '@/store/robot';
import { useHasProjector } from '@/store/capability';
import { actions, restErrorText, WEBRTC_RES } from '@/lib/rest';
import { t } from '@/lib/i18n';

function Row({ nm, sub, on, onChange, disabled }: {
  nm: string; sub?: string; on: boolean; onChange: (v: boolean) => void; disabled?: boolean;
}) {
  const { c } = useTheme();
  return (
    <View style={[styles.row, { borderBottomColor: c.line2, opacity: disabled ? 0.45 : 1 }]}>
      <View style={{ flex: 1 }}>
        <Text style={{ color: c.text, fontSize: 11.5, fontWeight: '600' }}>{nm}</Text>
        {!!sub && <Text style={{ color: c.dim, fontSize: 9.5, marginTop: 2 }}>{sub}</Text>}
      </View>
      <Toggle value={on} onChange={disabled ? undefined : onChange} disabled={disabled} />
    </View>
  );
}

export function VisionPopover({ onClose }: { onClose: () => void }) {
  const { c } = useTheme();
  const { height: winH } = useWindowDimensions();
  const hasProjector = useHasProjector();
  const insets = useSafeAreaInsets();
  const vt = useVisionToggles();
  const ip = useRobot((s) => s.ip);
  const p2g = useViewport((s) => s.p2g);
  const frontView = useViewport((s) => s.key) === 'front';
  const setP2g = useViewport((s) => s.setP2g);
  const toP2g = (v: boolean) => {
    setP2g(v);
    actions.visionPoint2Go(ip, v).catch((e: unknown) => {
      setP2g(!v);
      useRobot.getState().pushLog({ ts: '', process: 'App', level: 'ERROR',
        msg: `P2G ${v ? 'ON' : 'OFF'} 실패 — ${restErrorText(e)}` });
    });
  };
  const pos = { ...railAnchor(winH, 'right', insets.right), width: 250 };
  return (
    <Popover onClose={onClose} style={pos}>
      <Text style={styles.head}>{t('비전')}</Text>
      <Row nm="Point to Go" disabled={!frontView} on={p2g} onChange={toP2g}
        sub={frontView ? t('전방 영상의 바닥을 탭한 지점으로 이동') : t('전방 카메라 소스에서만 사용')} />
      <Row nm={t('야간 모드')} sub={t('저조도 촬영')} on={vt.night} onChange={(v) => vt.toNight(ip, v)} />
      <Row nm={t('스텔스')} sub={t('LED·표시등 소등')} on={vt.stealth} onChange={(v) => vt.toStealth(ip, v)} />
      <Row nm="GUIDE" sub={t('주행 가이드선')} on={vt.guide} onChange={(v) => vt.toGuide(ip, v)} />
      <Row nm="FACE" sub={t('얼굴 검출')} on={vt.faceDetect} onChange={(v) => vt.toFace(ip, v)} />
      <Row nm={t('DOCK 스캔')} sub={t('도킹 마커 탐색')} on={vt.dockScan} onChange={(v) => vt.toDockScan(ip, v)} />
      <Row nm={t('높이맵')} sub={t('지형 고저 표시')} on={vt.hmStairs}
        onChange={(v) => vt.putHeightmap(ip, v, vt.hmEdge)} />
      <Row nm={t('└ 경계선 강조')} on={vt.hmEdge} disabled={!vt.hmStairs}
        onChange={(v) => vt.putHeightmap(ip, vt.hmStairs, v)} />
      {hasProjector && <Row nm={t('IR 프로젝터')} sub={t('깊이 카메라 점무늬 투사')}
        on={vt.irProjector} onChange={(v) => vt.toProjector(ip, v)} />}
      <View style={styles.res}>
        <Text style={{ color: c.text, fontSize: 11.5, fontWeight: '600' }}>{t('영상 해상도')}</Text>
        <Segmented options={WEBRTC_RES.map((r, i) => ({ key: String(i), label: `${r.height}p` }))}
          value={String(vt.resIdx)}
          onChange={(v) => { const i = Number(v); vt.toRes(ip, i, WEBRTC_RES[i].width, WEBRTC_RES[i].height); }} />
      </View>
    </Popover>
  );
}

const styles = StyleSheet.create({
  head: { color: '#8b96a3', fontSize: 11, fontWeight: '700', marginBottom: 6 },
  row: { flexDirection: 'row', alignItems: 'center', gap: 10, paddingVertical: 6.5, borderBottomWidth: 1 },
  res: { gap: 6, paddingTop: 8 },
});

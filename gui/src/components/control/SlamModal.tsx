import { useEffect, useState } from 'react';
import { View, Text, StyleSheet, useWindowDimensions } from 'react-native';
import { useSafeAreaInsets } from 'react-native-safe-area-context';
import { useTheme } from '@/theme';
import { Icon } from '@/components/Icon';
import { Tappable } from '@/components/anim';
import { Popover } from '@/components/ui/overlays';
import { Segmented } from '@/components/ui/controls';
import { useRobot } from '@/store/robot';
import { useSettings } from '@/store/settings';
import { connection } from '@/lib/connection';
import { actions } from '@/lib/rest';
import { slam, SLAM_REQ } from '@/lib/slam';
import { useViewport } from '@/store/viewport';
import { useAvailableSources } from '@/lib/visionSources';
import { useTelemetry } from '@/store/telemetry';
import { railAnchor } from '@/components/control/ToolRail';
import { t } from '@/lib/i18n';
import { slam as slamExt, type SlamGroup as Group } from '@/modules/registry';
import { useDeviceToggles } from '@/store/deviceToggles';

const GAIT_STANDING = 1;
const PLOT_VIEW_2D = 0;
const PLOT_VIEW_3D = 1;


export function SlamModal({ onClose }: { onClose: () => void }) {
  const { c, radius } = useTheme();
  const setVpKey = useViewport((v) => v.setKey);
  const mapSourceReady = useAvailableSources().some((v) => v.key === 'slam');
  const ip = useRobot((s) => s.ip);
  const gaitId = useRobot((s) => s.robot?.gait_id);
  const accessLevel = useSettings((s) => s.accessLevel);
  const slamVerIdx = useSettings((s) => s.slamVersion);
  const variant = slamExt.variants.find((v) => v.slamVersion === slamVerIdx);
  const version = variant?.key ?? 'default';
  const setVersion = (key: string) => {
    const next = slamExt.variants.find((v) => v.key === key)?.slamVersion ?? 0;
    if (key !== version) useSettings.getState().setSlamVersion(next);
  };
  const st = useTelemetry((s) => s.slamState);
  const follow = useDeviceToggles((s) => s.slamFollow);
  const setFollow = (v: boolean) => useDeviceToggles.setState({ slamFollow: v });

  useEffect(() => { actions.visionProgram(ip, 5, true).catch(() => {}); }, [ip]);

  const hlc = (on: boolean) => actions.gamepadExternal(ip, on).catch(() => {});
  const walkIfStanding = () => { if (gaitId === GAIT_STANDING) connection.sendMotion('walk', true); };

  const view: Group = { title: 'View', rows: [
    { label: '2D', active: st ? st.plotView === PLOT_VIEW_2D : undefined,
      onPress: () => slam.bool(SLAM_REQ.view2d, true) },
    { label: '3D', active: st ? st.plotView === PLOT_VIEW_3D : undefined,
      onPress: () => slam.bool(SLAM_REQ.view3d, true) },
    { label: `Follow ${follow ? 'ON' : 'OFF'}`, active: follow,
      onPress: () => { const nv = !follow; setFollow(nv); slam.bool(SLAM_REQ.viewFollow, nv); } },
  ] };
  const groups: Group[] = variant ? variant.groups({ st, hlc, view }) : [
    view,
    { title: t('맵핑 · Mapping'), minLevel: 2, rows: [
      { label: 'Start', active: st?.mapBuilding,
        onPress: () => { slam.bool(SLAM_REQ.mapBuild, true); hlc(false); walkIfStanding(); } },
      { label: 'Stop', onPress: () => slam.bool(SLAM_REQ.mapStop, true) },
      { label: 'Save', active: st?.mapSaved, onPress: () => slam.bool(SLAM_REQ.mapSave, true) },
      { label: 'Reload', active: st?.mapReloaded, onPress: () => slam.bool(SLAM_REQ.mapReload, true) },
    ] },
    { title: t('경로 지정 · Annotate'), minLevel: 2, rows: [
      { label: 'Clear', onPress: () => slam.bool(SLAM_REQ.clearTopo, true) },
      { label: 'Start', active: st?.annotationMode || st?.quickAnnotation, onPress: () => {
        slam.bool(SLAM_REQ.annotModeOnOff, true); slam.bool(SLAM_REQ.quickAnnotOnOff, true);
        hlc(false); walkIfStanding();
      } },
      { label: 'Save', active: st?.annotationSaved,
        onPress: () => { slam.bool(SLAM_REQ.annotSave, true); slam.bool(SLAM_REQ.annotModeOnOff, false); } },
    ] },
    { title: t('경로 주행 · Navigation'), minLevel: 2, rows: [
      { label: t('정방향 (Go Initial)'), active: st?.driveStarted,
        onPress: () => { slam.bool(SLAM_REQ.scheduleStart, true); hlc(true); walkIfStanding(); } },
      { label: t('역방향 (Go Last)'), active: st?.autoTravel,
        onPress: () => { slam.bool(SLAM_REQ.rtb, true); hlc(true); walkIfStanding(); } },
      { label: 'Stop', onPress: () => { slam.bool(SLAM_REQ.eStop, true); hlc(false); } },
    ] },
    { title: t('세팅 · Settings'), minLevel: 3, rows: [
      { label: t('실내 (Indoor)'), disabled: true, note: t('로봇 미지원') },
      { label: t('실외 (Outdoor)'), disabled: true, note: t('로봇 미지원') },
      { label: t('연결 (Connect)'), disabled: true, note: t('로봇 미지원') },
      { label: 'Localization', active: st?.locaStarted || st?.locaInit,
        onPress: () => slam.bool(SLAM_REQ.autoInit, true) },
    ] },
  ];

  const { height: winH } = useWindowDimensions();
  const insets = useSafeAreaInsets();
  const [openGroup, setOpenGroup] = useState<string | null>(null);
  const pos = { ...railAnchor(winH, 'right', insets.right), width: 250 };
  return (
    <Popover onClose={onClose} style={pos}>
        <View style={styles.popH}>
          <Icon name="route" size={13} color={c.accent2} />
          <Text style={{ fontSize: 12, fontWeight: '700', color: c.text }}>{t('SLAM / 내비')}</Text>
        </View>
        <Text style={{ color: st ? c.accent2 : c.dim, fontSize: 9, marginBottom: 8 }}>
          {st ? t('로봇 상태 연동') : t('상태 수신 대기 — 마지막 선택 기준')}
        </Text>
        {slamExt.variants.length > 0 && (
          <>
            <Segmented options={[{ key: 'default', label: slamExt.defaultLabel }, ...slamExt.variants.map((v) => ({ key: v.key, label: v.label }))]}
              value={version} onChange={setVersion} />
            <View style={{ height: 10 }} />
          </>
        )}
        <Tappable disabled={!mapSourceReady} onPress={() => setVpKey('slam')}
          style={[styles.mapLink, {
            borderColor: c.line, borderRadius: radius.md, backgroundColor: c.elev,
            opacity: mapSourceReady ? 1 : 0.45,
          }]}>
          <Icon name="route" size={13} color={c.accent2} />
          <Text style={{ color: c.text, fontSize: 11, fontWeight: '700' }}>
            {mapSourceReady ? t('맵을 화면에 크게 보기') : t('맵 뷰 없음 — LiDAR 미감지')}
          </Text>
        </Tappable>
        {variant?.estop && (
          <Tappable onPress={() => { slam.bool(SLAM_REQ.eStop, true); hlc(false); }}
            style={[styles.estop, { borderRadius: radius.md, borderColor: c.dangerLine, backgroundColor: 'rgba(231,51,28,0.12)' }]}>
            <Icon name="estop" size={14} color={c.redbright} />
            <Text style={{ color: c.redTx, fontSize: 11, fontWeight: '700' }}>SLAM E-STOP</Text>
          </Tappable>
        )}
        {accessLevel < 2 && (
          <Text style={{ color: c.dim, fontSize: 9, lineHeight: 13, marginBottom: 10 }}>
            {t('맵핑·경로·주행은 개발자 모드(2단계)부터 — 설정에서 전환')}
          </Text>
        )}
        {openGroup === null ? (
          groups.filter((g) => accessLevel >= (g.minLevel ?? 1)).map((g) => (
            <Tappable key={g.title} onPress={() => setOpenGroup(g.title)}
              style={[styles.gHead, { borderColor: c.line, borderRadius: radius.sm, backgroundColor: c.elev }]}>
              <Text style={{ color: c.text, fontSize: 10.5, fontWeight: '700', flex: 1 }}>{g.title.toUpperCase()}</Text>
              <Text style={{ color: c.dim, fontSize: 11 }}>{'\u203a'}</Text>
            </Tappable>
          ))
        ) : (
          <>
            <Tappable onPress={() => setOpenGroup(null)}
              style={[styles.gHead, { borderColor: c.line, borderRadius: radius.sm, backgroundColor: c.elev, marginBottom: 8 }]}>
              <Text style={{ color: c.accent2, fontSize: 11, fontWeight: '700', marginRight: 6 }}>{'\u2039'}</Text>
              <Text style={{ color: c.text, fontSize: 10.5, fontWeight: '700', flex: 1 }}>{openGroup.toUpperCase()}</Text>
            </Tappable>
            <View style={styles.rows}>
              {(groups.find((g) => g.title === openGroup)?.rows ?? []).map((r) => (
                <Tappable key={r.label} disabled={r.disabled} onPress={r.onPress}
                  style={[styles.btn, {
                    borderRadius: radius.sm, opacity: r.disabled ? 0.4 : 1,
                    backgroundColor: r.active ? 'rgba(77,156,245,0.12)' : c.elev,
                    borderColor: r.active ? 'rgba(77,156,245,0.5)' : c.line,
                  }]}>
                  <Text style={{ color: r.active ? c.accent2 : c.text, fontSize: 10.5, fontWeight: '600' }}>{r.label}</Text>
                  {r.note && <Text style={{ color: c.dim, fontSize: 8 }}>{r.note}</Text>}
                </Tappable>
              ))}
            </View>
          </>
        )}
    </Popover>
  );
}

const styles = StyleSheet.create({
  popH: { flexDirection: 'row', alignItems: 'center', gap: 7, marginBottom: 3 },
  mapLink: { flexDirection: 'row', alignItems: 'center', justifyContent: 'center', gap: 7, height: 34, borderWidth: 1, marginBottom: 10 },
  gHead: { flexDirection: 'row', alignItems: 'center', height: 30, paddingHorizontal: 9, borderWidth: 1, marginBottom: 6 },
  rows: { flexDirection: 'row', flexWrap: 'wrap', gap: 7 },
  btn: { minHeight: 30, paddingHorizontal: 12, paddingVertical: 5, borderWidth: 1, alignItems: 'center', justifyContent: 'center' },
  estop: { flexDirection: 'row', alignItems: 'center', justifyContent: 'center', gap: 6, height: 34, borderWidth: 1, marginBottom: 10 },
});

import { useEffect, useState } from 'react';
import { View, Text, StyleSheet } from 'react-native';
import { useTheme } from '@/theme';
import { Icon } from '@/components/Icon';
import { Tappable } from '@/components/anim';
import { TRow } from '@/components/panels/PowerPanel';
import { openCameraCalib, askCancelCameraCalib, useCamCalibRunning } from '@/components/CameraCalib';
import { useRobot } from '@/store/robot';
import { useTelemetry } from '@/store/telemetry';
import { programHealth } from '@/lib/visionProgramStates';
import { useDevMode } from '@/store/settings';
import { actions } from '@/lib/rest';
import { webrtcClient } from '@/lib/webrtcClient';
import { t } from '@/lib/i18n';

export const VISION_PROGRAMS: { id: number; name: string }[] = [
  { id: 0, name: 'Vision' }, { id: 1, name: 'HAL' }, { id: 2, name: 'Streamer' },
  { id: 3, name: 'Handeye' }, { id: 4, name: 'Heightmap' },
  { id: 6, name: 'Cctv' }, { id: 7, name: 'Ptz' }, { id: 8, name: 'Thermal' },
];

export function VisionPanel() {
  const { c, radius } = useTheme();
  const ip = useRobot((s) => s.ip);
  const devMode = useDevMode();
  const calibRunning = useCamCalibRunning();
  useEffect(() => { if (ip) webrtcClient.ensureConnected(ip); }, [ip]);
  const [resetting, setResetting] = useState(false);
  const [err, setErr] = useState('');
  const programs = useTelemetry((s) => s.visionPrograms);
  const [progOn, setProgOn] = useState<Record<number, boolean>>({});

  const doReset = async () => {
    setErr(''); setResetting(true);
    try { await actions.visionReset(ip); }
    catch (e: any) { setErr(String(e?.message ?? e)); }
    setTimeout(() => setResetting(false), 8000);
  };
  const toProgram = async (id: number, running: boolean) => {
    setErr('');
    setProgOn((p) => ({ ...p, [id]: running }));
    try { await actions.visionProgram(ip, id, running); }
    catch (e: any) { setErr(String(e?.message ?? e)); setProgOn((p) => ({ ...p, [id]: !running })); }
  };
  const calibBtn = (kind: 'jig' | 'oak') => (
    <Tappable onPress={() => (calibRunning ? askCancelCameraCalib() : openCameraCalib(kind))}
      style={[styles.action, { borderColor: calibRunning ? 'rgba(210,153,34,0.5)' : c.line, backgroundColor: calibRunning ? 'rgba(210,153,34,0.10)' : c.elev, borderRadius: radius.sm }]}>
      <Text style={{ color: calibRunning ? c.amber : c.text, fontSize: 11, fontWeight: '700' }}>{calibRunning ? t('취소') : t('실행')}</Text>
    </Tappable>
  );

  return (
    <View>
      <Text style={{ color: c.dim, fontSize: 11 }}>
        {t('야간 모드 · IR 프로젝터 · 영상 해상도 같은 비전 토글은 제어 화면의 비전 팝업에 있습니다.')}
      </Text>

      {devMode && (
        <>
          <Text style={{ color: c.dim, fontSize: 11, fontWeight: '700', marginTop: 16, marginBottom: 2 }}>{t('카메라 캘리브레이션')}</Text>
          <TRow nm={t('Jig Extrinsic 보정')} sub={t('지그 마커로 base↔센서 TF 보정')} right={calibBtn('jig')} />
          <TRow nm={t('OAK-D Intrinsic 보정')} sub={t('보행하며 전·후방 스테레오 보정 — 로봇이 약 10분 스스로 걷습니다')} right={calibBtn('oak')} />
        </>
      )}

      <Text style={{ color: c.dim, fontSize: 11, fontWeight: '700', marginTop: 16, marginBottom: 2 }}>{t('데몬 제어')}</Text>
      <TRow nm="VISION RESET" sub={t('Handler가 HAL을 재시작 — 카메라 파이프라인 클린 재시작 (영상 수 초 끊김)')}
        right={
          <Tappable disabled={resetting} onPress={doReset}
            style={[styles.action, { borderColor: 'rgba(210,153,34,0.5)', backgroundColor: 'rgba(210,153,34,0.10)', borderRadius: radius.sm, opacity: resetting ? 0.5 : 1 }]}>
            <Icon name="recover" size={13} color={c.amber} />
            <Text style={{ color: c.amber, fontSize: 11, fontWeight: '700' }}>{resetting ? t('리셋 중…') : t('리셋')}</Text>
          </Tappable>
        } />
      {devMode && (
        <>
          <Text style={{ color: c.dim, fontSize: 11, fontWeight: '700', marginTop: 10, marginBottom: 6 }}>
            Vision Programs <Text style={{ fontWeight: '400' }}>{t('· 개발자 — 데몬 수동 기동/정지')}</Text>
          </Text>
          <View style={styles.progGrid}>
            {VISION_PROGRAMS.map((p) => {
              const h = programs ? programHealth(programs[p.id]) : null;
              const on = programs ? h === 'run' : !!progOn[p.id];
              return (
                <Tappable key={p.id} onPress={() => toProgram(p.id, !on)}
                  style={[styles.progBtn, {
                    borderRadius: radius.sm,
                    backgroundColor: on ? 'rgba(63,185,80,0.10)' : c.elev,
                    borderColor: on ? 'rgba(63,185,80,0.5)' : c.line,
                  }]}>
                  <View style={[styles.progDot, { backgroundColor: on ? c.green : c.dim }]} />
                  <Text style={{ color: on ? c.greenTx : c.text, fontSize: 11, fontWeight: '600' }}>{p.name}</Text>
                  <Text style={{ color: c.dim, fontSize: 9 }}>{on ? t('정지') : t('시작')}</Text>
                </Tappable>
              );
            })}
          </View>
        </>
      )}

      {err ? <Text style={{ color: c.redbright, fontSize: 11, marginTop: 10 }}>{err}</Text> : null}
    </View>
  );
}

const styles = StyleSheet.create({
  action: { flexDirection: 'row', alignItems: 'center', gap: 6, height: 34, paddingHorizontal: 14, borderWidth: 1 },
  progGrid: { flexDirection: 'row', flexWrap: 'wrap', gap: 7 },
  progBtn: { flexDirection: 'row', alignItems: 'center', gap: 6, height: 32, paddingHorizontal: 11, borderWidth: 1 },
  progDot: { width: 7, height: 7, borderRadius: 4 },
});

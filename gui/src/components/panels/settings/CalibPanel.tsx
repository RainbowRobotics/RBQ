import { useState } from 'react';
import { View, Text, StyleSheet } from 'react-native';
import { useTheme } from '@/theme';
import { CalibRow, TRow as RowT, type ConfirmReq } from '@/components/panels/PowerPanel';
import { CameraBasicCheckModal, DepthOnChipModal, DepthExtrinsicModal } from '@/components/CameraCheckModals';
import { ConfirmModal } from '@/components/ui/ConfirmModal';
import {
  useCalibGuard, useZmpCalibGuard, zmpCalibAllowed,
  useSitImuCalibGuard, sitImuCalibAllowed, commissioning,
} from '@/lib/commissioning';
import { ZmpCalibModal } from '@/components/ZmpCalibModal';
import { GyroBiasModal, GyroCalibHint } from '@/components/GyroBiasModal';
import { AccCalibModal } from '@/components/AccCalibModal';
import { ImuNullModal } from '@/components/ImuNullModal';
import { SitRobotHint } from '@/components/SitRobotHint';
import { ImuLevelHint } from '@/components/BubbleLevel';
import { DynamicsPanel } from '@/components/panels/settings/DynamicsPanel';
import { H2, Desc, LegibleText, Group } from './common';
import { Tappable } from '@/components/anim';
import { LegHomeCalibration } from '@/components/LegHomeCalibration';
import { t } from '@/lib/i18n';

export function CalibPanel() {
  const { c } = useTheme();
  const calib = useCalibGuard();
  const zmpGuard = useZmpCalibGuard();
  const sitGuard = useSitImuCalibGuard();
  const [confirmReq, setConfirmReq] = useState<ConfirmReq | null>(null);

  const [zmpOpen, setZmpOpen] = useState(false);
  const [gyroOpen, setGyroOpen] = useState(false);
  const [accOpen, setAccOpen] = useState(false);
  const [imuOpen, setImuOpen] = useState(false);
  const [camModal, setCamModal] = useState<null | 'basic' | 'onchip' | 'extrinsic'>(null);

  return (
    <>
      <LegibleText.Provider value>
      <H2>{t('IMU 캘리브레이션')}</H2>
      <Desc>{t('IMU 드리프트 현장 보정 — 로봇을 평지에 정지 상태로 두고 실행합니다.')}</Desc>

      <Group>
      <CalibRow nm={t('가속도계 보정 (ACC)')} sub={t('앉은 상태에서 평지에 두고 실행 — 약 2초')}
        blocked={sitGuard.blocked} blockReason={sitGuard.reason} allow={sitImuCalibAllowed} fix={sitGuard.fix}
        confirmMsg={t('로봇이 앉은 상태에서만 실행할 수 있습니다. 2초 동안 중력 크기를 평균 내 보정값을 저장합니다 — 그동안 로봇을 건드리지 마세요.')}
        confirmExtra={<SitRobotHint />}
        fire={() => setAccOpen(true)} request={setConfirmReq} />

      <CalibRow nm={t('IMU 롤/피치 영점')} sub={t('현재 자세를 수평 기준으로 영점 — 약 0.5초, 자세 유지')}
        blocked={calib.blocked} blockReason={calib.reason} fix={calib.fix}
        confirmMsg={t('지금 자세가 수평 기준이 됩니다. 기울어진 상태로 실행하면 이후 제어가 전부 틀어집니다.')}
        confirmExtra={<ImuLevelHint />}
        onRequest={commissioning.standIfStanding}
        fire={() => setImuOpen(true)} request={setConfirmReq} />

      <CalibRow nm={t('자이로 바이어스 보정')} sub={t('앉은 상태에서 실행 — IMU 리셋 후 재측정, 약 5초')}
        blocked={sitGuard.blocked} blockReason={sitGuard.reason} allow={sitImuCalibAllowed} fix={sitGuard.fix}
        confirmMsg={t('로봇이 앉은 상태에서만 실행할 수 있습니다. 실행하면 IMU 를 리셋하고 값이 자리 잡기를 기다린 뒤 자이로 값을 다시 측정합니다.')}
        confirmExtra={<GyroCalibHint />}
        fire={() => setGyroOpen(true)} request={setConfirmReq} />

      <Text style={{ color: c.amberTx, fontSize: 11, marginTop: 6, paddingBottom: 10 }}>
        ⚠ {t('보정 명령은 로봇 정지 상태에서만 — 주행 중 실행 금지')}
      </Text>
      </Group>

      <View style={styles.divider} />

      <H2>{t('무게 중심 보정 ')}<Text style={{ color: c.muted, fontSize: 11 }}>ZMP · Zero Moment Point</Text></H2>
      <Desc>{t('서 있는 자세에서 무게중심 오프셋을 자동으로 찾습니다. 수렴하면 로봇이 스스로 저장하고 STAND 로 돌아옵니다 — 그동안 건드리지 마세요.')}</Desc>

      <Group>
      <CalibRow nm={t('무게 중심 오프셋 자동 보정')} sub={t('서 있는 상태에서 실행 — 한 걸음 정렬 후 수렴까지 최대 60초')}
        blocked={zmpGuard.blocked} blockReason={zmpGuard.reason} allow={zmpCalibAllowed} fix={zmpGuard.fix}
        confirmMsg={t('보정이 끝날 때까지 로봇을 건드리지 마세요. 실행하면 로봇이 한 걸음 걸어 다리를 정렬한 뒤 자동 조정에 들어갑니다.')}
        fire={() => setZmpOpen(true)} request={setConfirmReq} />
      </Group>

      <View style={styles.divider} />

      <LegHomeCalibration />

      <View style={styles.divider} />

      <H2>{t('카메라 점검 · 보정 ')}<Text style={{ color: c.muted, fontSize: 11 }}>Camera Check</Text></H2>
      <Desc>{t('카메라 시야와 FPS 를 확인하고, 하단 depth 카메라의 품질과 위치를 보정합니다.')}</Desc>
      <Group>
        <RunRow nm={t('카메라 기본 검사')} sub={t('카메라 6개 화면과 FPS 를 띄워 시야를 확인합니다')} onPress={() => setCamModal('basic')} />
        <RunRow nm={t('하단 depth 카메라 On-chip 캘리브레이션')} sub={t('depth 품질이 떨어졌을 때 — 평평한 바닥에 세워 두고 실행합니다')} onPress={() => setCamModal('onchip')} />
        <RunRow nm={t('하단 depth 카메라 위치 보정')} sub={t('하단 depth 카메라들의 상대 위치를 맞춥니다 — 로봇 하단에 마커보드가 필요합니다')} onPress={() => setCamModal('extrinsic')} />
      </Group>

      <View style={styles.divider} />

      <DynamicsPanel />
      </LegibleText.Provider>

      {zmpOpen && <ZmpCalibModal onClose={() => setZmpOpen(false)} />}
      {gyroOpen && <GyroBiasModal onClose={() => setGyroOpen(false)} />}
      {accOpen && <AccCalibModal onClose={() => setAccOpen(false)} />}
      {imuOpen && <ImuNullModal onClose={() => setImuOpen(false)} />}
      {camModal === 'basic' && <CameraBasicCheckModal onClose={() => setCamModal(null)} />}
      {camModal === 'onchip' && <DepthOnChipModal onClose={() => setCamModal(null)} />}
      {camModal === 'extrinsic' && <DepthExtrinsicModal onClose={() => setCamModal(null)} />}
      {confirmReq && (
        <View>
          <ConfirmModal title={confirmReq.title} message={confirmReq.message} confirmLabel={confirmReq.confirmLabel}
            danger={confirmReq.danger} skipKey={confirmReq.skipKey}
            onConfirm={() => { confirmReq.run(); setConfirmReq(null); }} onClose={() => setConfirmReq(null)}>
            {confirmReq.extra}
          </ConfirmModal>
        </View>
      )}
    </>
  );
}

function RunRow({ nm, sub, onPress }: { nm: string; sub: string; onPress: () => void }) {
  const { c, radius } = useTheme();
  return (
    <RowT nm={nm} sub={sub} right={
      <Tappable onPress={onPress} style={[styles.run, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.sm }]}>
        <Text style={{ color: c.text, fontSize: 19, fontWeight: '800' }}>{t('실행')}</Text>
      </Tappable>
    } />
  );
}

const styles = StyleSheet.create({
  divider: { height: 28 },
  run: { alignItems: 'center', justifyContent: 'center', height: 42, minWidth: 112, paddingHorizontal: 18, borderWidth: 1 },
});

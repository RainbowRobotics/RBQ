import { useState } from 'react';
import { View, Text } from 'react-native';
import { useTheme } from '@/theme';
import { TRow, DriveRailRow, PDU_RAILS, DRIVE_RAILS, type ConfirmReq } from '@/components/panels/PowerPanel';
import { Toggle } from '@/components/ui/controls';
import { ConfirmModal } from '@/components/ui/ConfirmModal';
import { useTelemetry } from '@/store/telemetry';
import { useRobot } from '@/store/robot';
import { actions } from '@/lib/rest';
import { t } from '@/lib/i18n';
import { useFeatureCanFd } from '@/store/capability';
import { FdPowerRails } from '@/components/panels/FdPowerPanel';

export function PowerBasicPanel() {
  const { c } = useTheme();
  const ip = useRobot((s) => s.ip);
  const pdu = useTelemetry((s) => s.pdu);
  const safe = PDU_RAILS.filter((r) => !r.danger && !r.noL1);
  const drive = DRIVE_RAILS.filter((r) => r.l1);
  const [confirmReq, setConfirmReq] = useState<ConfirmReq | null>(null);
  const canFd = useFeatureCanFd();

  if (canFd) {
    return (
      <>
        <Text style={{ color: c.text, fontSize: 16, fontWeight: '700', marginBottom: 3 }}>⚡ {t('전원 제어')}</Text>
        <Text style={{ color: c.dim, fontSize: 11, marginBottom: 16 }}>{t('일상 운용에서 켜고 끄는 보조 전원입니다.')}</Text>
        <FdPowerRails level="l1" />
        <View style={{ marginTop: 12, padding: 10, borderWidth: 1, borderColor: c.line, borderRadius: 8 }}>
          <Text style={{ color: c.amberTx, fontSize: 10.5, lineHeight: 16 }}>
            ⚠ {t('카메라·통신·Vision PC 전원과 LEGS(48V), LiDAR, 시스템 재부팅은 [정비] 탭에 있습니다 — 끄면 연결·자세·주행이 끊기는 조작이라 레벨을 낮추지 않았습니다.')}
          </Text>
        </View>
      </>
    );
  }

  return (
    <>
      <Text style={{ color: c.text, fontSize: 16, fontWeight: '700', marginBottom: 3 }}>⚡ {t('전원 제어')}</Text>
      <Text style={{ color: c.dim, fontSize: 11, marginBottom: 16 }}>
        {t('일상 운용에서 켜고 끄는 보조 전원입니다.')}{pdu ? '' : t(' — 레일 상태 수신 대기 중…')}
      </Text>

      {safe.map((r) => (
        <TRow key={r.port} nm={t(r.nm)} sub={t(r.sub)}
          right={<Toggle value={pdu ? pdu[r.bit] : false} disabled={!pdu}
            onChange={(v) => actions.pduPower(ip, r.port, v).catch(() => {})} />} />
      ))}

      <Text style={{ color: c.dim, fontSize: 10, marginTop: 6 }}>
        {t('토글은 로봇 PDU의 실제 상태(Motion TCP)를 표시 — 반영에 1초 정도 걸릴 수 있습니다.')}
      </Text>

      <Text style={{ color: c.redbright, fontSize: 11, fontWeight: '700', marginTop: 16, marginBottom: 2 }}>
        {t('⚠ 48V 팔(ARM)·애드온·외부 확장 — 확인 후 실행')}
      </Text>
      {drive.map((r) => (
        <DriveRailRow key={r.port} nm={t(r.nm)} sub={t(r.sub)} on={pdu ? pdu[r.bit] : false}
          volts={pdu?.rails?.[r.railKey]?.v ?? null}
          blocked={!pdu} blockReason={t('레일 상태 수신 전 — 조작 불가')}
          fire={(next) => actions.pduPower(ip, r.port, next).catch(() => {})} request={setConfirmReq} />
      ))}

      <View style={{ marginTop: 12, padding: 10, borderWidth: 1, borderColor: c.line, borderRadius: 8 }}>
        <Text style={{ color: c.amberTx, fontSize: 10.5, lineHeight: 16 }}>
          ⚠ {t('카메라·통신·Vision PC 전원과 LEGS(48V), LiDAR, 시스템 재부팅은 [정비] 탭에 있습니다 — 끄면 연결·자세·주행이 끊기는 조작이라 레벨을 낮추지 않았습니다.')}
        </Text>
      </View>

      {confirmReq && (
        <ConfirmModal title={confirmReq.title} message={confirmReq.message} confirmLabel={confirmReq.confirmLabel}
          danger={confirmReq.danger} skipKey={confirmReq.skipKey}
          onConfirm={() => { confirmReq.run(); setConfirmReq(null); }} onClose={() => setConfirmReq(null)} />
      )}
    </>
  );
}

import { useEffect } from 'react';
import { ConfirmModal } from '@/components/ui/ConfirmModal';
import { useStandGate, standBlockText } from '@/lib/standGate';
import { useRobot } from '@/store/robot';
import { t } from '@/lib/i18n';

export function StandGateModal() {
  const p = useStandGate((s) => s.pending);
  const close = useStandGate((s) => s.close);
  const ip = useRobot((s) => s.ip);
  const stale = !!p?.ip && p.ip !== ip;
  useEffect(() => { if (stale) close(); }, [stale, close]);
  useEffect(() => { if (p && !p.ip && ip) useStandGate.setState({ pending: { ...p, ip } }); }, [p, ip]);
  if (!p || stale) return null;
  return (
    <ConfirmModal title={t('세우기 전에 확인하세요')} message={standBlockText(p)} confirmLabel={t('그래도 세우기')}
      onConfirm={() => { close(); p.force(); }} onClose={close} />
  );
}

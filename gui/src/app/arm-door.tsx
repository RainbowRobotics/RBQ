import { useEffect } from 'react';
import { useRouter } from 'expo-router';
import { Screen } from '@/components/Screen';
import { HubHeader } from '@/components/hub/HubHeader';
import { ArmDoorPanel } from '@/components/panels/ArmDoorPanel';
import { useTelemetry } from '@/store/telemetry';
import { useHasArm } from '@/store/capability';
import { t } from '@/lib/i18n';
import { goTop } from '@/lib/nav';

export default function ArmDoor() {
  const router = useRouter();
  const hasArm = useHasArm();
  const joints = useTelemetry((s) => s.robot?.joints);

  useEffect(() => {
    if (joints && !hasArm) goTop('/');
  }, [hasArm, joints, router]);

  return (
    <Screen>
      <HubHeader title={t('문 열기')} subtitle="ArmDoor · capability" />
      <ArmDoorPanel onExit={() => router.back()} />
    </Screen>
  );
}

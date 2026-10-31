import { useEffect } from 'react';
import { useRouter } from 'expo-router';
import { Screen } from '@/components/Screen';
import { HubHeader } from '@/components/hub/HubHeader';
import { ArmPanel } from '@/components/panels/ArmPanel';
import { useTelemetry } from '@/store/telemetry';
import { useArmAvailable } from '@/lib/useArmAvailable';
import { t } from '@/lib/i18n';
import { goTop } from '@/lib/nav';

export default function ArmControl() {
  const router = useRouter();
  const hasArm = useArmAvailable();
  const joints = useTelemetry((s) => s.robot?.joints);

  useEffect(() => {
    if (joints && !hasArm) goTop('/');
  }, [hasArm, joints, router]);

  return (
    <Screen>
      <HubHeader title={t('팔 조작')} subtitle="ManiControl · End-Effector" />
      {hasArm && <ArmPanel onExit={() => router.replace('/')} onOpenDoor={() => router.push('/arm-door')} />}
    </Screen>
  );
}

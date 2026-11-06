import { useRouter } from 'expo-router';
import { Screen } from '@/components/Screen';
import { HubHeader } from '@/components/hub/HubHeader';
import { ArmMotionWorkspace } from '@/components/panels/ArmMotionWorkspace';
import { t } from '@/lib/i18n';

export default function ArmMotion() {
  const router = useRouter();
  return <Screen>
    <HubHeader title={t('팔 모션')} subtitle="" />
    <ArmMotionWorkspace onExit={() => router.replace('/')} />
  </Screen>;
}

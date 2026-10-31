import { useEffect, useState } from 'react';
import { useRouter, useLocalSearchParams } from 'expo-router';
import { Screen } from '@/components/Screen';
import { HubHeader } from '@/components/hub/HubHeader';
import { LogPanel } from '@/components/panels/LogPanel';
import { useDevMode, useSettings, useSettingsHydrated } from '@/store/settings';
import { t } from '@/lib/i18n';

export default function Log() {
  const devMode = useDevMode();
  const hydrated = useSettingsHydrated();
  const router = useRouter();

  const params = useLocalSearchParams<{ focusTs?: string; focusMsg?: string }>();
  const [focus, setFocus] = useState<{ ts: string; msg: string } | null>(null);
  useEffect(() => {
    if (!params.focusTs || !params.focusMsg) return;
    router.setParams({ focusTs: undefined, focusMsg: undefined });
    setFocus({ ts: params.focusTs, msg: params.focusMsg });
  }, [params.focusTs, params.focusMsg]); // eslint-disable-line react-hooks/exhaustive-deps

  const [subtitle, setSubtitle] = useState('Real Time');



  return (
    <Screen>
      <HubHeader title={t('로그')} subtitle={subtitle} />
      <LogPanel focus={focus} onSubtitle={setSubtitle} />
    </Screen>
  );
}

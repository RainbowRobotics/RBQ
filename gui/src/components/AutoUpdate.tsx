import { useEffect, useRef, useState } from 'react';
import { AppState, View, Text, Pressable, StyleSheet } from 'react-native';
import { usePathname, useRouter } from 'expo-router';
import { create } from 'zustand';
import { useTheme } from '@/theme';
import { Icon } from '@/components/Icon';
import { Tappable } from '@/components/anim';
import { ConfirmModal } from '@/components/ui/ConfirmModal';
import { useSettings, useSettingsHydrated } from '@/store/settings';
import { useRobot } from '@/store/robot';
import { isDemo } from '@/lib/demoFlag';
import { checkUpdates, offerKey, type UpdateOffer } from '@/lib/autoUpdate';
import { downloadFirmware } from '@/lib/firmwareRelease';
import { currentAppVersion, runAppUpdate } from '@/lib/appSelfUpdate';
import { t } from '@/lib/i18n';
import { robotKinds } from '@/modules/registry';

const INTERVAL_MS = 10 * 60 * 1000;
const MIN_GAP_MS = 60 * 1000;
const CONTROL_ROUTES = ['/', '/arm', '/arm-door'];
const isControlRoute = (path: string) => CONTROL_ROUTES.includes(path) || robotKinds.some((k) => k.route === path);

type Kind = UpdateOffer['kind'];
interface Job { version: string; phase: 'run' | 'done' | 'error'; written: number; total: number; msg?: string }

const useAutoUpdateJobs = create<{ jobs: Partial<Record<Kind, Job>>; put: (k: Kind, j: Job | null) => void }>()((set) => ({
  jobs: {},
  put: (k, j) => set((s) => {
    const jobs = { ...s.jobs };
    if (j) jobs[k] = j; else delete jobs[k];
    return { jobs };
  }),
}));

export const useRobotPackageReady = () => useAutoUpdateJobs((s) => s.jobs.robot?.phase === 'done');

function start(o: UpdateOffer) {
  const { put } = useAutoUpdateJobs.getState();
  const job: Job = { version: o.version, phase: 'run', written: 0, total: 0 };
  const progress = (written: number, total: number) => put(o.kind, { ...job, written, total });
  const fail = (e: Error) => put(o.kind, { ...job, phase: 'error', msg: e.message });
  put(o.kind, job);
  if (o.kind === 'robot') {
    downloadFirmware(o.rel, progress, o.system).done
      .then(() => put('robot', { ...job, phase: 'done' }))
      .catch(fail);
  } else {
    runAppUpdate(o.rel, o.mode, progress)
      .then((msg) => put('app', { ...job, phase: 'done', msg }))
      .catch(fail);
  }
}

export function AutoUpdate() {
  const ready = useSettingsHydrated();
  const on = useSettings((s) => s.autoUpdate);
  const [queue, setQueue] = useState<UpdateOffer[]>([]);
  const asked = useRef(new Set<string>());
  const controlling = isControlRoute(usePathname());

  useEffect(() => {
    if (!ready || !on || isDemo()) return;
    let alive = true;
    let busy = false;
    let last = 0;
    const run = () => {
      if (busy || Date.now() - last < MIN_GAP_MS) return;
      busy = true; last = Date.now();
      const { ip, conn } = useRobot.getState();
      checkUpdates(conn === 'connected' ? ip : null, useSettings.getState().autoUpdateSkip)
        .then(({ offers, robotVersion }) => {
          if (!alive) return;
          const jobs = useAutoUpdateJobs.getState().jobs;
          if (robotVersion && jobs.robot?.phase === 'done' && jobs.robot.version === robotVersion) useAutoUpdateJobs.getState().put('robot', null);
          const fresh = offers.filter((o) => !asked.current.has(offerKey(o)) && jobs[o.kind]?.phase !== 'run');
          fresh.forEach((o) => asked.current.add(offerKey(o)));
          if (fresh.length) setQueue((q) => [...q, ...fresh]);
        })
        .catch(() => { last = 0; })
        .finally(() => { busy = false; });
    };
    run();
    const iv = setInterval(run, INTERVAL_MS);
    const sub = AppState.addEventListener('change', (st) => { if (st === 'active') run(); });
    const unsub = useRobot.subscribe((st, prev) => { if (st.conn === 'connected' && prev.conn !== 'connected') run(); });
    return () => { alive = false; clearInterval(iv); sub.remove(); unsub(); };
  }, [ready, on]);

  useEffect(() => { if (!on) setQueue([]); }, [on]);

  const cur = queue[0];
  if (!cur || controlling) return null;
  const answer = (yes: boolean) => {
    setQueue((q) => q.slice(1));
    if (yes) start(cur);
    else useSettings.getState().skipAutoUpdate(cur.kind, cur.version);
  };
  return cur.kind === 'robot' ? (
    <ConfirmModal key={offerKey(cur)} danger={false}
      title={t('새 로봇 소프트웨어')}
      message={`v${cur.version} — ${t('이 기기로 받아 둘까요? 로봇에 설치는 업데이트 › 소프트웨어 업데이트에서 합니다. 취소하면 이 버전은 다시 묻지 않습니다.')}`}
      confirmLabel={t('다운로드')}
      onConfirm={() => answer(true)} onClose={() => answer(false)} />
  ) : (
    <ConfirmModal key={offerKey(cur)} danger={false}
      title={t('새 앱 버전')}
      message={`v${currentAppVersion()} → v${cur.version} — ${cur.mode === 'exe'
        ? t('받아서 업데이트할까요? 받는 대로 앱이 닫혔다가 새 버전으로 다시 열리고, 그동안 로봇 연결이 끊깁니다. 취소하면 이 버전은 다시 묻지 않습니다.')
        : t('받아서 업데이트할까요? 취소하면 이 버전은 다시 묻지 않습니다.')}`}
      confirmLabel={t('업데이트')}
      onConfirm={() => answer(true)} onClose={() => answer(false)} />
  );
}

export function AutoUpdateBanner() {
  const { c, radius } = useTheme();
  const router = useRouter();
  const jobs = useAutoUpdateJobs((s) => s.jobs);
  const items = (Object.entries(jobs) as [Kind, Job][]).map(([kind, j]) => {
    const what = kind === 'robot' ? t('로봇 소프트웨어') : t('앱');
    const pct = j.total > 0 ? ` ${Math.floor((j.written / j.total) * 100)}%` : '';
    const text = j.phase === 'run' ? `${what} v${j.version} ${t('받는 중')}${pct}`
      : j.phase === 'error' ? `${what} v${j.version} ${t('업데이트 실패')} — ${j.msg ?? ''}`
      : kind === 'robot' ? `${what} v${j.version} ${t('받음 — 업데이트에서 로봇에 설치하세요')}`
      : `${what} v${j.version} — ${j.msg ?? ''}`;
    const tone = j.phase === 'error' ? c.redTx : j.phase === 'done' ? c.greenTx : c.accent2;
    return { kind, j, text, tone };
  });
  if (!items.length) return null;
  return (
    <>
      {items.map(({ kind, j, text, tone }) => (
        <View key={kind} style={[styles.banner, { borderColor: tone, borderRadius: radius.md }]}>
          <Icon name="download" size={14} color={tone} />
          <Text style={[styles.txt, { color: tone }]}>{text}</Text>
          {kind === 'robot' && j.phase === 'done' ? (
            <Tappable onPress={() => router.navigate('/update?tab=sw')} accessibilityLabel={t('소프트웨어 업데이트 열기')}
              style={[styles.open, { borderColor: tone, borderRadius: radius.sm }]}>
              <Text style={{ color: tone, fontSize: 11, fontWeight: '700' }}>{t('열기')}</Text>
            </Tappable>
          ) : null}
          {j.phase !== 'run' ? (
            <Pressable onPress={() => useAutoUpdateJobs.getState().put(kind, null)} hitSlop={8} accessibilityLabel={t('닫기')} style={{ marginLeft: 2 }}>
              <Icon name="x" size={13} color={tone} />
            </Pressable>
          ) : null}
        </View>
      ))}
    </>
  );
}

const styles = StyleSheet.create({
  banner: {
    flexDirection: 'row', alignItems: 'center', gap: 8, maxWidth: '92%',
    paddingHorizontal: 14, paddingVertical: 8, borderWidth: 1,
    backgroundColor: 'rgba(0,0,0,0.55)',
  },
  txt: { fontSize: 11.5, fontWeight: '700', flexShrink: 1 },
  open: { borderWidth: 1, paddingHorizontal: 10, paddingVertical: 3, marginLeft: 4 },
});

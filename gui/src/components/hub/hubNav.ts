import { useRouter, type Href } from 'expo-router';
import { openExternal } from '@/lib/openExternal';
import { useRobotKind } from '@/modules/registry';
import { useRobots, useRobotReady, SIM_SERIAL } from '@/store/robots';
import { useNotMine } from '@/lib/spectating';
import { CHANNEL } from '@/lib/firmwareRelease';
import { t } from '@/lib/i18n';
import { goTop } from '@/lib/nav';
import { usePtzQcDot } from '@/store/ptzQc';
import { useBoardFw } from '@/store/boardFirmware';
import { fwBanners } from '@/lib/boardFirmware';
import { useRobotPackageReady } from '@/components/AutoUpdate';
import type { IconName } from '@/components/Icon';

export const GUIDE_URL = CHANNEL === 'nightly'
  ? 'https://rainbowrobotics.github.io/RBQ/nightly/'
  : 'https://rainbowrobotics.github.io/RBQ/';

export type NavItem = { key: string; icon: IconName; label: string; on: boolean; go: () => void; dot?: boolean; dotHint?: string; path?: string };

export function useHubNav(): { control: NavItem; update: NavItem | null; pills: NavItem[] } {
  const kind = useRobotKind();
  const router = useRouter();
  const ready = useRobotReady();
  const isSim = useRobots((s) => s.currentSerial) === SIM_SERIAL;
  const online = ready || isSim;
  const notMine = useNotMine();
  const ptzDot = usePtzQcDot();
  const pkgReady = useRobotPackageReady();
  const fwNeeded = useBoardFw((s) => { const b = fwBanners(s.status); return b.update.length > 0 || b.mixedMotors; });
  const controlPath = kind?.route ?? '/';
  const control: NavItem = { key: 'control', icon: 'gamepad', label: t('컨트롤'), on: online, go: () => goTop(controlPath as Href), path: controlPath };
  const update: NavItem | null = kind ? null : {
    key: 'update', icon: 'download', label: t('업데이트'), on: !notMine, path: '/update',
    dot: pkgReady || fwNeeded, dotHint: t('설치할 업데이트가 있습니다'),
    go: () => router.push({ pathname: '/update', params: fwNeeded && !pkgReady ? { tab: 'fw' } : {} }),
  };
  const pills: NavItem[] = kind ? [] : [
    { key: 'dashboard', icon: 'gauge', label: t('대시보드'), on: ready && !notMine, go: () => router.push('/dashboard'), path: '/dashboard' },
    { key: 'settings', icon: 'sliders', label: t('설정'), on: true, go: () => router.push('/settings'), path: '/settings' },
    { key: 'maintenance', icon: 'wrench', label: t('정비'), on: true, dot: ptzDot, dotHint: t('새 PTZ 점검 결과가 있습니다'), path: '/maintenance',
      go: () => router.push({ pathname: '/maintenance', params: ptzDot ? { sec: 'ptzqc' } : {} }) },
    { key: 'log', icon: 'log', label: t('로그'), on: true, go: () => router.push('/log'), path: '/log' },
    { key: 'media', icon: 'save', label: t('미디어'), on: ready, go: () => router.push('/media'), path: '/media' },
    { key: 'guide', icon: 'book', label: t('가이드'), on: true, go: () => openExternal(GUIDE_URL) },
  ];
  return { control, update, pills };
}

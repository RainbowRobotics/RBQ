import { Platform } from 'react-native';
import { isDesktop } from '@/lib/desktopBridge';
import { rest } from '@/lib/rest';
import { fetchFirmwareVersions, listCachedTars, cacheId, type FwRelease } from '@/lib/firmwareRelease';
import { appUpdateMode, appInstallStatus, newerAppRelease, type AppRelease, type AppUpdateMode } from '@/lib/appSelfUpdate';

export type UpdateOffer =
  | { kind: 'robot'; version: string; rel: FwRelease }
  | { kind: 'app'; version: string; rel: AppRelease; mode: AppUpdateMode };

export const offerKey = (o: UpdateOffer) => `${o.kind}:${o.version}`;

export interface OfferInput {
  top: FwRelease | null;
  robotSupported: boolean;
  robotPkgOnDevice: boolean;
  robotVersion: string | null;
  appRel: AppRelease | null;
  appMode: AppUpdateMode;
  skip: { robot?: string; app?: string };
}

export function pickOffers(i: OfferInput): UpdateOffer[] {
  const out: UpdateOffer[] = [];
  const top = i.top;
  if (top && i.robotSupported && !i.robotPkgOnDevice
    && i.robotVersion !== top.version && i.skip.robot !== top.version) {
    out.push({ kind: 'robot', version: top.version, rel: top });
  }
  if (i.appRel && (i.appMode === 'apk' || i.appMode === 'appimage' || i.appMode === 'exe') && i.skip.app !== i.appRel.version) {
    out.push({ kind: 'app', version: i.appRel.version, rel: i.appRel, mode: i.appMode });
  }
  return out;
}

const folderName = (r: FwRelease) => (r.channel === 'nightly' ? (r.publishedAt || r.tag) : r.tag);

async function robotPkgOnDevice(rel: FwRelease): Promise<boolean> {
  if (Platform.OS === 'web') {
    const st = await appInstallStatus();
    return !!st?.builds.find((b) => b.name === folderName(rel))?.tar;
  }
  const base = `RBQ-${rel.channel === 'nightly' ? 'nightly' : rel.tag}`;
  const id = cacheId(rel);
  return (await listCachedTars()).some((f) =>
    (f.name === `${base}.tar.gz` || f.name === `${base}.zip`)
    && (f.id ? f.id === id : rel.channel !== 'nightly'));
}

export async function checkUpdates(robotIp: string | null, skip: OfferInput['skip']): Promise<UpdateOffer[]> {
  const rels = await fetchFirmwareVersions();
  const top = rels.find((r) => r.latest) ?? rels[0] ?? null;
  const appMode = await appUpdateMode();
  const robotSupported = Platform.OS !== 'web' || (isDesktop() && appMode === 'appimage');
  const [onDevice, robotVersion] = await Promise.all([
    top && robotSupported ? robotPkgOnDevice(top).catch(() => false) : Promise.resolve(false),
    robotIp ? rest.version(robotIp).then((v) => v.version || null).catch(() => null) : Promise.resolve(null),
  ]);
  return pickOffers({
    top, robotSupported, robotPkgOnDevice: onDevice, robotVersion,
    appRel: newerAppRelease(rels), appMode, skip,
  });
}

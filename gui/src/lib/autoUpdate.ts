import { Platform } from 'react-native';
import { isDesktop } from '@/lib/desktopBridge';
import { rest } from '@/lib/rest';
import { fetchFirmwareVersions, listCachedTars, cacheId, pickPackage, pkgCandidates, type FwRelease } from '@/lib/firmwareRelease';
import { knownSystem } from '@/store/robots';
import { appUpdateMode, appInstallStatus, newerAppRelease, type AppRelease, type AppUpdateMode } from '@/lib/appSelfUpdate';

export type UpdateOffer =
  | { kind: 'robot'; version: string; rel: FwRelease; system?: string }
  | { kind: 'app'; version: string; rel: AppRelease; mode: AppUpdateMode };

export const offerKey = (o: UpdateOffer) => `${o.kind}:${o.version}`;

export interface OfferInput {
  top: FwRelease | null;
  robotSupported: boolean;
  robotPkgOnDevice: boolean;
  robotPkgPublished?: boolean;
  robotVersion: string | null;
  robotSystem?: string;
  appRel: AppRelease | null;
  appMode: AppUpdateMode;
  skip: { robot?: string; app?: string };
}

export function pickOffers(i: OfferInput): UpdateOffer[] {
  const out: UpdateOffer[] = [];
  const top = i.top;
  if (top && i.robotSupported && !i.robotPkgOnDevice && i.robotPkgPublished !== false
    && i.robotVersion !== top.version && i.skip.robot !== top.version) {
    out.push({ kind: 'robot', version: top.version, rel: top, system: i.robotSystem });
  }
  if (i.appRel && (i.appMode === 'apk' || i.appMode === 'appimage' || i.appMode === 'exe') && i.skip.app !== i.appRel.version) {
    out.push({ kind: 'app', version: i.appRel.version, rel: i.appRel, mode: i.appMode });
  }
  return out;
}

const folderName = (r: FwRelease) => (r.channel === 'nightly' ? (r.publishedAt || r.tag) : r.tag);

async function robotPkgOnDevice(rel: FwRelease, system: string | undefined): Promise<boolean> {
  if (Platform.OS === 'web') {
    const st = await appInstallStatus();
    const b = st?.builds.find((x) => x.name === folderName(rel));
    return !!(b?.pkgs ? pickPackage(b.pkgs, rel, system) : b?.tar);
  }
  const id = cacheId(rel);
  return !!pickPackage((await listCachedTars()).filter((f) => (f.id ? f.id === id : rel.channel !== 'nightly')),
    rel, system);
}

export async function checkUpdates(robotIp: string | null, skip: OfferInput['skip']): Promise<{ offers: UpdateOffer[]; robotVersion: string | null }> {
  const rels = await fetchFirmwareVersions();
  const top = rels.find((r) => r.latest) ?? rels[0] ?? null;
  const appMode = await appUpdateMode();
  const robotSupported = Platform.OS !== 'web' || (isDesktop() && appMode === 'appimage');
  const info = robotIp ? await rest.version(robotIp).catch(() => null) : null;
  const robotVersion = info?.version || null;
  const robotSystem = info?.system || knownSystem('motion', robotIp ?? undefined);
  const onDevice = top && robotSupported ? await robotPkgOnDevice(top, robotSystem).catch(() => false) : false;
  const robotPkgPublished = !top || pkgCandidates(top, robotSystem).length > 0;
  const offers = pickOffers({
    top, robotSupported, robotPkgOnDevice: onDevice, robotPkgPublished, robotVersion, robotSystem,
    appRel: newerAppRelease(rels), appMode, skip,
  });
  return { offers, robotVersion };
}

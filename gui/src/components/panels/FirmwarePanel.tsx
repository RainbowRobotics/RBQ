import { useEffect, useMemo, useRef, useState } from 'react';
import { View, Text, StyleSheet, ScrollView } from 'react-native';
import Constants from 'expo-constants';
import { pickDocument } from '@/lib/pickDocument';
import { cacheDirectory } from 'expo-file-system/legacy';
import { useTheme } from '@/theme';
import { Icon } from '@/components/Icon';
import { Tappable } from '@/components/anim';
import { Modal } from '@/components/ui/overlays';
import { ConfirmModal } from '@/components/ui/ConfirmModal';
import { useRobot } from '@/store/robot';
import { useSettings } from '@/store/settings';
import { useKnownSystem, useRobots } from '@/store/robots';
import { actions, rest } from '@/lib/rest';
import { otaTransfer, otaProbe, otaAvailable, visionOtaIp, AUTO_DEPLOY_RE, OTA_PORT, type OtaProgress, type OtaHandle } from '@/lib/ota';
import { fetchFirmwareVersions, downloadFirmware, fetchReleaseNotes, listCachedTars, removeCachedTar, cacheId,
  pickPackage, CHANNEL, LEGACY_SYSTEM, type FwRelease, type FwDownload, type FwDownloaded, type CachedTar, type FwFile, type FwPart } from '@/lib/firmwareRelease';
import { appUpdateMode, newerAppRelease, runAppUpdate, appInstallStatus, appSelectBuild, appFetchOnly, appRemoveBuild,
  currentAppVersion,
  type AppUpdateMode, type AppInstallStatus, type AppBuild } from '@/lib/appSelfUpdate';
import { t } from '@/lib/i18n';
import { useLang } from '@/store/lang';
import type { VersionInfo } from '@/types/robot';
import { notify } from '@/components/DoneToast';

const fmtMB = (b: number) => `${(b / 1048576).toFixed(1)} MB`;
const partLabel = (p?: FwPart) => (p ? `${t('파일')} ${p.i}/${p.n} · ` : '');
const aborted = (e: Error) => e.message === 'cancelled' || e.name === 'AbortError';
interface VRow {
  key: string;
  version: string;
  latest: boolean;
  rel?: FwRelease;
  build?: AppBuild;
  cached?: CachedTar;
}
const rowKey = (r: FwRelease) => (r.channel === 'nightly' ? (r.publishedAt || r.tag) : r.tag);
const channelBadge = (ch: string | null) => (ch === 'nightly' ? 'NIGHTLY' : ch ? null : 'LOCAL');
const fwCardHd = (color: string) => ({ color, fontSize: 11, fontWeight: '700' as const, marginBottom: 5 });

function KV({ k, v, vColor, mono = true }: { k: string; v: string; vColor?: string; mono?: boolean }) {
  const { c, fonts } = useTheme();
  return (
    <View style={{ flexDirection: 'row', justifyContent: 'space-between', paddingVertical: 4 }}>
      <Text style={{ color: c.muted, fontSize: 12 }}>{k}</Text>
      <Text style={{ color: vColor ?? c.text, fontSize: 12, fontFamily: mono ? fonts.mono : undefined }}>{v}</Text>
    </View>
  );
}

export function FirmwarePanel() {
  const { c, fonts, radius } = useTheme();
  const ip = useRobot((s) => s.ip);
  const visionIp = useRobot((s) => s.visionIp);
  const [tgt, setTgt] = useState<'motion' | 'vision'>('motion');
  const destIp = tgt === 'vision' ? visionOtaIp(visionIp) : ip;
  const destLabel = tgt === 'vision' ? `Vision (${destIp})` : `Motion (${destIp})`;
  const destSystem = useKnownSystem(tgt, destIp);
  useEffect(() => {
    let alive = true;
    otaProbe(destIp).then((r) => {
      if (!alive || !r || useRobot.getState().conn !== 'connected') return;
      const sys = r.system || LEGACY_SYSTEM;
      useSettings.getState().rememberOtaSystem(destIp, sys);
      const sn = useRobots.getState().currentSerial;
      if (sn) useRobots.getState().setSystem(sn, tgt, sys);
    });
    return () => { alive = false; };
  }, [destIp, tgt]);
  const [ver, setVer] = useState<VersionInfo | null>(null);
  const [file, setFile] = useState<FwDownloaded | null>(null);
  const [prog, setProg] = useState<OtaProgress | null>(null);
  const [err, setErr] = useState<string | null>(null);
  const [doneMsg, setDoneMsg] = useState(false);
  const [confirm, setConfirm] = useState(false);
  const [appConfirm, setAppConfirm] = useState(false);
  const [askReboot, setAskReboot] = useState(false);
  const [rels, setRels] = useState<FwRelease[] | null>(null);
  const [relErr, setRelErr] = useState<string | null>(null);
  const [relLoading, setRelLoading] = useState(false);
  const [dlTag, setDlTag] = useState<string | null>(null);
  const [dlProg, setDlProg] = useState<{ written: number; total: number; part?: FwPart } | null>(null);
  const dl = useRef<FwDownload | null>(null);
  const handle = useRef<OtaHandle | null>(null);
  const scroll = useRef<ScrollView>(null);
  const [appMode, setAppMode] = useState<AppUpdateMode>('none');
  const [appBusy, setAppBusy] = useState(false);
  const [appProg, setAppProg] = useState<{ written: number; total: number } | null>(null);
  const [appMsg, setAppMsg] = useState<string | null>(null);
  const [appErr, setAppErr] = useState<string | null>(null);
  useEffect(() => { appUpdateMode().then(setAppMode).catch(() => {}); }, []);
  const [inst, setInst] = useState<AppInstallStatus | null>(null);
  const [instBusy, setInstBusy] = useState<string | null>(null);
  const [instMsg, setInstMsg] = useState<string | null>(null);
  const [instErr, setInstErr] = useState<string | null>(null);
  const appAbort = useRef<AbortController | null>(null);
  const [cached, setCached] = useState<CachedTar[]>([]);
  const loadInst = () => {
    appInstallStatus().then(setInst).catch(() => {});
    listCachedTars().then(setCached).catch(() => {});
  };
  useEffect(loadInst, []);
  const selectBuild = (name: string) => {
    if (instBusy) return;
    setInstBusy(name); setInstErr(null); setInstMsg(null);
    appSelectBuild(name)
      .then(() => { setInstMsg(`${t('재시작하면')} ${name} ${t('빌드로 시작합니다')}`); loadInst(); })
      .catch((e: Error) => setInstErr(e.message))
      .finally(() => setInstBusy(null));
  };
  const vrows: VRow[] = useMemo(() => {
    const m = new Map<string, VRow>();
    for (const r of rels ?? []) m.set(rowKey(r), { key: rowKey(r), version: r.version, latest: r.latest, rel: r });
    for (const b0 of inst?.builds ?? []) {
      const cur = m.get(b0.name);
      const b = b0.pkgs ? { ...b0, tar: pickPackage(b0.pkgs, cur?.rel, destSystem) ?? null } : b0;
      if (cur) cur.build = b;
      else m.set(b.name, { key: b.name, version: b.version || b.name, latest: false, build: b });
    }
    for (const row of m.values()) {
      if (!row.rel) continue;
      const want_id = cacheId(row.rel);
      const hit = pickPackage(cached.filter((f) => (f.id ? f.id === want_id : row.rel!.channel !== 'nightly')),
        row.rel, destSystem);
      if (hit) row.cached = hit;
    }
    const cmp = (a: string, b: string) => {
      const pa = a.match(/\d+|\D+/g) ?? [];
      const pb = b.match(/\d+|\D+/g) ?? [];
      for (let i = 0; i < Math.max(pa.length, pb.length); i++) {
        const x = pa[i] ?? '', y = pb[i] ?? '';
        if (x === y) continue;
        const nx = Number(x), ny = Number(y);
        if (!Number.isNaN(nx) && !Number.isNaN(ny) && x !== '' && y !== '') return nx - ny;
        return x < y ? -1 : 1;
      }
      return 0;
    };
    return [...m.values()].sort((a, b) => cmp(b.key, a.key));
  }, [rels, inst, ver, cached, destSystem]);
  const [sel, setSel] = useState<string | null>(null);
  const [rowBusy, setRowBusy] = useState<string | null>(null);
  const [rowDl, setRowDl] = useState(false);
  const [rowProg, setRowProg] = useState<{ written: number; total: number; part?: FwPart } | null>(null);
  const downloadRow = (row: VRow) => {
    if (!row.rel || rowBusy) return;
    const rel = row.rel;
    setRowBusy(row.key); setRowDl(true); setErr(null); setRowProg({ written: 0, total: 0 });
    appAbort.current = new AbortController();
    (appMode === 'appimage' && !row.build?.app
      ? appFetchOnly({ tag: rel.tag, version: rel.version, channel: rel.channel }, appAbort.current.signal)
      : Promise.resolve())
      .catch((e: Error) => { if (!aborted(e)) setErr(e.message); })
      .then(() => new Promise<void>((ok, no) => {
        if (appAbort.current?.signal.aborted) { no(new Error('cancelled')); return; }
        if (row.build?.tar || row.cached) { ok(); return; }
        dl.current = downloadFirmware(rel, (written, total, part) => setRowProg({ written, total, part }), destSystem);
        dl.current.done.then(() => ok()).catch(no).finally(() => { dl.current = null; });
      }))
      .then(() => { loadInst(); setSel(row.key); })
      .catch((e: Error) => { if (!aborted(e)) setErr(e.message); })
      .finally(() => { dl.current = null; appAbort.current = null; setRowBusy(null); setRowDl(false); setRowProg(null); });
  };
  const installRow = (row: VRow) => {
    if (appMode === 'appimage') { if (row.build) selectBuild(row.build.name); return; }
    const rel = row.rel;
    if (!rel || rowBusy) return;
    setRowBusy(row.key); setInstErr(null); setInstMsg(null); setRowProg({ written: 0, total: 0 });
    runAppUpdate({ tag: rel.tag, version: rel.version, channel: rel.channel }, appMode,
                 (written, total) => setRowProg({ written, total }))
      .then(setInstMsg)
      .catch((e: Error) => setInstErr(e.message))
      .finally(() => { setRowBusy(null); setRowProg(null); });
  };
  const lang = useLang((st) => st.lang);
  const [notes, setNotes] = useState<{ ko: string; en: string } | null>(null);
  const [notesFor, setNotesFor] = useState('');
  const notesTag = (() => {
    const row = vrows.find((r) => r.key === sel)
      ?? (ver?.version ? vrows.find((r) => r.version === ver.version) : undefined)
      ?? vrows.find((r) => r.latest);
    return row ? (row.rel?.tag ?? row.key) : '';
  })();
  useEffect(() => {
    if (!notesTag) { setNotes(null); setNotesFor(''); return; }
    let alive = true;
    fetchReleaseNotes(notesTag)
      .then((n) => { if (alive) { setNotes(n); setNotesFor(n ? notesTag : ''); } })
      .catch(() => {});
    return () => { alive = false; };
  }, [notesTag]);
  const removeRow = (row: VRow) => {
    if (rowBusy) return;
    setRowBusy(row.key); setErr(null);
    const job = appMode === 'appimage' ? appRemoveBuild(row.key)
              : row.cached ? removeCachedTar(row.cached.name) : Promise.resolve();
    job.then(() => { loadInst(); if (sel === row.key) setSel(null); })
      .catch((e: Error) => setErr(e.message))
      .finally(() => setRowBusy(null));
  };
  const transferRow = (row: VRow) => {
    if (handle.current) return;
    pickStoredTar(row);
    setConfirm(true);
  };
  const pickStoredTar = (row: VRow) => {
    if (row.build?.tar) {
      const q = `channel=${encodeURIComponent(CHANNEL ?? 'release')}&tag=${encodeURIComponent(row.rel?.tag ?? row.key)}`
        + `&folder=${encodeURIComponent(row.key)}&name=${encodeURIComponent(row.build.tar.name)}`;
      setFile({ uri: `/fw-file?${q}`, name: row.build.tar.name, size: row.build.tar.size });
      return;
    }
    if (row.cached) setFile({ uri: `${cacheDirectory}${row.cached.name}`, name: row.cached.name, size: row.cached.size });
  };
  const startAppInstall = () => {
    if (appBusy) return;
    setAppBusy(true); setAppErr(null); setAppMsg(null); setAppProg(null);
    (rels ? Promise.resolve(rels) : fetchFirmwareVersions())
      .then((list) => {
        const top = list.find((r) => r.latest) ?? list[0];
        if (!top) throw new Error(t('이 채널에 발행된 빌드가 없습니다'));
        return runAppUpdate({ tag: top.tag, version: top.version, channel: top.channel }, appMode,
                            (written, total) => setAppProg({ written, total }));
      })
      .then((msg) => { setAppMsg(msg); loadInst(); })
      .catch((e: Error) => setAppErr(e.message))
      .finally(() => { setAppBusy(false); setAppProg(null); });
  };
  const appRel = rels && appMode !== 'none' ? newerAppRelease(rels) : null;
  const appRelDate = appRel ? (rels ?? []).find((r) => r.tag === appRel.tag)?.publishedAt : '';
  const startAppUpdate = () => {
    if (!appRel || appBusy) return;
    setAppBusy(true); setAppErr(null); setAppMsg(null); setAppProg(null);
    runAppUpdate(appRel, appMode, (written, total) => setAppProg({ written, total }))
      .then(setAppMsg)
      .catch((e: Error) => setAppErr(e.message))
      .finally(() => { setAppBusy(false); setAppProg(null); });
  };
  useEffect(() => {
    rest.version(ip).then(setVer).catch(() => setVer(null));
  }, [ip]);
  useEffect(() => { if (CHANNEL === 'nightly') loadReleases(); }, []);   // eslint-disable-line react-hooks/exhaustive-deps
  useEffect(() => () => { handle.current?.cancel(); dl.current?.cancel(); appAbort.current?.abort(); }, []);

  const showList = (appMode === 'appimage' && !!inst?.appimage) || (appMode === 'apk' && !!CHANNEL);

  const busy = !!prog && prog.phase !== 'complete';
  const autoDeploy = file ? AUTO_DEPLOY_RE.test(file.name) : false;

  const list = rels;
  const loadReleases = () => {
    setRelLoading(true); setRelErr(null);
    fetchFirmwareVersions()
      .then(setRels)
      .catch((e: Error) => setRelErr(e.message))
      .finally(() => setRelLoading(false));
  };

  const startDownload = (rel: FwRelease) => {
    if (dl.current) return;
    setErr(null); setDoneMsg(false); setDlTag(rel.tag); setDlProg({ written: 0, total: 0 });
    dl.current = downloadFirmware(rel, (written, total, part) => setDlProg({ written, total, part }), destSystem);
    dl.current.done
      .then((f) => setFile(f))
      .catch((e: Error) => { if (!aborted(e)) setErr(e.message); })
      .finally(() => { dl.current = null; setDlTag(null); setDlProg(null); });
  };

  const pick = async () => {
    const r = await pickDocument({ copyToCacheDirectory: true });
    const a = r.assets?.[0];
    if (r.canceled || !a) return;
    if (a.size == null) { setErr(t('파일 크기를 확인할 수 없습니다 — 다른 위치에서 선택해 보세요')); return; }
    setErr(null); setDoneMsg(false); setProg(null);
    setFile({ uri: a.uri, name: a.name, size: a.size });
  };

  const start = () => {
    if (!file || handle.current) return;
    setConfirm(false); setErr(null); setDoneMsg(false);
    setTimeout(() => scroll.current?.scrollToEnd({ animated: true }), 80);
    handle.current = otaTransfer(destIp, file, setProg, file.alts);
    handle.current.done
      .then(() => { setDoneMsg(true); if (AUTO_DEPLOY_RE.test(file.name)) setAskReboot(true); })
      .catch((e: Error) => {
        setProg(null);
        if (e.message !== 'cancelled') setErr(e.message);
      })
      .finally(() => { handle.current = null; });
  };

  return (
    <View style={{ flex: 1 }}>
      <ScrollView ref={scroll} showsVerticalScrollIndicator={false}>
      <Text style={{ color: c.text, fontSize: 17, fontWeight: '700', marginBottom: 3 }}>
        {t('⬇ 로봇 소프트웨어')} <Text style={{ color: c.dim, fontSize: 11 }}>Software Update</Text>
      </Text>
      <Text style={{ color: c.dim, fontSize: 12, marginBottom: 10, lineHeight: 16 }}>
        {t('버전 확인과 로봇 소프트웨어 업데이트')}
      </Text>
      <View style={{ flexDirection: 'row', alignItems: 'center', gap: 8, marginBottom: 10, flexWrap: 'wrap' }}>
        <Text style={{ color: c.muted, fontSize: 12 }}>GUI Version :</Text>
        <Text style={{ color: c.text, fontSize: 12, fontFamily: fonts.mono }}>
          v{Constants.expoConfig?.version ?? '—'}
        </Text>
        {channelBadge(CHANNEL) ? (
          <Text style={{ color: c.accent2, fontSize: 9.5, fontWeight: '700', borderWidth: 1, borderColor: c.accent2, borderRadius: 5, paddingHorizontal: 5, paddingVertical: 1 }}>{channelBadge(CHANNEL)}</Text>
        ) : null}
        {ver && Constants.expoConfig?.version ? (ver.version === Constants.expoConfig.version ? (
          <Text style={{ color: c.greenTx, fontSize: 9.5, fontWeight: '700', borderWidth: 1, borderColor: 'rgba(63,185,80,0.5)', borderRadius: 5, paddingHorizontal: 5, paddingVertical: 1 }}>✓ {t('로봇과 일치')}</Text>
        ) : (
          <Text style={{ color: c.amberTx, fontSize: 9.5, fontWeight: '700', borderWidth: 1, borderColor: 'rgba(210,153,34,0.5)', borderRadius: 5, paddingHorizontal: 5, paddingVertical: 1 }}>{t('로봇과 다름')} · {t('로봇')} v{ver.version}</Text>
        )) : null}
        {appRel && !appBusy && !appMsg ? (
          <Tappable onPress={appMode === 'exe' ? () => setAppConfirm(true) : startAppUpdate}
            style={[styles.action, { backgroundColor: c.accent, borderColor: c.accent, borderRadius: radius.md, height: 24 }]}>
            <Icon name={appMode === 'store' ? 'external' : 'download'} size={11} color={c.onAccent} />
            <Text style={{ color: c.onAccent, fontSize: 11, fontWeight: '600' }}>
              {appMode === 'store' ? t('스토어에서 업데이트')
                : `${appRel.tag}${appRelDate ? ` · ${appRelDate}` : ''} ${t('업데이트')}`}
            </Text>
          </Tappable>
        ) : null}
        {appBusy ? (
          <Text style={{ color: c.accent2, fontSize: 11, fontFamily: fonts.mono }}>
            {appProg && appProg.total > 0
              ? `${fmtMB(appProg.written)} / ${fmtMB(appProg.total)}`
              : t('다운로드·적용 중…')}
          </Text>
        ) : null}
        {appMsg ? <Text style={{ color: c.greenTx, fontSize: 11.5 }}>✓ {appMsg}</Text> : null}
        {appErr ? <Text style={{ color: c.redTx, fontSize: 11.5 }}>{t('오류:')} {appErr}</Text> : null}
        {rels && appMode !== 'none' && !appRel && !appBusy && !appMsg && !appErr ? (
          <Text style={{ color: c.dim, fontSize: 11.5 }}>✓ {t('최신 버전입니다')}</Text>
        ) : null}
      </View>

      <View style={{ flexDirection: 'row', gap: 10 }}>
        <View style={[styles.fwCard, { borderColor: c.line, borderRadius: radius.md, backgroundColor: c.bg }]}>
          <Text style={fwCardHd(c.dim)}>{t('현재 로봇')} · Motion ({ip})</Text>
          <KV k="Version" v={ver?.version ?? '—'} vColor={c.greenTx} />
          <KV k="Branch" v={ver?.branch ?? '—'} />
          <KV k="Commit" v={ver?.commit_date ?? '—'} />
          <KV k="Build" v={ver ? ver.build_date.split(' ')[0] : '—'} />
        </View>
        <View style={[styles.fwCard, { borderColor: c.line, borderRadius: radius.md, backgroundColor: c.bg }]}>
          <Text style={fwCardHd(c.dim)}>{t('업데이트 파일')}</Text>
          <KV k={t('선택됨')} v={file?.name ?? '—'} />
          {file ? <KV k={t('버전')} v={/^(?:RBQ-(.+)\.(?:tar\.gz|zip)|core-(v.+)-u\d+-\w+\.rbq)$/.exec(file.name)?.slice(1).find(Boolean) ?? '—'} vColor={c.greenTx} /> : null}
          <KV k={t('크기')} v={file ? fmtMB(file.size) : '—'} />
          <View style={{ flexDirection: 'row', alignItems: 'center', justifyContent: 'space-between', paddingVertical: 4, gap: 10 }}>
            <Text style={{ color: c.muted, fontSize: 12 }}>{t('대상')}</Text>
            <View style={{ flexDirection: 'row', alignItems: 'center', gap: 8, flexShrink: 1 }}>
              <Text numberOfLines={1} style={{ color: c.text, fontSize: 12, fontFamily: fonts.mono }}>{destLabel}</Text>
            {(['motion', 'vision'] as const).map((k) => (
              <Tappable key={k} onPress={() => setTgt(k)} disabled={busy}
                style={{ paddingVertical: 5, paddingHorizontal: 10, borderRadius: radius.sm, borderWidth: 1,
                  borderColor: tgt === k ? c.accent : c.line, backgroundColor: c.elev, opacity: busy ? 0.5 : 1 }}>
                <Text style={{ color: tgt === k ? c.text : c.muted, fontSize: 11.5, fontWeight: '700' }}>
                  {k === 'motion' ? 'Motion' : 'Vision'}
                </Text>
              </Tappable>
            ))}
            </View>
          </View>
          <KV k={t('시스템')} v={destSystem === LEGACY_SYSTEM ? t('구형 (RBQ-*)') : destSystem ?? '—'} />
          {tgt === 'vision' && !(visionIp ?? '').trim() ? (
            <Text style={{ color: c.amber, fontSize: 10, marginTop: 4 }}>
              {t('비전 IP 가 설정되어 있지 않아 기본값으로 보냅니다 — 설정 → 연결에서 지정할 수 있습니다')}
            </Text>
          ) : null}
          {appMode === 'appimage' ? null : (
            <Text style={{ color: c.dim, fontSize: 10, marginTop: 4 }}>{t('파일 선택 = 시스템 피커 (USB OTG 포함)')}</Text>
          )}
        </View>
      </View>

      {showList ? (
        <View style={[styles.fwBlock, { borderColor: c.line, borderRadius: radius.md, backgroundColor: c.bg, marginTop: 10 }]}>
          <View style={{ flexDirection: 'row', alignItems: 'center', marginBottom: 5 }}>
            <Text style={[fwCardHd(c.dim), { marginBottom: 0, flex: 1 }]}>
              {!CHANNEL ? t('보관 중인 빌드')
                : CHANNEL === 'nightly' ? `${t('나이틀리 빌드')} · ${t('최근 10일치 보관')}`
                : `${t('온라인 업데이트')} · ${t('인터넷 필요')}`}
            </Text>
            {!CHANNEL || CHANNEL === 'nightly' ? null : (
              <Tappable onPress={loadReleases} disabled={relLoading || !!rowBusy}
                style={[styles.action, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.md, height: 26, opacity: rowBusy ? 0.4 : 1 }]}>
                <Text style={{ color: c.text, fontSize: 11.5, fontWeight: '600' }}>{relLoading ? t('조회 중…') : list ? t('새로 고침') : t('버전 확인')}</Text>
              </Tappable>
            )}
          </View>
          {appMode === 'appimage' && inst && !inst.managed ? (
            <View style={{ flexDirection: 'row', alignItems: 'center', gap: 8, flexWrap: 'wrap', marginBottom: 4 }}>
              <Text style={{ color: c.dim, fontSize: 11.5, flex: 1, lineHeight: 15 }}>
                {t('받아 둔 파일에서 실행 중입니다. 설치하면 문서 폴더로 들어가고 바탕화면·메뉴에 등록됩니다.')}
              </Text>
              <Tappable onPress={startAppInstall} disabled={appBusy}
                style={[styles.action, { backgroundColor: c.accent, borderColor: c.accent, borderRadius: radius.md, height: 26, opacity: appBusy ? 0.4 : 1 }]}>
                <Icon name="download" size={11} color={c.onAccent} />
                <Text style={{ color: c.onAccent, fontSize: 11.5, fontWeight: '600' }}>{t('온라인 설치')}</Text>
              </Tappable>
            </View>
          ) : null}
          {!CHANNEL ? (
            <Text style={{ color: c.dim, fontSize: 11.5, lineHeight: 15, marginBottom: 4 }}>
              {t('이 빌드는 배포 채널이 없어 새 버전을 확인하지 않습니다 — 받아 둔 것만 보입니다.')}
            </Text>
          ) : null}
          {relErr ? <Text style={{ color: c.redTx, fontSize: 11.5 }}>{t('오류:')} {relErr}</Text> : null}
          {vrows.map((row) => {
            const b = row.build;
            const tar = b?.tar ?? row.cached ?? null;
            const showApp = appMode === 'appimage';
            const hasApp = showApp && !!b?.app;
            const hasTar = !!tar;
            const current = b ? b.current : row.version === currentAppVersion();
            const canInstall = !current && (showApp ? hasApp : !!row.rel);
            if (b?.current && !row.rel && !hasTar) return null;
            const checked = sel === row.key;
            const busy = rowBusy === row.key;
            const busyTx = !!prog && prog.phase !== 'complete';
            return (
              <View key={row.key} style={{ flexDirection: 'row', alignItems: 'center', gap: 6, paddingVertical: 5 }}>
                <Tappable onPress={() => setSel(checked ? null : row.key)}
                  style={{ width: 18, height: 18, borderRadius: 4, borderWidth: 1, alignItems: 'center', justifyContent: 'center',
                           borderColor: checked ? c.accent : c.line, backgroundColor: checked ? c.accent : 'transparent' }}>
                  {checked ? <Text style={{ color: c.onAccent, fontSize: 12, fontWeight: '700' }}>✓</Text> : null}
                </Tappable>
                <Text numberOfLines={1} style={{ color: c.text, fontSize: 12, fontFamily: fonts.mono, minWidth: 88, flexShrink: 1 }}>{row.key}</Text>
                {row.version && row.version !== row.key ? (
                  <Text numberOfLines={1} style={{ color: c.dim, fontSize: 10.5, fontFamily: fonts.mono, flexShrink: 1 }}>{row.version}</Text>
                ) : null}
                {row.latest ? (
                  <Text style={{ color: c.accent2, fontSize: 9.5, fontWeight: '700', borderWidth: 1, borderColor: c.line, borderRadius: 5, paddingHorizontal: 5, paddingVertical: 1 }}>{t('최신')}</Text>
                ) : null}
                {current ? (
                  <Text style={{ color: c.greenTx, fontSize: 9.5, fontWeight: '700', borderWidth: 1, borderColor: 'rgba(63,185,80,0.5)', borderRadius: 5, paddingHorizontal: 5, paddingVertical: 1 }}>{t('사용 중')}</Text>
                ) : null}
                <View style={{ flex: 1 }} />
                {showApp ? (
                  <Text style={{ color: hasApp ? c.greenTx : c.dim, fontSize: 10, fontFamily: fonts.mono }}>
                    {t('앱')} {hasApp ? '✓' : '—'}
                  </Text>
                ) : null}
                <Text style={{ color: hasTar ? c.greenTx : c.dim, fontSize: 10, fontFamily: fonts.mono }}>
                  {t('패키지')} {hasTar ? fmtMB(tar!.size) : '—'}
                </Text>
                {busy ? (
                  <>
                    <Text style={{ color: c.accent2, fontSize: 11, fontFamily: fonts.mono }}>
                      {partLabel(rowProg?.part)}{rowProg && rowProg.total > 0 ? `${fmtMB(rowProg.written)} / ${fmtMB(rowProg.total)}` : t('받는 중…')}
                    </Text>
                    {rowDl ? (
                    <Tappable onPress={() => { appAbort.current?.abort(); dl.current?.cancel(); }}
                      style={[styles.action, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.md, height: 24 }]}>
                      <Text style={{ color: c.text, fontSize: 11, fontWeight: '600' }}>{t('취소')}</Text>
                    </Tappable>
                    ) : null}
                  </>
                ) : (
                  <>
                    {row.rel && !(showApp ? hasApp && hasTar : hasTar) ? (
                      <Tappable onPress={() => downloadRow(row)} disabled={!!rowBusy || busyTx}
                        style={[styles.action, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.md, height: 24, opacity: rowBusy || busyTx ? 0.4 : 1 }]}>
                        <Icon name="download" size={10} color={c.muted} />
                        <Text style={{ color: c.text, fontSize: 11, fontWeight: '600' }}>{t('다운로드')}</Text>
                      </Tappable>
                    ) : null}
                    {(showApp
                      ? !!b && !current && (inst?.builds.length ?? 0) > 1
                      : hasTar) ? (
                      <Tappable onPress={() => removeRow(row)} disabled={!!rowBusy || busyTx}
                        style={[styles.action, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.md, height: 24, opacity: rowBusy || busyTx ? 0.4 : 1 }]}>
                        <Text style={{ color: c.redTx, fontSize: 11, fontWeight: '600' }}>{t('제거')}</Text>
                      </Tappable>
                    ) : null}
                    {checked && canInstall ? (
                      <Tappable onPress={() => installRow(row)} disabled={!!instBusy || busyTx}
                        style={[styles.action, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.md, height: 24, opacity: instBusy || busyTx ? 0.4 : 1 }]}>
                        <Text style={{ color: c.text, fontSize: 11, fontWeight: '600' }}>{instBusy === b?.name ? t('전환 중…') : t('앱 설치')}</Text>
                      </Tappable>
                    ) : null}
                    {otaAvailable && checked && hasTar ? (
                      <Tappable onPress={() => transferRow(row)} disabled={busy}
                        style={[styles.action, { backgroundColor: c.accent, borderColor: c.accent, borderRadius: radius.md, height: 24, opacity: busy ? 0.4 : 1 }]}>
                        <Text style={{ color: c.onAccent, fontSize: 11, fontWeight: '600' }}>{t('로봇 소프트웨어 전송')}</Text>
                      </Tappable>
                    ) : null}
                  </>
                )}
              </View>
            );
          })}
          {instMsg ? <Text style={{ color: c.greenTx, fontSize: 11.5, marginTop: 3 }}>✓ {instMsg}</Text> : null}
          {instErr ? <Text style={{ color: c.redTx, fontSize: 11.5, marginTop: 3 }}>{t('오류:')} {instErr}</Text> : null}
          {notes ? (
            <View style={{ borderTopWidth: 1, borderTopColor: c.line, marginTop: 8, paddingTop: 8 }}>
              <View style={{ flexDirection: 'row', alignItems: 'center', gap: 6, marginBottom: 6 }}>
                <Text style={{ color: c.muted, fontSize: 12, fontWeight: '700' }}>{t('출시 노트')}</Text>
                <Text style={{ color: c.accent2, fontSize: 11, fontFamily: fonts.mono, fontWeight: '700',
                               borderWidth: 1, borderColor: c.line, borderRadius: 5, paddingHorizontal: 6, paddingVertical: 1 }}>
                  {notesFor}
                </Text>
              </View>
              <Text style={{ color: c.dim, fontSize: 12, lineHeight: 21 }}>
                {(lang === 'en' ? notes.en : notes.ko) || notes.ko || notes.en}
              </Text>
            </View>
          ) : null}
          <Text style={{ color: c.dim, fontSize: 10, marginTop: 4, lineHeight: 13 }}>
            {t('다운로드는 앱과 로봇 패키지를 함께 받습니다 — 로봇 망으로 바꾼 뒤 전송해도 됩니다.')}
          </Text>
        </View>
      ) : null}

      {showList ? null : !CHANNEL ? (
        <View style={[styles.fwBlock, { borderColor: c.line, borderRadius: radius.md, backgroundColor: c.bg, marginTop: 10 }]}>
          <Text style={fwCardHd(c.dim)}>{t('온라인 업데이트')}</Text>
          <Text style={{ color: c.dim, fontSize: 11.5, lineHeight: 15 }}>
            {t('이 빌드는 배포 채널이 없어 업데이트를 확인하지 않습니다. 정식 또는 나이틀리 빌드를 설치하세요.')}
          </Text>
        </View>
      ) : (
      <View style={[styles.fwBlock, { borderColor: c.line, borderRadius: radius.md, backgroundColor: c.bg, marginTop: 10 }]}>
        <View style={{ flexDirection: 'row', alignItems: 'center', marginBottom: 5 }}>
          <Text style={[fwCardHd(c.dim), { marginBottom: 0, flex: 1 }]}>{t('온라인 업데이트')} {'·'} {t('인터넷 필요')}</Text>
          <Tappable onPress={loadReleases} disabled={relLoading || !!dlTag}
            style={[styles.action, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.md, height: 26, opacity: dlTag ? 0.4 : 1 }]}>
            <Text style={{ color: c.text, fontSize: 11.5, fontWeight: '600' }}>{relLoading ? t('조회 중…') : list ? t('새로 고침') : t('버전 확인')}</Text>
          </Tappable>
        </View>
        {relErr ? <Text style={{ color: c.redTx, fontSize: 11.5 }}>{t('오류:')} {relErr}</Text> : null}
        {list?.slice(0, 5).map((r) => {
          const isCur = ver?.version === r.version;
          const downloading = dlTag === r.tag;
          return (
            <View key={r.tag} style={{ flexDirection: 'row', alignItems: 'center', gap: 8, paddingVertical: 4 }}>
              <Text numberOfLines={1} style={{ color: c.text, fontSize: 12, fontFamily: fonts.mono, minWidth: 76, flexShrink: 1 }}>{r.tag}</Text>
              <Text style={{ color: c.dim, fontSize: 11, fontFamily: fonts.mono, width: 80 }}>{r.publishedAt}</Text>
              {r.latest && (
                <Text style={{ color: c.accent2, fontSize: 9.5, fontWeight: '700', borderWidth: 1, borderColor: c.line, borderRadius: 5, paddingHorizontal: 5, paddingVertical: 1 }}>{t('최신')}</Text>
              )}
              {isCur && (
                <Text style={{ color: c.greenTx, fontSize: 9.5, fontWeight: '700', borderWidth: 1, borderColor: 'rgba(63,185,80,0.5)', borderRadius: 5, paddingHorizontal: 5, paddingVertical: 1 }}>{t('현재 로봇')}</Text>
              )}
              <View style={{ flex: 1 }} />
              {downloading && dlProg ? (
                <>
                  <Text style={{ color: c.accent2, fontSize: 11, fontFamily: fonts.mono }}>
                    {partLabel(dlProg.part)}{fmtMB(dlProg.written)}{dlProg.total > 0 ? ` / ${fmtMB(dlProg.total)}` : t(' 수신 중…')}
                  </Text>
                  <Tappable onPress={() => dl.current?.cancel()}
                    style={[styles.action, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.md, height: 24 }]}>
                    <Text style={{ color: c.text, fontSize: 11 }}>{t('취소')}</Text>
                  </Tappable>
                </>
              ) : (
                <Tappable onPress={() => startDownload(r)} disabled={!!dlTag || busy}
                  style={[styles.action, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.md, height: 24, opacity: dlTag || busy ? 0.4 : 1 }]}>
                  <Icon name="download" size={11} color={c.muted} />
                  <Text style={{ color: c.text, fontSize: 11, fontWeight: '600' }}>{t('받기')}</Text>
                </Tappable>
              )}
            </View>
          );
        })}
        {list && list.length === 0 ? <Text style={{ color: c.dim, fontSize: 11.5 }}>{t('릴리스가 없습니다')}</Text> : null}
        <Text style={{ color: c.dim, fontSize: 10, marginTop: 4 }}>
          {t('받은 파일은 기기에 남습니다 — 로봇 망으로 바꾼 뒤 전송해도 됩니다.')}
        </Text>
      </View>
      )}

      {busy && prog ? (
        <View style={[styles.fwBlock, { borderColor: c.line, borderRadius: radius.md, backgroundColor: c.bg, marginTop: 10 }]}>
          <View style={{ flexDirection: 'row', justifyContent: 'space-between', marginBottom: 6 }}>
            <Text style={{ color: c.text, fontSize: 11.5, fontFamily: fonts.mono }}>
              {file?.name} → {destIp}:{OTA_PORT}
            </Text>
            <Text style={{ color: c.accent2, fontSize: 11.5, fontWeight: '700' }}>
              {prog.phase === 'transfer' ? `${prog.percentage}%`
                : prog.phase === 'applying' ? t('로봇에서 설치 중')
                : prog.phase === 'reconnecting' ? `${t('다시 연결 중')} (${prog.attempt ?? 1})`
                : t('연결 중…')}
            </Text>
          </View>
          <View style={{ height: 8, borderRadius: 4, backgroundColor: c.elev, overflow: 'hidden' }}>
            <View style={{ height: 8, width: `${prog.percentage}%`, backgroundColor: c.accent }} />
          </View>
          <Text style={{ color: c.dim, fontSize: 10.5, marginTop: 6, fontFamily: fonts.mono }}>
            {prog.phase === 'applying'
              ? t('로봇이 압축 해제·설치 중입니다 — 연결이 끊겨도 로봇은 끝까지 하고, 다시 붙어 결과를 확인합니다')
              : prog.phase === 'reconnecting'
                ? `${fmtMB(prog.acked)} / ${fmtMB(prog.total)} · ${t('연결이 끊겨 다시 붙는 중 — 받은 데까지 이어서 보냅니다')}`
                : <>{fmtMB(prog.acked)} / {fmtMB(prog.total)} · {t('전송 중')} —{' '}
                  {autoDeploy ? t('완료 시 로봇이 자동으로 설치합니다') : t('저장만 됩니다 (패키지 이름 아님)')}</>}
          </Text>
          {prog.phase !== 'applying' ? (
            <View style={{ flexDirection: 'row', justifyContent: 'flex-end', marginTop: 8 }}>
              <Tappable onPress={() => { handle.current?.cancel(); setProg(null); }}
                style={[styles.action, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.md, height: 28 }]}>
                <Text style={{ color: c.text, fontSize: 12 }}>{t('취소')}</Text>
              </Tappable>
            </View>
          ) : null}
        </View>
      ) : !otaAvailable ? null : (
        <View style={{ flexDirection: 'row', gap: 8, marginTop: 10 }}>
          <Tappable onPress={pick} style={[styles.action, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.md }]}>
            <Icon name="download" size={14} color={c.muted} />
            <Text style={{ color: c.text, fontSize: 12, fontWeight: '600' }}>{t('USB·파일에서 불러오기')}</Text>
          </Tappable>
          <Tappable onPress={() => file && setConfirm(true)} style={[styles.action, {
            backgroundColor: c.accent, borderColor: c.accent, borderRadius: radius.md, opacity: file ? 1 : 0.4,
          }]}>
            <Icon name="download" size={14} color={c.onAccent} />
            <Text style={{ color: c.onAccent, fontSize: 12, fontWeight: '600' }}>{t('로봇 소프트웨어 전송')}</Text>
          </Tappable>
        </View>
      )}
      {err ? <Text style={{ color: c.redTx, fontSize: 12, marginTop: 8 }}>{t('오류:')} {err}</Text> : null}
      {doneMsg ? (
        <Text style={{ color: c.greenTx, fontSize: 12, marginTop: 8 }}>
          {t('✓ 전송 완료')}{autoDeploy ? t(' — 로봇이 압축 해제·배포를 실행합니다. 완료 후 재부팅하세요.') : t(' — rbq_ws/patches에 저장됨')}
        </Text>
      ) : null}
      <Text style={{ color: c.amberTx, fontSize: 11.5, marginTop: 8, lineHeight: 15 }}>
        ⚠ {t('로봇 소프트웨어는 업데이트 후 로봇을 재부팅해야 적용됩니다.')}
      </Text>
      </ScrollView>
      {askReboot && (
        <Modal onClose={() => setAskReboot(false)}>
          <View style={[styles.modal, { backgroundColor: c.panel, borderColor: c.line, borderRadius: radius.lg }]}>
            <Text style={{ color: c.text, fontSize: 16, fontWeight: '700', textAlign: 'center', marginBottom: 10 }}>
              {t('배포 전송 완료 — 재부팅할까요?')}
            </Text>
            <Text style={{ color: c.muted, fontSize: 12.5, lineHeight: 17, textAlign: 'center' }}>
              {t('로봇이 압축 해제·배포를 실행합니다. 반영에는 시스템 재부팅이 필요합니다.')}
            </Text>
            {tgt === 'vision' ? (
              <>
                <Text style={{ color: c.amber, fontSize: 12, lineHeight: 16, textAlign: 'center', marginTop: 8 }}>
                  {t('비전 PC 는 앱에서 재부팅할 수 없습니다 — 해당 PC 를 직접 재부팅하세요.')}
                </Text>
                <View style={{ flexDirection: 'row', gap: 9, marginTop: 14 }}>
                  <Tappable onPress={() => setAskReboot(false)}
                    style={[styles.modalBtn, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.md }]}>
                    <Text style={{ color: c.text, fontSize: 13, fontWeight: '600' }}>{t('확인')}</Text>
                  </Tappable>
                </View>
              </>
            ) : (
              <>
                <Text style={{ color: c.amberTx, fontSize: 12, lineHeight: 17, textAlign: 'center', marginTop: 8 }}>
                  ⚠ {t('재부팅하면 로봇이 힘을 잃고 주저앉습니다 — 서 있으면 먼저 앉히고 바닥에 앉아 있는지 확인하세요.')}
                </Text>
                <Text style={{ color: c.dim, fontSize: 11.5, fontFamily: fonts.mono, textAlign: 'center', marginTop: 8 }}>
                  {t('대상')} · Motion ({ip})
                </Text>
                <View style={{ flexDirection: 'row', gap: 9, marginTop: 14 }}>
                  <Tappable onPress={() => { actions.restart(ip).then(() => notify(t('로봇을 재시작하는 중입니다 — 잠시 뒤 다시 연결해 주세요')), (e) => notify(`${t('재시작 요청 실패')}: ${e?.message ?? e}`, 'warn')); setAskReboot(false); }}
                    style={[styles.modalBtn, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.md }]}>
                    <Icon name="power" size={13} color={c.text} />
                    <Text style={{ color: c.text, fontSize: 13, fontWeight: '600' }}>{t('로봇 서비스 재시작')}</Text>
                  </Tappable>
                  <Tappable onPress={() => { actions.reboot(ip).then(() => notify(t('재부팅을 요청했습니다 — 잠시 뒤 연결이 끊깁니다')), (e) => notify(`${t('재부팅 요청 실패')}: ${e?.message ?? e}`, 'warn')); setAskReboot(false); }}
                    style={[styles.modalBtn, { backgroundColor: 'rgba(231,51,28,0.85)', borderColor: c.dangerLine, borderRadius: radius.md }]}>
                    <Icon name="power" size={13} color="#fff" />
                    <Text style={{ color: '#fff', fontSize: 13, fontWeight: '600' }}>{t('로봇 PC 재부팅')}</Text>
                  </Tappable>
                </View>
                <View style={{ flexDirection: 'row', marginTop: 9 }}>
                  <Tappable onPress={() => setAskReboot(false)}
                    style={[styles.modalBtn, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.md }]}>
                    <Text style={{ color: c.text, fontSize: 13, fontWeight: '600' }}>{t('나중에')}</Text>
                  </Tappable>
                </View>
              </>
            )}
          </View>
        </Modal>
      )}
      {appConfirm && appRel ? (
        <ConfirmModal danger={false}
          title={t('앱 업데이트')}
          message={`${appRel.tag} — ${t('받는 대로 앱이 닫혔다가 새 버전으로 다시 열리고, 그동안 로봇 연결이 끊깁니다. 업데이트할까요?')}`}
          confirmLabel={t('업데이트')}
          onConfirm={() => { setAppConfirm(false); startAppUpdate(); }} onClose={() => setAppConfirm(false)} />
      ) : null}
      {confirm && file && (
        <Modal onClose={() => setConfirm(false)}>
          <View style={[styles.modal, { backgroundColor: c.panel, borderColor: c.line, borderRadius: radius.lg }]}>
            <Text style={{ color: c.text, fontSize: 16, fontWeight: '700', textAlign: 'center', marginBottom: 10 }}>
              {t('로봇으로 전송하시겠습니까?')}
            </Text>
            <KV k={t('파일')} v={file.name} />
            <KV k={t('크기')} v={fmtMB(file.size)} />
            <KV k={t('대상')} v={`${destLabel} · ${OTA_PORT}`} />
            <Text style={{ color: autoDeploy ? c.amberTx : c.muted, fontSize: 11.5, marginTop: 10, lineHeight: 15 }}>
              {autoDeploy
                ? t('⚠ 로봇 소프트웨어 패키지입니다 — 전송이 끝나면 로봇이 바로 설치를 시작합니다. 실행 중인 로봇 소프트웨어가 교체됩니다.')
                : t('로봇 소프트웨어 패키지 이름(RBQ-<버전>.tar.gz, core-<버전>.rbq)이 아니라 로봇에 저장만 되고 설치는 실행되지 않습니다.')}
            </Text>
            <View style={{ flexDirection: 'row', gap: 9, marginTop: 14 }}>
              <Tappable onPress={() => setConfirm(false)}
                style={[styles.modalBtn, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.md }]}>
                <Text style={{ color: c.text, fontSize: 13, fontWeight: '600' }}>{t('취소')}</Text>
              </Tappable>
              <Tappable onPress={start}
                style={[styles.modalBtn, {
                  backgroundColor: autoDeploy ? 'rgba(231,51,28,0.85)' : c.accent,
                  borderColor: autoDeploy ? c.dangerLine : c.accent, borderRadius: radius.md,
                }]}>
                <Icon name="download" size={13} color="#fff" />
                <Text style={{ color: '#fff', fontSize: 13, fontWeight: '600' }}>{t('전송')}</Text>
              </Tappable>
            </View>
          </View>
        </Modal>
      )}
    </View>
  );
}

const styles = StyleSheet.create({
  action: { flexDirection: 'row', alignItems: 'center', gap: 6, height: 34, paddingHorizontal: 14, borderWidth: 1 },
  fwCard: { flex: 1, paddingHorizontal: 14, paddingVertical: 11, borderWidth: 1 },
  fwBlock: { paddingHorizontal: 14, paddingVertical: 11, borderWidth: 1 },
  modal: { width: 440, borderWidth: 1, padding: 18 },
  modalBtn: { flex: 1, flexDirection: 'row', alignItems: 'center', justifyContent: 'center', gap: 6, height: 38, borderWidth: 1 },
});

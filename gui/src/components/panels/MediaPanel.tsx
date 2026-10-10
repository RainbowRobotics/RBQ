import { useEffect, useMemo, useRef, useState } from 'react';
import { View, Text, Image, Platform, Pressable, StyleSheet, ActivityIndicator, useWindowDimensions } from 'react-native';
import { useTheme } from '@/theme';
import { Icon, type IconName } from '@/components/Icon';
import { Tappable } from '@/components/anim';
import { Toggle, Segmented } from '@/components/ui/controls';
import { Modal } from '@/components/ui/overlays';
import { ConfirmModal } from '@/components/ui/ConfirmModal';
import {
  media, useMedia, humanSize, clipSeconds, mmss, stampParts, prefixOf, soundPath,
  groupCaptures, groupByDay, dayLabel, type MediaKind, type Capture,
} from '@/lib/media';
import { rest, type LogFileEntry } from '@/lib/rest';
import { saveMediaFile } from '@/lib/download';
import { robotAuth } from '@/lib/auth';
import { ptt, usePtt } from '@/lib/ptt';
import { useRobot } from '@/store/robot';
import { MediaVideo } from '@/components/MediaVideo';
import { t } from '@/lib/i18n';
import { pickDocument } from '@/lib/pickDocument';
import { SOUNDS, SOUND_CATEGORIES } from '@/lib/sounds';

const PAGE = 60;
const NEW_MS = 20_000;


function SectionHead({ title, desc, right }: { title: string; desc: string; right?: React.ReactNode }) {
  const { c } = useTheme();
  return (
    <View style={S.head}>
      <View style={{ flex: 1, minWidth: 0 }}>
        <Text style={{ color: c.text, fontSize: 16, fontWeight: '700', marginBottom: 3 }}>{title}</Text>
        <Text style={{ color: c.dim, fontSize: 11, lineHeight: 16 }}>{desc}</Text>
      </View>
      {right && <View style={S.headRight}>{right}</View>}
    </View>
  );
}

type Tone = 'accent' | 'plain' | 'green' | 'red';

function Pill({ icon, label, onPress, tone = 'plain', busy, disabled }: {
  icon: IconName; label: string; onPress: () => void; tone?: Tone; busy?: boolean; disabled?: boolean;
}) {
  const { c } = useTheme();
  const look = {
    accent: { bg: c.accent, bd: c.accent, fg: c.onAccent, ic: c.onAccent },
    plain: { bg: c.elev, bd: c.line, fg: c.text, ic: c.accent2 },
    green: { bg: 'rgba(63,185,80,0.14)', bd: 'rgba(63,185,80,0.6)', fg: c.greenTx, ic: c.green },
    red: { bg: 'rgba(231,51,28,0.12)', bd: 'rgba(231,51,28,0.55)', fg: c.redbright, ic: c.redbright },
  }[tone];
  return (
    <Tappable onPress={disabled || busy ? undefined : onPress}
      style={[S.pill, { backgroundColor: look.bg, borderColor: look.bd, opacity: disabled ? 0.45 : 1 }]}>
      {busy ? <ActivityIndicator size="small" color={look.ic} /> : <Icon name={icon} size={14} color={look.ic} />}
      <Text numberOfLines={1} style={{ color: look.fg, fontSize: 12, fontWeight: '700' }}>{label}</Text>
    </Tappable>
  );
}

function IconBtn({ icon, onPress, busy, label, disabled }: {
  icon: IconName; onPress: () => void; busy?: boolean; label: string; disabled?: boolean;
}) {
  const { c, radius } = useTheme();
  return (
    <Tappable onPress={busy || disabled ? undefined : onPress} accessibilityLabel={label}
      style={[S.iconBtn, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.sm, opacity: disabled ? 0.4 : 1 }]}>
      {busy ? <ActivityIndicator size="small" color={c.accent2} /> : <Icon name={icon} size={15} color={c.muted} />}
    </Tappable>
  );
}


type DelItem = { key: string; paths: string[] };
type SelectState = { on: boolean; checked: (key: string) => boolean; toggle: (key: string) => void };

function useDeleter(kind: MediaKind, items: DelItem[]) {
  const { c } = useTheme();
  const [on, setOn] = useState(false);
  const [sel, setSel] = useState<Set<string>>(new Set());
  const [ask, setAsk] = useState<string[] | null>(null);
  const [busy, setBusy] = useState(false);
  const [note, setNote] = useState<string | null>(null);
  const live = new Set(items.map((i) => i.key));
  const picked = [...sel].filter((k) => live.has(k));
  const allOn = items.length > 0 && picked.length === items.length;
  const cancel = () => { setOn(false); setSel(new Set()); };
  const select: SelectState = {
    on,
    checked: (k) => sel.has(k),
    toggle: (k) => setSel((p) => { const n = new Set(p); if (n.has(k)) n.delete(k); else n.add(k); return n; }),
  };
  const run = async (paths: string[]) => {
    setBusy(true); setNote(null);
    try {
      const skipped = await media.deleteFiles(kind, paths);
      if (skipped) setNote(t('{n}개는 지우지 못했습니다 — 녹화·녹음 중이거나 이미 없는 파일입니다').replace('{n}', String(skipped)));
      cancel();
    } catch (e) {
      setNote((e as Error)?.message ?? t('지우지 못했습니다'));
    } finally { setBusy(false); }
  };
  const head = on ? (
    <>
      <Text style={{ color: c.muted, fontSize: 11.5 }}>{t('{n}개 선택').replace('{n}', String(picked.length))}</Text>
      <Pill icon="check" label={allOn ? t('선택 해제') : t('전체 선택')}
        onPress={() => setSel(allOn ? new Set() : new Set(items.map((i) => i.key)))} />
      <Pill icon="trash" tone="red" label={t('삭제')} busy={busy} disabled={picked.length === 0}
        onPress={() => setAsk(items.filter((i) => sel.has(i.key)).flatMap((i) => i.paths))} />
      <Pill icon="x" label={t('취소')} onPress={cancel} />
    </>
  ) : (
    <IconBtn icon="trash" label={t('파일 지우기')} disabled={items.length === 0} onPress={() => { setNote(null); setOn(true); }} />
  );
  const modal = ask && (
    <ConfirmModal title={t('로봇에서 지울까요?')}
      message={t('파일 {n}개를 로봇에서 지웁니다. 되돌릴 수 없습니다.').replace('{n}', String(ask.length))}
      confirmLabel={t('삭제')} onClose={() => setAsk(null)}
      onConfirm={() => { const p = ask; setAsk(null); void run(p); }} />
  );
  return { select, head, modal, note };
}

function CheckMark({ checked }: { checked: boolean }) {
  const { c } = useTheme();
  return (
    <View style={[S.check, { borderColor: checked ? c.accent : c.line, backgroundColor: checked ? c.accent : c.panel }]}>
      {checked && <Icon name="check" size={12} color={c.onAccent} strokeWidth={3} />}
    </View>
  );
}

function RefreshBtn({ kind }: { kind: MediaKind }) {
  const loading = useMedia((s) => s.lists[kind].loading);
  return <IconBtn icon="recover" label={t('새로고침')} busy={loading} onPress={() => media.refresh(kind)} />;
}

function DayHead({ day, count }: { day: string; count: number }) {
  const { c, fonts } = useTheme();
  return (
    <View style={S.dayHead}>
      <Text style={{ color: c.muted, fontSize: 11, fontWeight: '700' }}>{dayLabel(day)}</Text>
      <Text style={{ color: c.dim, fontSize: 10, fontFamily: fonts.mono }}>{count}</Text>
      <View style={{ flex: 1, height: 1, backgroundColor: c.line2 }} />
    </View>
  );
}

function Empty({ icon, title, sub }: { icon: IconName; title: string; sub: string }) {
  const { c, radius } = useTheme();
  return (
    <View style={[S.empty, { borderColor: c.line2, borderRadius: radius.md }]}>
      <View style={[S.emptyIc, { backgroundColor: c.elev, borderRadius: radius.md }]}>
        <Icon name={icon} size={22} color={c.dim} />
      </View>
      <Text style={{ color: c.text, fontSize: 13, fontWeight: '600' }}>{title}</Text>
      <Text style={{ color: c.dim, fontSize: 11, textAlign: 'center', lineHeight: 16, maxWidth: 380 }}>{sub}</Text>
    </View>
  );
}

function Notice({ text }: { text: string }) {
  const { c, radius } = useTheme();
  return (
    <View style={[S.notice, { borderRadius: radius.sm, backgroundColor: 'rgba(251,186,22,0.10)', borderColor: 'rgba(251,186,22,0.45)' }]}>
      <Icon name="warn" size={14} color={c.amber} />
      <Text style={{ color: c.amberTx, fontSize: 11.5, flex: 1, lineHeight: 16 }}>{text}</Text>
    </View>
  );
}

function Loading() {
  const { c } = useTheme();
  return <View style={{ paddingVertical: 40, alignItems: 'center' }}><ActivityIndicator color={c.dim} /></View>;
}

function MoreBtn({ shown, total, onMore }: { shown: number; total: number; onMore: () => void }) {
  if (shown >= total) return null;
  return (
    <View style={{ alignItems: 'center', marginTop: 12 }}>
      <Pill icon="caret" label={`${t('더 보기')} (${total - shown})`} onPress={onMore} />
    </View>
  );
}

function useDownload() {
  const ip = useRobot((s) => s.ip);
  const [busy, setBusy] = useState<string | null>(null);
  const [err, setErr] = useState<string | null>(null);
  const run = async (kind: MediaKind, f: LogFileEntry) => {
    setBusy(f.path); setErr(null);
    try { await saveMediaFile(ip, kind, f.path, f.size); }
    catch (e) { setErr((e as Error)?.message ?? t('파일을 받지 못했습니다')); }
    finally { setBusy(null); }
  };
  return { busy, err, run };
}


type ImgSrc = { uri: string; headers?: Record<string, string> } | null;

function usePhotoSource(path: string | undefined): ImgSrc {
  const ip = useRobot((s) => s.ip);
  const [blobUri, setBlobUri] = useState<string | null>(null);
  const url = path ? rest.mediaFileUrl(ip, 'photo', path) : null;
  const web = Platform.OS === 'web';
  useEffect(() => {
    setBlobUri(null);
    if (!web || !url || !path) return;
    let alive = true;
    let made: string | null = null;
    rest.mediaBinary(ip, 'photo', path)
      .then((buf: ArrayBuffer) => {
        if (!alive) return;
        made = URL.createObjectURL(new Blob([buf], { type: 'image/jpeg' }));
        setBlobUri(made);
      })
      .catch(() => {});
    return () => { alive = false; if (made) URL.revokeObjectURL(made); };
  }, [web, url, ip, path]);
  if (!url) return null;
  if (web) return blobUri ? { uri: blobUri } : null;
  return { uri: url, headers: robotAuth() };
}

function Thumb({ file, cam, w, onPress }: { file?: LogFileEntry; cam: string; w: number; onPress?: () => void }) {
  const { c, radius } = useTheme();
  const src = usePhotoSource(file?.path);
  const h = Math.round(w * 9 / 16);
  return (
    <Tappable onPress={file ? onPress : undefined}
      style={{ width: w, height: h, borderRadius: radius.sm, overflow: 'hidden', backgroundColor: c.panel2 }}>
      {file && src && <Image source={src} style={StyleSheet.absoluteFill} resizeMode="cover" />}
      {file && !src && <View style={S.center}><ActivityIndicator size="small" color={c.dim} /></View>}
      {!file && <View style={S.center}><Text style={{ color: c.dim, fontSize: 10 }}>{t('없음')}</Text></View>}
      <View style={S.camChip}>
        <Text style={{ color: '#fff', fontSize: 9, fontWeight: '700', letterSpacing: 0.5 }}>{cam}</Text>
      </View>
    </Tappable>
  );
}

function CaptureCard({ cap, thumbW, onOpen: openFile, select }: {
  cap: Capture; thumbW: number; onOpen: (f: LogFileEntry) => void; select?: SelectState;
}) {
  const { c, radius, fonts } = useTheme();
  const picking = !!select?.on;
  const checked = picking && select!.checked(cap.key);
  const onOpen = (f: LogFileEntry) => (picking ? select!.toggle(cap.key) : openFile(f));
  const isNew = Date.now() - cap.mtime < NEW_MS;
  const size = (cap.front?.size ?? 0) + (cap.rear?.size ?? 0) + cap.other.reduce((a, f) => a + f.size, 0);
  return (
    <View style={[S.capCard, {
      backgroundColor: c.elev, borderRadius: radius.md,
      borderColor: checked ? c.accent : isNew ? 'rgba(77,156,245,0.6)' : c.line,
    }]}>
      {picking && <View style={S.capCheck} pointerEvents="none"><CheckMark checked={checked} /></View>}
      <View style={{ flexDirection: 'row', gap: 6 }}>
        <Thumb file={cap.front} cam={t('전방')} w={thumbW} onPress={() => cap.front && onOpen(cap.front)} />
        <Thumb file={cap.rear} cam={t('후방')} w={thumbW} onPress={() => cap.rear && onOpen(cap.rear)} />
      </View>
      <View style={S.capFoot}>
        <Text style={{ color: c.text, fontSize: 12, fontFamily: fonts.mono }}>{cap.time || '—'}</Text>
        {isNew && (
          <View style={[S.badge, { backgroundColor: 'rgba(77,156,245,0.14)', borderColor: 'rgba(77,156,245,0.55)' }]}>
            <Text style={{ color: c.accent2, fontSize: 8.5, fontWeight: '700', letterSpacing: 0.5 }}>NEW</Text>
          </View>
        )}
        <View style={{ flex: 1 }} />
        <Text style={{ color: c.dim, fontSize: 10, fontFamily: fonts.mono }}>{humanSize(size)}</Text>
      </View>
    </View>
  );
}

function PhotoViewer({ files, index, onIndex, onClose }: {
  files: LogFileEntry[]; index: number; onIndex: (i: number) => void; onClose: () => void;
}) {
  const { c, radius, fonts } = useTheme();
  const { width: winW, height: winH } = useWindowDimensions();
  const f = files[index];
  const src = usePhotoSource(f?.path);
  const dl = useDownload();
  if (!f) return null;
  const st = stampParts(f.path);
  const cam = prefixOf(f.path) === 'rear' ? t('후방') : prefixOf(f.path) === 'front' ? t('전방') : '';
  const imgW = Math.min(winW - 64, (winH - 150) * 16 / 9, 1100);
  const imgH = imgW * 9 / 16;
  return (
    <Modal onClose={onClose}>
      <View style={[S.viewer, { backgroundColor: c.panel, borderColor: c.line, borderRadius: radius.lg, width: imgW + 24 }]}>
        <View style={S.viewerBar}>
          <Text style={{ color: c.text, fontSize: 13, fontWeight: '700' }}>{cam}</Text>
          <Text style={{ color: c.muted, fontSize: 12, fontFamily: fonts.mono }}>
            {st ? `${dayLabel(st.day)}  ${st.time}` : f.path}
          </Text>
          <View style={{ flex: 1 }} />
          <IconBtn icon="x" label={t('닫기')} onPress={onClose} />
        </View>
        <View style={{ width: imgW, height: imgH, borderRadius: radius.md, overflow: 'hidden', backgroundColor: c.panel2 }}>
          {src ? <Image source={src} style={StyleSheet.absoluteFill} resizeMode="contain" />
            : <View style={S.center}><ActivityIndicator color={c.dim} /></View>}
        </View>
        {dl.err && <Notice text={dl.err} />}
        <View style={S.viewerBar}>
          <IconBtn icon="prev" label={t('이전')} disabled={index <= 0} onPress={() => onIndex(Math.max(0, index - 1))} />
          <Text style={{ color: c.muted, fontSize: 11.5, fontFamily: fonts.mono, minWidth: 64, textAlign: 'center' }}>
            {index + 1} / {files.length}
          </Text>
          <IconBtn icon="next" label={t('다음')} disabled={index >= files.length - 1} onPress={() => onIndex(Math.min(files.length - 1, index + 1))} />
          <Text style={{ color: c.dim, fontSize: 10.5, fontFamily: fonts.mono, marginLeft: 6 }}>{humanSize(f.size)}</Text>
          <View style={{ flex: 1 }} />
          <Pill icon="download" label={dl.busy ? t('받는 중…') : t('내려받기')} busy={!!dl.busy} onPress={() => dl.run('photo', f)} />
        </View>
      </View>
    </Modal>
  );
}

export function MediaPhotos() {
  const list = useMedia((s) => s.lists.photo);
  const shooting = useMedia((s) => s.shooting);
  const lastShot = useMedia((s) => s.lastShot);
  const [limit, setLimit] = useState(PAGE);
  const [viewIdx, setViewIdx] = useState<number | null>(null);
  const { width: winW } = useWindowDimensions();
  const thumbW = winW >= 1400 ? 150 : winW >= 1000 ? 132 : 112;
  const caps = useMemo(() => groupCaptures(list.files), [list.files]);
  const shown = caps.slice(0, limit);
  const days = groupByDay(shown, (x) => x.day);
  const flat = useMemo(() => caps.flatMap((cp) => [cp.front, cp.rear, ...cp.other].filter(Boolean) as LogFileEntry[]), [caps]);
  const open = (f: LogFileEntry) => setViewIdx(Math.max(0, flat.findIndex((x) => x.path === f.path)));
  const showShotErr = lastShot && !lastShot.ok && Date.now() - lastShot.at < 15_000;
  const del = useDeleter('photo', useMemo(() => caps.map((cp) => ({
    key: cp.key, paths: [cp.front, cp.rear, ...cp.other].filter(Boolean).map((f) => f!.path),
  })), [caps]));
  return (
    <>
      <SectionHead title={t('사진')} desc={t('로봇 카메라로 찍은 사진입니다. 한 번 찍으면 전방·후방이 함께 저장됩니다.')}
        right={del.select.on ? del.head : <>
          {del.head}
          <RefreshBtn kind="photo" />
          <Pill icon="maximize" tone="accent" label={shooting ? t('촬영 중…') : t('사진 찍기')} busy={shooting}
            onPress={() => { void media.snapshot(); }} />
        </>} />
      {showShotErr && <Notice text={lastShot!.msg} />}
      {del.note && <Notice text={del.note} />}
      {del.modal}
      {list.error && <Notice text={list.error} />}
      {!list.loaded ? <Loading />
        : caps.length === 0
          ? <Empty icon="maximize" title={t('아직 찍은 사진이 없습니다')}
              sub={t('위의 사진 찍기, 또는 제어 화면 워키토키 옆의 촬영 버튼으로 찍으면 여기에 모입니다.')} />
          : days.map((g) => (
            <View key={g.day} style={{ marginBottom: 6 }}>
              <DayHead day={g.day} count={g.items.length} />
              <View style={S.grid}>
                {g.items.map((cp) => <CaptureCard key={cp.key} cap={cp} thumbW={thumbW} onOpen={open} select={del.select} />)}
              </View>
            </View>
          ))}
      <MoreBtn shown={shown.length} total={caps.length} onMore={() => setLimit((n) => n + PAGE)} />
      {viewIdx != null && <PhotoViewer files={flat} index={viewIdx} onIndex={setViewIdx} onClose={() => setViewIdx(null)} />}
    </>
  );
}


function FileRow({ icon, iconTone, title, meta, active, children, select, selKey }: {
  icon: IconName; iconTone?: string; title: string; meta: string; active?: boolean; children: React.ReactNode;
  select?: SelectState; selKey?: string;
}) {
  const { c, radius, fonts } = useTheme();
  if (select?.on && selKey) {
    const on = select.checked(selKey);
    return (
      <Tappable onPress={() => select.toggle(selKey)} style={[S.row, {
        borderRadius: radius.md,
        backgroundColor: on ? 'rgba(77,156,245,0.08)' : 'transparent',
        borderColor: on ? 'rgba(77,156,245,0.45)' : 'transparent',
      }]}>
        <CheckMark checked={on} />
        <View style={{ flex: 1, minWidth: 0 }}>
          <Text numberOfLines={1} style={{ color: c.text, fontSize: 12.5, fontWeight: '600' }}>{title}</Text>
          <Text numberOfLines={1} style={{ color: c.dim, fontSize: 10.5, fontFamily: fonts.mono, marginTop: 2 }}>{meta}</Text>
        </View>
      </Tappable>
    );
  }
  return (
    <View style={[S.row, {
      borderRadius: radius.md,
      backgroundColor: active ? 'rgba(63,185,80,0.08)' : 'transparent',
      borderColor: active ? 'rgba(63,185,80,0.45)' : 'transparent',
    }]}>
      <View style={[S.rowIc, { backgroundColor: c.elev, borderRadius: radius.sm }]}>
        <Icon name={icon} size={16} color={iconTone ?? c.muted} />
      </View>
      <View style={{ flex: 1, minWidth: 0 }}>
        <Text numberOfLines={1} style={{ color: c.text, fontSize: 12.5, fontWeight: '600' }}>{title}</Text>
        <Text numberOfLines={1} style={{ color: c.dim, fontSize: 10.5, fontFamily: fonts.mono, marginTop: 2 }}>{meta}</Text>
      </View>
      <View style={S.rowActs}>{children}</View>
    </View>
  );
}


function videoCam(path: string): string {
  const p = prefixOf(path);
  if (p === 'front') return t('전방 카메라');
  if (p === 'rear') return t('후방 카메라');
  return p || t('카메라');
}

function VideoViewer({ kind, file: first, siblings, onClose }: {
  kind: MediaKind; file: LogFileEntry; onClose: () => void;
  siblings?: LogFileEntry[];
}) {
  const [file, setFile] = useState(first);
  const { c, radius, fonts } = useTheme();
  const { width: winW, height: winH } = useWindowDimensions();
  const dl = useDownload();
  const st = stampParts(file.path);
  const w = Math.min(winW - 64, (winH - 150) * 16 / 9, 1100);
  return (
    <Modal onClose={onClose}>
      <View style={[S.viewer, { backgroundColor: c.panel, borderColor: c.line, borderRadius: radius.lg, width: w + 24 }]}>
        <View style={S.viewerBar}>
          {siblings && siblings.length > 1
            ? siblings.map((f) => (
              <Pill key={f.path} icon="play2" label={videoCam(f.path)} tone={f.path === file.path ? 'accent' : 'plain'} onPress={() => setFile(f)} />
            ))
            : <Text style={{ color: c.text, fontSize: 13, fontWeight: '700' }}>{videoCam(file.path)}</Text>}
          <Text style={{ color: c.muted, fontSize: 12, fontFamily: fonts.mono }}>
            {st ? `${dayLabel(st.day)}  ${st.time}` : file.path}
          </Text>
          <View style={{ flex: 1 }} />
          <IconBtn icon="x" label={t('닫기')} onPress={onClose} />
        </View>
        <View style={{ width: w, height: w * 9 / 16, borderRadius: radius.md, overflow: 'hidden', backgroundColor: '#000' }}>
          <MediaVideo key={file.path} kind={kind} path={file.path} size={file.size} />
        </View>
        {dl.err && <Notice text={dl.err} />}
        <View style={S.viewerBar}>
          <Text style={{ color: c.dim, fontSize: 10.5, fontFamily: fonts.mono }}>{humanSize(file.size)} · {file.path.split('/').pop()}</Text>
          <View style={{ flex: 1 }} />
          <Pill icon="download" label={dl.busy ? t('받는 중…') : t('내려받기')} busy={!!dl.busy} onPress={() => dl.run(kind, file)} />
        </View>
      </View>
    </Modal>
  );
}

function VideoList({ kind, files, loaded, loading, emptyIcon, emptyTitle, emptySub, hideAfter, paired, select }: {
  kind: MediaKind; files: LogFileEntry[]; loaded: boolean; loading: boolean;
  select?: SelectState;
  paired?: boolean;
  emptyIcon: IconName; emptyTitle: string; emptySub: string;
  hideAfter?: number;
}) {
  const { c } = useTheme();
  const [limit, setLimit] = useState(PAGE);
  const [open, setOpen] = useState<{ file: LogFileEntry; siblings?: LogFileEntry[] } | null>(null);
  const dl = useDownload();
  const list = files.filter((f) => f.path.endsWith('.mp4') && !(hideAfter && f.mtime >= hideAfter - 2000));
  const caps = useMemo(() => (paired ? groupCaptures(list) : []), [paired, list]);
  const shown = list.slice(0, limit);
  const days = groupByDay(shown, (f) => stampParts(f.path)?.day ?? '');
  const shownCaps = caps.slice(0, limit);
  const capDays = groupByDay(shownCaps, (x) => x.day);
  const camsOf = (cp: Capture) => [cp.front, cp.rear, ...cp.other].filter(Boolean) as LogFileEntry[];
  const saveAll = async (cams: LogFileEntry[]) => { for (const f of cams) await dl.run(kind, f); };
  if (paired) {
    return (
      <>
        {dl.err && <Notice text={dl.err} />}
        {!loaded ? <Loading />
          : caps.length === 0 ? <Empty icon={emptyIcon} title={emptyTitle} sub={emptySub} />
          : capDays.map((g) => (
            <View key={g.day} style={{ marginBottom: 6 }}>
              <DayHead day={g.day} count={g.items.length} />
              {g.items.map((cp) => {
                const cams = camsOf(cp);
                const total = cams.reduce((a, f) => a + f.size, 0);
                return (
                  <FileRow key={cp.key} icon="play2" iconTone={c.accent2} select={select} selKey={cp.key}
                    title={`${t('녹화')} · ${cp.time}`}
                    meta={`${cams.map((f) => videoCam(f.path)).join(' · ')} · ${humanSize(total)}`}>
                    {cams.map((f) => (
                      <Pill key={f.path} icon="play2" label={prefixOf(f.path) === 'front' ? t('전방') : prefixOf(f.path) === 'rear' ? t('후방') : videoCam(f.path)}
                        onPress={() => setOpen({ file: f, siblings: cams })} />
                    ))}
                    <IconBtn icon="download" label={t('내려받기')} busy={cams.some((f) => dl.busy === f.path)} onPress={() => { void saveAll(cams); }} />
                  </FileRow>
                );
              })}
            </View>
          ))}
        {loading && loaded && caps.length === 0 && <Loading />}
        <MoreBtn shown={shownCaps.length} total={caps.length} onMore={() => setLimit((n) => n + PAGE)} />
        {open && <VideoViewer kind={kind} file={open.file} siblings={open.siblings} onClose={() => setOpen(null)} />}
      </>
    );
  }
  return (
    <>
      {dl.err && <Notice text={dl.err} />}
      {!loaded ? <Loading />
        : list.length === 0 ? <Empty icon={emptyIcon} title={emptyTitle} sub={emptySub} />
        : days.map((g) => (
          <View key={g.day} style={{ marginBottom: 6 }}>
            <DayHead day={g.day} count={g.items.length} />
            {g.items.map((f) => {
              const st = stampParts(f.path);
              return (
                <FileRow key={f.path} icon="play2" iconTone={c.accent2} select={select} selKey={f.path}
                  title={`${videoCam(f.path)} · ${st?.time ?? ''}`}
                  meta={`${humanSize(f.size)} · ${f.path.split('/').pop()}`}>
                  <Pill icon="play2" label={t('보기')} onPress={() => setOpen({ file: f })} />
                  <IconBtn icon="download" label={t('내려받기')} busy={dl.busy === f.path} onPress={() => dl.run(kind, f)} />
                </FileRow>
              );
            })}
          </View>
        ))}
      {loading && loaded && list.length === 0 && <Loading />}
      <MoreBtn shown={shown.length} total={list.length} onMore={() => setLimit((n) => n + PAGE)} />
      {open && <VideoViewer kind={kind} file={open.file} onClose={() => setOpen(null)} />}
    </>
  );
}

function useVideoElapsed(): number {
  const since = useMedia((s) => s.videoSince);
  const [now, setNow] = useState(Date.now());
  useEffect(() => {
    if (!since) return;
    const id = setInterval(() => setNow(Date.now()), 500);
    return () => clearInterval(id);
  }, [since]);
  return since ? (now - since) / 1000 : 0;
}

function VideoRecordCard() {
  const { c, radius, fonts } = useTheme();
  const since = useMedia((s) => s.videoSince);
  const busy = useMedia((s) => s.videoBusy);
  const last = useMedia((s) => s.lastShot);
  const secs = useVideoElapsed();
  const on = since !== 0;
  const err = last && !last.ok && Date.now() - last.at < 15_000 ? last.msg : null;
  return (
    <View style={[S.recCard, { backgroundColor: c.panel2, borderColor: on ? 'rgba(231,51,28,0.45)' : c.line, borderRadius: radius.lg }]}>
      <Tappable onPress={busy ? undefined : () => { void media.toggleVideo(); }}
        accessibilityLabel={on ? t('녹화 정지') : t('녹화 시작')}
        style={[S.recBtn, { borderColor: on ? 'rgba(231,51,28,0.6)' : c.line, backgroundColor: on ? 'rgba(231,51,28,0.12)' : c.elev }]}>
        {busy ? <ActivityIndicator size="small" color={c.redbright} />
          : <View style={on ? [S.stopSq, { backgroundColor: c.redbright }] : [S.recDot, { backgroundColor: c.red }]} />}
      </Tappable>
      <View style={{ flex: 1, minWidth: 0, gap: 3 }}>
        <View style={{ flexDirection: 'row', alignItems: 'baseline', gap: 8 }}>
          <Text style={{ color: on ? c.redbright : c.text, fontSize: 13, fontWeight: '700' }}>{on ? t('녹화 중') : t('새 녹화')}</Text>
          {on && <Text style={{ color: c.redbright, fontSize: 13, fontFamily: fonts.mono }}>{mmss(secs)}</Text>}
        </View>
        <Text style={{ color: err ? c.amberTx : c.dim, fontSize: 11, lineHeight: 16 }}>
          {err ?? (on ? t('다시 누르면 저장합니다 — 전방·후방이 각각 한 파일로 남습니다')
                      : t('전방·후방 카메라를 함께 녹화합니다(초당 10장). 한 번에 최대 30분입니다'))}
        </Text>
      </View>
    </View>
  );
}

function videoFiles(files: LogFileEntry[], hideAfter?: number) {
  return files.filter((f) => f.path.endsWith('.mp4') && !(hideAfter && f.mtime >= hideAfter - 2000));
}

export function MediaClips() {
  const list = useMedia((s) => s.lists.clip);
  const since = useMedia((s) => s.videoSince);
  const del = useDeleter('clip', useMemo(() => groupCaptures(videoFiles(list.files, since || undefined)).map((cp) => ({
    key: cp.key, paths: [cp.front, cp.rear, ...cp.other].filter(Boolean).map((f) => f!.path),
  })), [list.files, since]));
  return (
    <>
      <SectionHead title={t('녹화')} desc={t('버튼으로 녹화한 영상입니다. 블랙박스 상시 녹화와는 따로 모입니다.')}
        right={del.select.on ? del.head : <>{del.head}<RefreshBtn kind="clip" /></>} />
      <VideoRecordCard />
      {list.error && <Notice text={list.error} />}
      {del.note && <Notice text={del.note} />}
      {del.modal}
      <VideoList kind="clip" paired select={del.select} files={list.files} loaded={list.loaded} loading={list.loading} hideAfter={since || undefined}
        emptyIcon="play2" emptyTitle={t('아직 녹화한 영상이 없습니다')}
        emptySub={t('위 카드나 제어 화면 워키토키 옆의 촬영 버튼으로 녹화하면 여기에 모입니다.')} />
    </>
  );
}

export function MediaArchive() {
  const list = useMedia((s) => s.lists.archive);
  const del = useDeleter('archive', useMemo(() => videoFiles(list.files).map((f) => ({ key: f.path, paths: [f.path] })), [list.files]));
  return (
    <>
      <SectionHead title={t('블랙박스')}
        desc={t('로봇이 스스로 계속 녹화한 영상입니다. 30분 단위로 나뉘어 저장됩니다.')}
        right={del.select.on ? del.head : <>{del.head}<RefreshBtn kind="archive" /></>} />
      {list.error && <Notice text={list.error} />}
      {del.note && <Notice text={del.note} />}
      {del.modal}
      <VideoList kind="archive" select={del.select} files={list.files} loaded={list.loaded} loading={list.loading}
        emptyIcon="calendar" emptyTitle={t('상시 녹화 파일이 없습니다')}
        emptySub={t('로봇의 블랙박스 상시 녹화가 켜져 있어야 쌓입니다. 넘어짐 같은 사고 순간 클립은 로그 화면의 블랙박스에 있습니다.')} />
    </>
  );
}


function useRecElapsed(): number {
  const since = useMedia((s) => s.recordingSince);
  const on = useMedia((s) => s.recordingPath != null);
  const [now, setNow] = useState(Date.now());
  useEffect(() => {
    if (!on) return;
    const id = setInterval(() => setNow(Date.now()), 500);
    return () => clearInterval(id);
  }, [on]);
  return on && since ? (now - since) / 1000 : 0;
}

export function RecordControl({ compact, onOpenList }: { compact?: boolean; onOpenList?: () => void }) {
  const { c, radius, fonts } = useTheme();
  const recPath = useMedia((s) => s.recordingPath);
  const picked = useMedia((s) => s.recordingSource);
  const noMic = usePtt((s) => s.status?.input === 'none');
  const micQuiet = usePtt((s) => s.status?.input === 'robot' && s.status?.mic_signal === false);
  const source = noMic ? 'uplink' : picked;
  useEffect(() => { ptt.refreshStatus(); }, []);
  const heldRef = useRef(false);
  useEffect(() => () => { if (heldRef.current) { heldRef.current = false; ptt.stopSpeak(); } }, []);
  const speaking = usePtt((s) => s.speaking);
  const secs = useRecElapsed();
  const [busy, setBusy] = useState(false);
  const [err, setErr] = useState<string | null>(null);
  const recording = recPath != null;
  const toggle = async () => {
    setErr(null); setBusy(true);
    try { if (recording) await media.stopRecord(); else await media.startRecord(source); }
    catch (e) { setErr((e as Error)?.message ?? t('녹음을 시작하지 못했습니다')); }
    finally { setBusy(false); }
  };
  const quiet = useMedia((s) => s.lastRecQuiet);
  const quietMsg = !recording && quiet
    ? (quiet.source === 'mic'
      ? t('방금 녹음에 소리가 거의 없습니다 — 로봇에 마이크가 꽂혀 있는지 확인하세요')
      : t('방금 녹음에 소리가 거의 없습니다 — 녹음 중에 워키토키를 누르고 말해야 담깁니다'))
    : null;
  const pickHint = !recording && source === 'mic' && micQuiet
    ? t('로봇 마이크에 소리가 거의 들어오지 않습니다 — 마이크가 꽂혀 있는지 확인하세요') : null;
  const hint = err ?? quietMsg ?? pickHint ?? (recording
    ? (source === 'uplink' ? t('누르고 말하기로 말하는 동안 담깁니다') : t('로봇 주변 소리를 담는 중입니다'))
    : (source === 'uplink' ? t('내가 말한 소리를 로봇에 저장합니다 — 로봇 스피커로 다시 틀 수 있습니다') : t('로봇 마이크에 들리는 소리를 저장합니다')));

  const srcOptions: { key: 'uplink' | 'mic'; label: string }[] = [{ key: 'uplink', label: t('내 마이크') }, { key: 'mic', label: t('로봇 마이크') }];

  const recBtn = (
    <Tappable onPress={busy ? undefined : toggle} accessibilityLabel={recording ? t('녹음 정지') : t('녹음 시작')}
      style={[compact ? S.recBtnSm : S.recBtn, {
        borderColor: recording ? 'rgba(231,51,28,0.6)' : c.line,
        backgroundColor: recording ? 'rgba(231,51,28,0.12)' : c.elev,
      }]}>
      {busy ? <ActivityIndicator size="small" color={c.redbright} />
        : <View style={recording ? [S.stopSq, { backgroundColor: c.redbright }] : [S.recDot, { backgroundColor: c.red }]} />}
    </Tappable>
  );

  if (compact) {
    return (
      <View style={{ marginTop: 12, paddingTop: 11, borderTopWidth: 1, borderColor: c.line2, gap: 8 }}>
        <View style={{ flexDirection: 'row', alignItems: 'center', gap: 10 }}>
          {recBtn}
          <View style={{ flex: 1 }}>
            <Text style={{ fontSize: 11, fontWeight: '600', color: recording ? c.redbright : c.text }}>
              {recording ? `${t('녹음 중')}  ` : t('녹음')}
              {recording && <Text style={{ fontFamily: fonts.mono }}>{mmss(secs)}</Text>}
            </Text>
            <Text style={{ fontSize: 9, color: err || quietMsg || pickHint ? c.amberTx : c.dim, marginTop: 1 }} numberOfLines={3}>{hint}</Text>
          </View>
          {onOpenList && (
            <Tappable onPress={onOpenList} style={{ paddingVertical: 4, paddingLeft: 4 }}>
              <Text style={{ fontSize: 10, color: c.accent2, fontWeight: '600' }}>{t('목록')} ›</Text>
            </Tappable>
          )}
        </View>
        {!recording && !noMic && (
          <Segmented options={srcOptions} value={source} onChange={(v) => useMedia.getState().setRecordingSource(v)} />
        )}
      </View>
    );
  }

  return (
    <View style={[S.recCard, { backgroundColor: c.panel2, borderColor: recording ? 'rgba(231,51,28,0.45)' : c.line, borderRadius: radius.lg }]}>
      {recBtn}
      <View style={{ flex: 1, minWidth: 0, gap: 3 }}>
        <View style={{ flexDirection: 'row', alignItems: 'baseline', gap: 8 }}>
          <Text style={{ color: recording ? c.redbright : c.text, fontSize: 13, fontWeight: '700' }}>
            {recording ? t('녹음 중') : t('새 녹음')}
          </Text>
          {recording && <Text style={{ color: c.redbright, fontSize: 13, fontFamily: fonts.mono }}>{mmss(secs)}</Text>}
          {recording && <Text style={{ color: c.dim, fontSize: 11 }}>· {source === 'uplink' ? t('내 마이크') : t('로봇 마이크')}</Text>}
        </View>
        <Text style={{ color: err || quietMsg || pickHint ? c.amberTx : c.dim, fontSize: 11, lineHeight: 16 }}>{hint}</Text>
      </View>
      {recording
        ? source === 'uplink' && (
          <Pressable onPressIn={() => { heldRef.current = true; void ptt.startSpeak(); }}
            onPressOut={() => { heldRef.current = false; ptt.stopSpeak(); }}
            style={[S.pill, { height: 40, paddingHorizontal: 16, touchAction: 'none', userSelect: 'none' } as any, speaking
              ? { backgroundColor: 'rgba(63,185,80,0.16)', borderColor: 'rgba(63,185,80,0.65)' }
              : { backgroundColor: c.elev, borderColor: c.line }]}>
            <Icon name="mic" size={15} color={speaking ? c.green : c.accent2} />
            <Text style={{ color: speaking ? c.greenTx : c.text, fontSize: 12, fontWeight: '700' }}>
              {speaking ? t('말하는 중…') : t('누르고 말하기')}
            </Text>
          </Pressable>
        )
        : !noMic && (
          <View style={{ width: 190 }}>
            <Segmented options={srcOptions} value={source} onChange={(v) => useMedia.getState().setRecordingSource(v)} />
          </View>
        )}
    </View>
  );
}

export function MediaAudio() {
  const { c } = useTheme();
  const list = useMedia((s) => s.lists.audio);
  const playing = useMedia((s) => s.playingPath);
  const loop = useMedia((s) => s.loop);
  const recPath = useMedia((s) => s.recordingPath);
  const [limit, setLimit] = useState(PAGE);
  const dl = useDownload();
  const [err, setErr] = useState<string | null>(null);
  const files = list.files.filter((f) => f.path.endsWith('.wav') && f.path !== recPath && !f.path.startsWith('library/'));
  const shown = files.slice(0, limit);
  const days = groupByDay(shown, (f) => stampParts(f.path)?.day ?? '');
  const play = async (f: LogFileEntry) => {
    setErr(null);
    try { await media.play(f); } catch (e) { setErr((e as Error)?.message ?? t('재생하지 못했습니다')); }
  };
  const del = useDeleter('audio', useMemo(() => files.map((f) => ({ key: f.path, paths: [f.path] })), [files]));
  return (
    <>
      <SectionHead title={t('녹음')} desc={t('로봇에 저장된 녹음입니다. 재생은 로봇 스피커로 나갑니다.')}
        right={del.select.on ? del.head : <>
          {del.head}
          <View style={S.loopBox}>
            <Text style={{ color: loop ? c.text : c.muted, fontSize: 11.5 }}>{t('반복 재생')}</Text>
            <Toggle value={loop} onChange={(v) => useMedia.getState().setLoop(v)} />
          </View>
          <RefreshBtn kind="audio" />
        </>} />
      <RecordControl />
      {(list.error || err || dl.err) && <Notice text={(list.error ?? err ?? dl.err)!} />}
      {del.note && <Notice text={del.note} />}
      {del.modal}
      {!list.loaded ? <Loading />
        : files.length === 0
          ? <Empty icon="mic" title={t('아직 녹음한 파일이 없습니다')}
              sub={t('위 카드에서 녹음을 시작하세요. 안내 방송처럼 미리 녹음해 두고 반복 재생할 수 있습니다.')} />
          : days.map((g) => (
            <View key={g.day} style={{ marginBottom: 6 }}>
              <DayHead day={g.day} count={g.items.length} />
              {g.items.map((f) => {
                const st = stampParts(f.path);
                const mine = prefixOf(f.path) === 'uplink';
                const on = playing === f.path;
                const silent = clipSeconds(f.size) < 0.1;
                return (
                  <FileRow key={f.path} active={on} select={del.select} selKey={f.path}
                    icon={mine ? 'mic' : 'headset'} iconTone={on ? c.green : silent ? c.dim : mine ? c.accent2 : c.cyan}
                    title={`${mine ? t('내 마이크') : t('로봇 마이크')} · ${st?.time ?? ''}`}
                    meta={silent ? t('소리 없음 — 녹음 중에 담긴 소리가 없습니다') : `${mmss(clipSeconds(f.size))} · ${humanSize(f.size)}`}>
                    <Pill icon={on ? 'pause' : 'play2'} tone={on ? 'green' : 'plain'} disabled={silent}
                      label={on ? (loop ? t('반복 중 · 정지') : t('재생 중 · 정지')) : t('로봇에서 재생')}
                      onPress={() => play(f)} />
                    <IconBtn icon="download" label={t('내려받기')} busy={dl.busy === f.path} onPress={() => dl.run('audio', f)} />
                  </FileRow>
                );
              })}
            </View>
          ))}
      <MoreBtn shown={shown.length} total={files.length} onMore={() => setLimit((n) => n + PAGE)} />
    </>
  );
}

export function AudioRouteLine() {
  const { c } = useTheme();
  const st = usePtt((s) => s.status);
  useEffect(() => {
    ptt.refreshStatus();
    const id = setInterval(() => ptt.refreshStatus(), 5000);
    return () => clearInterval(id);
  }, []);
  if (!st?.output) return null;
  const none = st.output === 'none';
  const label = st.output === 'ptz' ? t('PTZ 스피커로 재생') : st.output === 'robot' ? t('로봇 스피커로 재생') : t('재생 장치 없음 — 로봇에 스피커가 없습니다');
  return (
    <View style={S.route}>
      <Icon name="headset" size={13} color={none ? c.amberTx : c.greenTx} />
      <Text style={{ color: none ? c.amberTx : c.muted, fontSize: 11.5 }}>{label}</Text>
    </View>
  );
}

export function MediaSounds() {
  const { c } = useTheme();
  const list = useMedia((s) => s.lists.audio);
  const playing = useMedia((s) => s.playingPath);
  const loop = useMedia((s) => s.loop);
  const busy = useMedia((s) => s.soundBusy);
  const noOut = usePtt((s) => s.status?.output === 'none');
  const [err, setErr] = useState<string | null>(null);
  const mine = list.files.filter((f) => f.path.startsWith('library/my-') && f.path.endsWith('.wav'));
  const del = useDeleter('audio', useMemo(() => mine.map((f) => ({ key: f.path, paths: [f.path] })), [list.files]));
  const run = async (fn: () => Promise<unknown>) => {
    setErr(null);
    try { await fn(); } catch (e) { setErr((e as Error)?.message ?? t('재생하지 못했습니다')); }
  };
  const pick = () => run(async () => {
    const r = await pickDocument({ type: 'audio/*', copyToCacheDirectory: true });
    const a = r.assets?.[0];
    if (r.canceled || !a) return;
    const bytes = new Uint8Array(await (await fetch(a.uri)).arrayBuffer());
    await media.uploadUserFile(a.name, bytes);
  });
  const stopRow = playing?.startsWith('library/') && (
    <Pill icon="pause" tone="green" label={t('정지')} onPress={() => run(() => media.stopPlay())} />
  );
  return (
    <>
      <SectionHead title={t('사운드')} desc={t('누르면 로봇에서 재생합니다. 반복 재생을 켜 두면 정지할 때까지 되풀이합니다.')}
        right={<>
          <View style={S.loopBox}>
            <Text style={{ color: loop ? c.text : c.muted, fontSize: 11.5 }}>{t('반복 재생')}</Text>
            <Toggle value={loop} onChange={(v) => useMedia.getState().setLoop(v)} />
          </View>
          {stopRow}
        </>} />
      <AudioRouteLine />
      {(list.error || err) && <Notice text={(err ?? list.error)!} />}
      {SOUND_CATEGORIES.map((cat) => (
        <View key={cat.key} style={{ marginBottom: 12 }}>
          <Text style={[S.catHead, { color: c.dim }]}>{t(cat.label)}</Text>
          <View style={S.soundGrid}>
            {SOUNDS.filter((s) => s.category === cat.key).map((s) => {
              const on = playing === soundPath(s);
              return (
                <Pill key={s.id} icon={on ? 'pause' : 'play2'} tone={on ? 'green' : 'plain'} busy={busy === s.id}
                  disabled={noOut || (!!busy && busy !== s.id)} label={t(s.label)}
                  onPress={() => run(() => media.playSound(s))} />
              );
            })}
          </View>
        </View>
      ))}
      <View style={[S.head, { marginTop: 8, marginBottom: 8, alignItems: 'center' }]}>
        <Text style={{ color: c.text, fontSize: 13, fontWeight: '700', flex: 1 }}>{t('내 사운드')}</Text>
        {del.select.on ? del.head : <>
          {del.head}
          <Pill icon="upload" label={t('파일 올리기')} busy={busy === 'upload'} disabled={!!busy && busy !== 'upload'} onPress={pick} />
        </>}
      </View>
      {del.note && <Notice text={del.note} />}
      {del.modal}
      {mine.length === 0
        ? <Empty icon="upload" title={t('올린 소리가 없습니다')} sub={t('기기에 있는 소리 파일(mp3·wav 등, 10 MB 이하)을 올리면 로봇에 저장되어 언제든 재생할 수 있습니다.')} />
        : mine.map((f) => {
          const on = playing === f.path;
          const name = f.path.replace(/^library\/my-/, '').replace(/\.wav$/, '');
          return (
            <FileRow key={f.path} active={on} icon="play2" iconTone={on ? c.green : c.accent2} title={name} select={del.select} selKey={f.path}
              meta={`${mmss(clipSeconds(f.size))} · ${humanSize(f.size)}`}>
              <Pill icon={on ? 'pause' : 'play2'} tone={on ? 'green' : 'plain'} disabled={noOut}
                label={on ? (loop ? t('반복 중 · 정지') : t('재생 중 · 정지')) : t('로봇에서 재생')}
                onPress={() => run(() => media.play(f))} />
            </FileRow>
          );
        })}
    </>
  );
}

export function SoundQuick() {
  const { c } = useTheme();
  const list = useMedia((s) => s.lists.audio);
  const playing = useMedia((s) => s.playingPath);
  const busy = useMedia((s) => s.soundBusy);
  const recPath = useMedia((s) => s.recordingPath);
  const noOut = usePtt((s) => s.status?.output === 'none');
  const [err, setErr] = useState<string | null>(null);
  useEffect(() => { void media.refresh('audio'); ptt.refreshStatus(); }, []);
  const run = async (fn: () => Promise<unknown>) => {
    setErr(null);
    try { await fn(); } catch (e) { setErr((e as Error)?.message ?? t('재생하지 못했습니다')); }
  };
  const wavs = list.files.filter((f) => f.path.endsWith('.wav') && f.path !== recPath);
  const mine = wavs.filter((f) => f.path.startsWith('library/my-'));
  const recs = wavs.filter((f) => !f.path.startsWith('library/') && clipSeconds(f.size) >= 0.1).slice(0, 8);
  const pill = (key: string, label: string, on: boolean, onPress: () => void, isBusy = false) => (
    <Pill key={key} icon={on ? 'pause' : 'play2'} tone={on ? 'green' : 'plain'} busy={isBusy}
      disabled={noOut || (!!busy && !isBusy)} label={label} onPress={onPress} />
  );
  const head = (title: string) => <Text style={[S.catHead, { color: c.dim, marginTop: 4 }]}>{title}</Text>;
  return (
    <View style={{ gap: 6 }}>
      <View style={{ flexDirection: 'row', alignItems: 'center', gap: 8 }}>
        <Text style={{ flex: 1, color: c.dim, fontSize: 10 }}>{t('누르면 로봇에서 반복 재생합니다')}</Text>
        {playing && <Pill icon="pause" tone="green" label={t('정지')} onPress={() => run(() => media.stopPlay())} />}
      </View>
      {noOut && <Text style={{ color: c.amberTx, fontSize: 10 }}>{t('재생 장치 없음 — 로봇에 스피커가 없습니다')}</Text>}
      {err && <Text style={{ color: c.amberTx, fontSize: 10 }}>{err}</Text>}
      {SOUND_CATEGORIES.map((cat) => (
        <View key={cat.key}>
          {head(t(cat.label))}
          <View style={S.soundGrid}>
            {SOUNDS.filter((s) => s.category === cat.key).map((s) =>
              pill(s.id, t(s.label), playing === soundPath(s), () => run(() => media.playSound(s, true)), busy === s.id))}
          </View>
        </View>
      ))}
      {mine.length > 0 && (
        <View>
          {head(t('내 사운드'))}
          <View style={S.soundGrid}>
            {mine.map((f) => pill(f.path, f.path.replace(/^library\/my-/, '').replace(/\.wav$/, ''), playing === f.path, () => run(() => media.play(f, true))))}
          </View>
        </View>
      )}
      {recs.length > 0 && (
        <View>
          {head(t('최근 녹음'))}
          <View style={S.soundGrid}>
            {recs.map((f) => {
              const st = stampParts(f.path);
              const who = prefixOf(f.path) === 'uplink' ? t('내 마이크') : t('로봇 마이크');
              return pill(f.path, `${who} ${st ? `${dayLabel(st.day)} ${st.time.slice(0, 5)}` : ''}`.trim(), playing === f.path, () => run(() => media.play(f, true)));
            })}
          </View>
        </View>
      )}
    </View>
  );
}

const S = StyleSheet.create({
  center: { position: 'absolute', top: 0, left: 0, right: 0, bottom: 0, alignItems: 'center', justifyContent: 'center' },
  head: { flexDirection: 'row', alignItems: 'flex-start', gap: 12, marginBottom: 16 },
  headRight: { flexDirection: 'row', alignItems: 'center', gap: 8 },
  pill: { flexDirection: 'row', alignItems: 'center', justifyContent: 'center', gap: 6, height: 33, paddingHorizontal: 13, borderRadius: 999, borderWidth: 1 },
  iconBtn: { width: 33, height: 33, alignItems: 'center', justifyContent: 'center', borderWidth: 1 },
  dayHead: { flexDirection: 'row', alignItems: 'center', gap: 7, marginTop: 6, marginBottom: 10 },
  grid: { flexDirection: 'row', flexWrap: 'wrap', gap: 12 },
  empty: { alignItems: 'center', gap: 8, paddingVertical: 36, paddingHorizontal: 20, borderWidth: 1, borderStyle: 'dashed', marginTop: 4 },
  emptyIc: { width: 46, height: 46, alignItems: 'center', justifyContent: 'center', marginBottom: 4 },
  notice: { flexDirection: 'row', alignItems: 'center', gap: 8, paddingHorizontal: 11, paddingVertical: 8, borderWidth: 1, marginBottom: 12 },
  camChip: { position: 'absolute', left: 6, bottom: 6, paddingHorizontal: 6, paddingVertical: 2, borderRadius: 999, backgroundColor: 'rgba(0,0,0,0.55)' },
  capCard: { padding: 7, borderWidth: 1, gap: 7 },
  capFoot: { flexDirection: 'row', alignItems: 'center', gap: 7, paddingHorizontal: 2 },
  badge: { paddingHorizontal: 5, paddingVertical: 1, borderRadius: 999, borderWidth: 1 },
  viewer: { borderWidth: 1, padding: 12, gap: 10 },
  viewerBar: { flexDirection: 'row', alignItems: 'center', gap: 8 },
  row: { flexDirection: 'row', alignItems: 'center', gap: 11, paddingHorizontal: 8, paddingVertical: 8, borderWidth: 1, marginBottom: 2 },
  rowIc: { width: 36, height: 36, alignItems: 'center', justifyContent: 'center' },
  rowActs: { flexDirection: 'row', alignItems: 'center', gap: 6 },
  loopBox: { flexDirection: 'row', alignItems: 'center', gap: 7, marginRight: 4 },
  route: { flexDirection: 'row', alignItems: 'center', gap: 6, marginTop: -8, marginBottom: 14 },
  catHead: { fontSize: 11, fontWeight: '700', marginBottom: 8, letterSpacing: 0.3 },
  soundGrid: { flexDirection: 'row', flexWrap: 'wrap', gap: 8 },
  recCard: { flexDirection: 'row', alignItems: 'center', gap: 14, padding: 14, borderWidth: 1, marginBottom: 16 },
  recBtn: { width: 52, height: 52, borderRadius: 26, borderWidth: 1, alignItems: 'center', justifyContent: 'center' },
  recBtnSm: { width: 36, height: 36, borderRadius: 18, borderWidth: 1, alignItems: 'center', justifyContent: 'center' },
  recDot: { width: 16, height: 16, borderRadius: 8 },
  stopSq: { width: 14, height: 14, borderRadius: 3 },
  check: { width: 20, height: 20, borderRadius: 5, borderWidth: 1.5, alignItems: 'center', justifyContent: 'center' },
  capCheck: { position: 'absolute', top: 12, right: 12, zIndex: 2 },
});

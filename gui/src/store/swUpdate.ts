import { create } from 'zustand';
import type { OtaProgress, OtaHandle } from '@/lib/ota';
import type { FwDownload, FwDownloaded, FwPart, FwRelease } from '@/lib/firmwareRelease';
import { t } from '@/lib/i18n';
import { useRobot } from './robot';
import type { JobSource } from './jobs';

type Bytes = { written: number; total: number; part?: FwPart };
export interface SwJobs {
  tgt: 'motion' | 'vision';
  file: FwDownloaded | null;
  sigFor: string | null;
  prog: OtaProgress | null;
  sendingOpt: boolean;
  sendName: string;
  sendTo: string;
  err: string | null;
  doneMsg: boolean;
  askReboot: boolean;
  rels: FwRelease[] | null;
  dlTag: string | null;
  dlProg: Bytes | null;
  rowBusy: string | null;
  rowDl: boolean;
  rowProg: Bytes | null;
  appBusy: boolean;
  appProg: Bytes | null;
  appMsg: string | null;
  appErr: string | null;
  open: number;
  unseen: 'send' | 'download' | 'app' | null;
}

const INIT: SwJobs = {
  tgt: 'motion', file: null, sigFor: null, prog: null, sendingOpt: false, sendName: '', sendTo: '',
  err: null, doneMsg: false, askReboot: false, rels: null, dlTag: null, dlProg: null,
  rowBusy: null, rowDl: false, rowProg: null, appBusy: false, appProg: null, appMsg: null, appErr: null,
  open: 0, unseen: null,
};
export const useSwJobs = create<SwJobs>(() => ({ ...INIT }));

const setters = new Map<keyof SwJobs, (v: never) => void>();
export function useSw<K extends keyof SwJobs>(k: K): [SwJobs[K], (v: SwJobs[K]) => void] {
  let f = setters.get(k);
  if (!f) { f = (v: never) => useSwJobs.setState({ [k]: v } as Partial<SwJobs>); setters.set(k, f); }
  return [useSwJobs((s) => s[k]), f as (v: SwJobs[K]) => void];
}

export const swRefs = {
  handle: { current: null as OtaHandle | null },
  dl: { current: null as FwDownload | null },
  appAbort: { current: null as AbortController | null },
};

const sending = (s: SwJobs) => s.sendingOpt || (!!s.prog && s.prog.phase !== 'complete');
const downloading = (s: SwJobs) => !!s.dlTag || s.rowDl;
useSwJobs.subscribe((s, p) => {
  if (s.open > 0) return;
  const ended = sending(p) && !sending(s) ? 'send' : downloading(p) && !downloading(s) ? 'download'
    : p.appBusy && !s.appBusy ? 'app' : null;
  if (ended) useSwJobs.setState({ unseen: ended });
});

useRobot.subscribe((r, p) => {
  const s = useSwJobs.getState();
  if (r.ip === p.ip || sending(s) || downloading(s)) return;
  useSwJobs.setState({ askReboot: false, doneMsg: false, unseen: null, err: null, file: null, sigFor: null });
});

export const stillOn = (startIp: string) => useRobot.getState().ip === startIp;

export const pkgVersion = (name: string) =>
  /^(?:RBQ-(.+)\.(?:tar\.gz|zip)|core-(v.+)-u\d+-\w+\.rbq)$/.exec(name)?.slice(1).find(Boolean);

const mb = (b: number) => `${(b / 1048576).toFixed(0)} MB`;
export type SwBanner = { text: string; tone: 'run' | 'ok' | 'bad'; closable: boolean };
export function swBanner(s: SwJobs): SwBanner | null {
  if (s.open > 0) return null;
  const what = `${t('로봇 소프트웨어')}${s.file ? ` ${pkgVersion(s.file.name) ?? ''}` : ''}`.trim();
  if (sending(s)) {
    const p = s.prog;
    const text = p?.phase === 'applying' ? t('로봇이 소프트웨어를 설치하는 중 — 끄지 마세요')
      : `${what} ${s.sendingOpt ? t('옵션 먼저 보내는 중') : t('보내는 중')}`
        + (p?.phase === 'transfer' ? ` ${p.percentage}%` : p?.phase === 'reconnecting' ? ` — ${t('다시 연결 중')}` : '');
    return { text, tone: 'run', closable: false };
  }
  if (downloading(s)) {
    const b = s.dlProg ?? s.rowProg;
    return { text: `${t('로봇 소프트웨어')} ${t('받는 중')}${b && b.total > 0 ? ` ${mb(b.written)} / ${mb(b.total)}` : ''}`, tone: 'run', closable: false };
  }
  if (s.appBusy) {
    const b = s.appProg;
    return { text: `${t('앱 업데이트')} ${t('받는 중')}${b && b.total > 0 ? ` ${mb(b.written)} / ${mb(b.total)}` : ''}`, tone: 'run', closable: false };
  }
  if (s.unseen === 'send') {
    if (s.err) return { text: `${what} ${t('업데이트 실패')} — ${s.err}`, tone: 'bad', closable: true };
    if (s.doneMsg) return { text: `${what} ${t('전송 완료 — 눌러서 결과를 보고 재부팅하세요')}`, tone: 'ok', closable: true };
  }
  if (s.unseen === 'download') {
    if (s.err) return { text: `${t('로봇 소프트웨어')} ${t('받기 실패')} — ${s.err}`, tone: 'bad', closable: true };
    return { text: `${t('로봇 소프트웨어')} ${t('받음 — 눌러서 로봇에 보내세요')}`, tone: 'ok', closable: true };
  }
  if (s.unseen === 'app') {
    if (s.appErr) return { text: `${t('앱 업데이트')} ${t('업데이트 실패')} — ${s.appErr}`, tone: 'bad', closable: true };
    if (s.appMsg) return { text: `${t('앱 업데이트')} — ${s.appMsg}`, tone: 'ok', closable: true };
  }
  return null;
}

export const swJob: JobSource = {
  key: 'sw', route: '/update?tab=sw', icon: 'download', label: () => t('소프트웨어 업데이트 열기'),
  useLine: () => swBanner(useSwJobs()),
  dismiss: () => useSwJobs.setState({ unseen: null }),
};

import { screenState, jobLine, type JobSource } from './jobs';
import { useRobot } from './robot';
import { useTelemetry } from './telemetry';
import { swRefs, useSwJobs } from './swUpdate';
import { optionRefs } from './optionInstall';
import { otaTransfer, otaProbe, type OtaHandle, type OtaProgress } from '@/lib/ota';
import { rest, actions } from '@/lib/rest';
import { connection } from '@/lib/connection';
import { readFileAt } from '@/lib/fileStream';
import { parseSidecars } from '@/lib/firmwareRelease';
import { fwStopped } from '@/lib/boardFirmware';
import { readBundle, installBundle, type Bundle, type BundleDeps, type RobotNow, type UsbStep } from '@/lib/usbBundle';
import { t } from '@/lib/i18n';

export interface PickedFile { uri: string; name: string; size: number }

export const useUsbJob = screenState({
  picked: null as { file: PickedFile; bundle: Bundle } | null,
  reading: false,
  label: '',
  option: '',
  step: null as UsbStep | null,
  prog: null as OtaProgress | null,
  err: null as string | null,
  done: null as string | null,
}, (s) => s.step != null);

export const usbRefs = { handle: { current: null as OtaHandle | null } };

useRobot.subscribe((r, p) => {
  if (r.ip === p.ip || useUsbJob.getState().step) return;
  useUsbJob.setState({ picked: null, done: null, err: null, unseen: false });
});

type S = ReturnType<typeof useUsbJob.getState>;
export function usbRun(s: S): string | null {
  const p = s.prog;
  const how = p?.phase === 'applying' ? t('로봇이 설치하는 중 — 끄지 마세요')
    : p?.phase === 'transfer' ? `${t('보내는 중')} ${p.percentage}%`
    : p?.phase === 'reconnecting' ? t('다시 연결 중') : t('연결 중…');
  switch (s.step) {
    case 'check': return t('로봇 확인 중');
    case 'restart': return t('로봇 소프트웨어를 다시 켜고 연결을 기다리는 중');
    case 'legacy': return `${t('구형 꾸러미')} ${how}`;
    case 'core': return `${t('본체')} ${how}`;
    case 'option': return `${s.option.toUpperCase()} ${t('옵션')} ${how}`;
    default: return null;
  }
}
export function usbLine(s: S) {
  const what = `${t('USB 묶음')} ${s.label}`;
  const run = usbRun(s);
  return jobLine(s, { run: run && `${what} — ${run}`, bad: s.err ? `${what} — ${s.err}` : null, ok: s.done });
}

export const usbJob: JobSource = {
  key: 'usb', route: '/update?tab=sw', icon: 'download', label: () => t('소프트웨어 업데이트 열기'),
  useLine: () => usbLine(useUsbJob()),
  dismiss: () => useUsbJob.setState({ unseen: false }),
};

export function waitBack(ip: string, since: number, ms: number): Promise<void> {
  return new Promise((resolve, reject) => {
    let ended = false;
    let off = () => {};
    const end = (e?: Error) => {
      if (ended) return;
      ended = true;
      clearTimeout(timer);
      off();
      if (e) reject(e); else resolve();
    };
    const timer = setTimeout(() => end(new Error(t('로봇이 다시 연결되지 않았습니다 — 로봇을 확인하고 묶음을 다시 고르면 깔린 단계는 건너뜁니다'))), ms);
    const check = () => {
      const r = useRobot.getState();
      if (r.ip !== ip) end(new Error(t('로봇이 바뀌어 묶음 설치를 멈췄습니다')));
      else if (r.conn === 'connected' && (r.droppedAt ?? 0) >= since) end();
    };
    off = useRobot.subscribe(check);
    check();
  });
}

const statusOf = (e: unknown) => (e as { status?: number } | null)?.status;

export async function retry<T>(f: () => Promise<T>, tries = 5, gapMs = 2000): Promise<T> {
  for (let i = 1; ; i++) {
    try { return await f(); } catch (e) {
      if (i >= tries) throw e;
      await new Promise((r) => setTimeout(r, gapMs));
    }
  }
}

export async function robotNow(ip: string): Promise<RobotNow> {
  const v = await retry(() => rest.version(ip));
  const probe = v.system ? null
    : await retry(() => otaProbe(ip).then((p) => p ?? Promise.reject(new Error('no OTA answer')))).catch(() => null);
  const f = (await retry(() => rest.robotFeatures(ip).catch((e) => (statusOf(e) === 404 ? null : Promise.reject(e)))))?.features;
  return {
    system: v.system || probe?.system,
    legacy: !v.system && !!probe && !probe.system,
    version: v.version,
    option: typeof f?.option === 'string' ? f.option : '',
    optionVersion: f?.option_version != null ? String(f.option_version) : '',
  };
}

function depsFor(ip: string, file: PickedFile): BundleDeps {
  const set = useUsbJob.setState;
  return {
    robot: () => robotNow(ip),
    signature: async (p) => {
      const sig = parseSidecars(null, new TextDecoder().decode(await readFileAt(file.uri, p.offset, p.size))).signature;
      if (!sig) throw new Error(`${t('묶음 안의 서명 파일이 맞지 않습니다 — 다시 받으세요')}: ${p.name}`);
      return sig;
    },
    send: (f) => {
      set({ prog: null });
      const h = otaTransfer(ip, { uri: file.uri, ...f }, (prog) => set({ prog }));
      usbRefs.handle.current = h;
      return h.done.finally(() => { usbRefs.handle.current = null; set({ prog: null }); });
    },
    own: async () => {
      const o = await retry(() => rest.getOwnership(ip));
      if (o.IsOwner) return;
      if (o.ownerIP) throw new Error(t('다른 기기가 제어권을 가져가 로봇 소프트웨어를 다시 시작하지 못했습니다'));
      await retry(() => connection.claimOwnership().then((ok) => ok || Promise.reject(new Error(t('제어권을 다시 가져오지 못했습니다')))));
    },
    restart: () => actions.restart(ip),
    reboot: () => actions.reboot(ip),
    standing: () => { const r = useTelemetry.getState().robot; return !!r && (r.isStanding || !fwStopped(r.gaitId, false)); },
    waitBack: (since, ms) => waitBack(ip, since, ms),
    step: (step) => set({ step }),
    now: () => Date.now(),
  };
}

export function openUsbBundle(file: PickedFile) {
  if (useUsbJob.getState().step) return;
  useUsbJob.setState({ reading: true, picked: null, err: null, done: null });
  readBundle((offset, length) => readFileAt(file.uri, offset, length), file.size)
    .then((bundle) => useUsbJob.setState({ picked: { file, bundle } }))
    .catch((e: Error) => useUsbJob.setState({ err: `${file.name} — ${e.message}` }))
    .finally(() => useUsbJob.setState({ reading: false }));
}

export function startUsbBundle(ip: string) {
  const s = useUsbJob.getState();
  if (!s.picked || s.step) return;
  const { file, bundle } = s.picked;
  if (useSwJobs.getState().tgt !== 'motion') {
    useUsbJob.setState({ picked: null, err: t('묶음 설치는 Motion 대상에서만 됩니다 — 대상을 Motion 으로 바꾸세요') });
    return;
  }
  if (swRefs.handle.current || optionRefs.handle.current) {
    useUsbJob.setState({ picked: null, err: t('다른 전송이 도는 중입니다 — 끝난 뒤 다시 고르세요') });
    return;
  }
  useUsbJob.setState({ picked: null, err: null, done: null, prog: null, step: 'check',
    label: bundle.manifest.label, option: bundle.manifest.options[0]?.option ?? '' });
  installBundle(bundle, depsFor(ip, file))
    .then((done) => useUsbJob.setState({ done }))
    .catch((e: Error) => useUsbJob.setState({ err: e.message === 'cancelled' ? null : e.message }))
    .finally(() => useUsbJob.setState({ step: null, prog: null }));
}

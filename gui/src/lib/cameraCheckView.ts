import type { CameraCheckSensor, CameraCheckVerdict, CameraStatus, CameraStreamKey, DepthCheckSensor } from '@/lib/rest';
import { t } from '@/lib/i18n';

export const CAMERA_NAMES = ['Bottom0', 'Bottom1', 'Bottom2', 'Bottom3', 'Front', 'Rear'] as const;
export const cameraName = (id: number) => CAMERA_NAMES[id] ?? `#${id}`;

export const CHECK_VIEW_COLS = 3;
export const CHECK_VIEW_ROWS = 2;

export const STREAM_KEYS: readonly CameraStreamKey[] = ['rgb', 'ir', 'depth'];

const RANK: Record<CameraCheckVerdict, number> = { skip: 0, pass: 1, fail: 2 };

export function worstVerdict(sensors: CameraCheckSensor[]): CameraCheckVerdict {
  let worst: CameraCheckVerdict = 'pass';
  for (const s of sensors) if (s.verdict && RANK[s.verdict] > RANK[worst]) worst = s.verdict;
  return worst;
}

export const verdictLabel = (v: CameraCheckVerdict) =>
  v === 'pass' ? t('정상') : v === 'fail' ? t('점검 필요') : t('꺼짐');

const STREAM_LABEL: Record<string, string> = { rgb: 'RGB', ir: 'IR', depth: 'Depth' };
export const streamLabel = (k: string) => STREAM_LABEL[k] ?? k;

export const usbLabel = (usb: number | null | undefined) => (usb === 3 ? '3.0' : usb === 2 ? '2.0' : '');

export type CellTone = 'ok' | 'warn' | 'fail' | 'off';

export function statusCell(s: CameraStatus): { text: string; tone: CellTone } {
  if (!s.enabled) return { text: t('꺼짐'), tone: 'off' };
  if (s.running) return { text: 'RUN', tone: 'ok' };
  if (s.sleep) return { text: 'SLEEP', tone: 'off' };
  if (s.idle) return { text: 'IDLE', tone: 'warn' };
  if (!s.detected) return { text: t('미감지'), tone: 'warn' };
  if (!s.powered) return { text: t('전원 꺼짐'), tone: 'fail' };
  if (!s.connected) return { text: t('미연결'), tone: 'warn' };
  return { text: t('대기'), tone: 'warn' };
}

export const emitCell = (s: CameraStatus): string => (!s.projector ? 'N' : s.projector_on ? 'ON' : 'OFF');

export function fpsCell(fps: number | undefined): { text: string; tone: CellTone } {
  if (fps == null) return { text: 'N', tone: 'off' };
  return { text: String(fps), tone: fps === 0 ? 'fail' : 'ok' };
}

export function defaultView(available: CameraStreamKey[]): CameraStreamKey | 'auto' {
  return STREAM_KEYS.find((k) => available.includes(k)) ?? 'auto';
}

export function reasonText(s: CameraCheckSensor): string {
  const streams = s.reason_streams.map(streamLabel).join('·');
  switch (s.reason) {
    case 'ok': return t('정상');
    case 'disabled': return t('설정에서 꺼진 카메라입니다');
    case 'sleeping': return 'AlwaysON = false';
    case 'not_opened': return t('카메라를 열지 못했습니다 — 카메라 점검이 필요합니다');
    case 'not_detected': return t('카메라가 인식되지 않습니다 — 케이블 연결을 확인하세요');
    case 'device_error': return t('카메라 오류(코드 {n}) — 카메라 점검이 필요합니다').replace('{n}', String(s.status.error_id));
    case 'stream_missing': return t('{s} 영상이 나오지 않습니다 — 카메라 점검이 필요합니다').replace('{s}', streams);
    case 'frame_drop': return (s.usb === 2
      ? t('{s} 프레임이 자주 끊깁니다 — USB2 로 연결돼 있어 케이블·포트 점검이 필요합니다')
      : t('{s} 프레임이 자주 끊깁니다 — 카메라 점검이 필요합니다')).replace('{s}', streams);
    default: return '';
  }
}

export const usbNote = (s: CameraCheckSensor): string =>
  s.usb === 2 && s.reason !== 'frame_drop' ? t('USB2 로 연결돼 있습니다 — 케이블·포트 점검을 권장합니다') : '';

export const failText = (error: string | null) =>
  error === 'hal_unavailable' ? t('카메라 상태를 받지 못했습니다 — HAL 이 실행 중인지 확인하세요') : t('측정에 실패했습니다');

export const depthNow = (s: DepthCheckSensor) => ((s.applied || s.occ === 'saved') && s.after?.frames ? s.after : s.measured);

export function occCell(s: DepthCheckSensor): { text: string; tone: CellTone } {
  if (s.occ === 'saved') return { text: t('보정 저장'), tone: 'ok' };
  if (s.occ === 'applied') return { text: t('보정 적용'), tone: 'ok' };
  if (s.occ === 'kept') return { text: t('기존 보정 유지'), tone: 'off' };
  if (s.occ === 'not_streaming') return { text: t('영상 없음'), tone: 'off' };
  if (s.occ === 'timeout') return { text: t('응답 없음'), tone: 'fail' };
  if (s.occ && s.occ !== 'running') return { text: t('실패'), tone: 'fail' };
  return { text: '—', tone: 'off' };
}

export function depthVerdict(s: DepthCheckSensor): { text: string; tone: CellTone } {
  if (s.result === 'running') return { text: t('측정 중'), tone: 'off' };
  if (s.result === 'not_streaming') return { text: t('영상 없음'), tone: 'off' };
  if (s.result !== 'ok') return { text: s.result === 'timeout' ? t('응답 없음') : t('실패'), tone: 'fail' };
  if (!depthNow(s)?.frames) return { text: t('평면 없음'), tone: 'warn' };
  if (s.low_fill) return { text: t('시야 확인'), tone: 'warn' };
  return s.needs_occ ? { text: t('OCC 권장'), tone: 'warn' } : { text: t('정상'), tone: 'ok' };
}

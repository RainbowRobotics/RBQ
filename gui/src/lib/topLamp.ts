import type { Rgb } from '@/lib/ledBottom';

export const LAMP_TEST_MS = 1600;
export const LAMP_MSG_HOLD_MS = 4000;
export const LAMP_ROWS: number[] = [4, 3];

const RED: Rgb = [255, 0, 0];
const GREEN: Rgb = [0, 255, 0];
const BLUE: Rgb = [0, 0, 255];

export function lampColorAt(elapsedMs: number): Rgb | null {
  if (!(elapsedMs >= 0) || elapsedMs >= LAMP_TEST_MS) return null;
  if (elapsedMs < LAMP_TEST_MS / 3) return RED;
  if (elapsedMs < (2 * LAMP_TEST_MS) / 3) return GREEN;
  return BLUE;
}

export function lampErrorText(status: number): string | null {
  if (status === 404) return '로봇 소프트웨어에 이 기능이 없습니다. 업데이트가 필요합니다';
  if (status === 409) return 'CAN FD 로봇에서만 할 수 있습니다';
  if (status === 0) return '보내지 못했습니다. 연결을 확인해 주세요';
  return null;
}

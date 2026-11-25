import { INIT_STATE, INIT_STEPS, type AutostartInfo } from '@/types/robot';
import { t } from '@/lib/i18n';

export const LEGS = [
  { cap: 'HR', name: '후방-우측', base: 0 },
  { cap: 'HL', name: '후방-좌측', base: 3 },
  { cap: 'FR', name: '전방-우측', base: 6 },
  { cap: 'FL', name: '전방-좌측', base: 9 },
] as const;
export const AX = ['R', 'P', 'K'] as const;
const WHEELS = [
  { name: 'HRW', idx: 12 }, { name: 'HLW', idx: 13 },
  { name: 'FRW', idx: 14 }, { name: 'FLW', idx: 15 },
] as const;

export function jointName(i: number): string {
  if (i < 0) return '';
  if (i >= 12) return WHEELS[i - 12] ? `${WHEELS[i - 12].name}${i}` : `CH${i}`;
  return `${LEGS[Math.floor(i / 3)].cap}${AX[i % 3]}${i}`;
}

export function rateText(kbps: number | undefined): string {
  if (!kbps) return '—';
  if (kbps === 0xFFFF) return '?';
  return kbps % 1000 === 0 ? `${kbps / 1000}M` : `${(kbps / 1000).toFixed(2)}M`;
}

export function failReason(a: AutostartInfo): { title: string; hint: string } | null {
  if (!a.steps[a.step] || a.steps[a.step] !== INIT_STATE.fail) return null;
  const ch = a.fail_ch >= 0 ? jointName(a.fail_ch) : '';
  switch (INIT_STEPS[a.step]) {
    case 'precheck':
      return { title: t('이미 서 있는 상태입니다'),
               hint: t('로봇을 앉힌 뒤 다시 시작하세요.') };
    case 'leg_power':
      return { title: t('다리 48V 전원이 올라오지 않습니다'),
               hint: t('배터리 잔량과 퓨즈 상태를 확인하세요.') };
    case 'data_rate': {
      const k = a.fail_ch >= 0 ? (a.ch_rate_kbps?.[a.fail_ch] ?? 0) : 0;
      if (k !== 3000 && k !== 4000) {
        return { title: `${ch} ${t('보드의 데이터 속도가 3M · 4M 이 아닙니다')} (${rateText(k)})`,
                 hint: t('모터 보드 CAN 설정(0xCE)을 3M 으로 다시 쓴 뒤 다시 시작하세요.') };
      }
      if ((a.rate_rewritten ?? 0) > 0 && k !== 4000) {
        return { title: `${ch} ${t('보드를 4M 으로 고쳐 썼지만 그대로 {r} 입니다').replace('{r}', rateText(k))}`,
                 hint: t('해당 보드의 펌웨어와 EEPROM 쓰기를 확인한 뒤 다시 시작하세요.') };
      }
      return { title: t('로봇 PC 다리 버스를 {r} 으로 올리지 못했습니다').replace('{r}', rateText(k)),
               hint: t('로봇 PC CAN 카드 클럭을 60 MHz 로 바꾸지 못했을 수 있습니다. Motion 로그의 [CAN] 줄을 확인하세요.') };
    }
    case 'comm':
      return { title: `${ch} ${t('보드가 응답하지 않습니다')}`,
               hint: t('해당 다리의 전원과 CAN 커넥터를 확인한 뒤 다시 시작하세요.') };
    case 'param':
      return { title: `${ch} ${t('모터 설정값이 기대값과 다릅니다')}`,
               hint: t('보드 교체나 펌웨어 갱신 직후라면 토크상수·감속비 설정을 확인하세요.') };
    case 'homing': {
      const bad = a.ch_home
        .map((s, i) => (s === INIT_STATE.fail ? i : -1))
        .filter((i) => i >= 0);
      if (!bad.length) {
        return { title: t('홈 위치를 확인하지 못했습니다'),
                 hint: t('요청된 홈 자세 값이 올바르지 않습니다. 로봇 로그를 확인하세요.') };
      }
      return {
        title: `${bad.map(jointName).join(', ')} ${t('관절이 홈 범위를 벗어났습니다')}`,
        hint: t('로봇을 평평한 바닥에 내려 네 다리를 접은 자세로 만든 뒤 다시 시작하세요.'),
      };
    }
    case 'imu':
      return { title: t('IMU가 연결되지 않았습니다'),
               hint: t('IF 보드 연결을 확인하세요.') };
    case 'control':
      return { title: `${ch} ${t('모터가 켜지지 않습니다')}`,
               hint: t('해당 관절의 보드 상태와 CAN 연결을 확인한 뒤 다시 시작하세요.') };
    default:
      return { title: t('기동에 실패했습니다'), hint: '' };
  }
}

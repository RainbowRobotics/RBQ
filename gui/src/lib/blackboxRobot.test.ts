import { describe, it, expect, vi } from 'vitest';

vi.mock('@/lib/rest', () => ({ rest: {} }));
import { buildSession, serialFromZipName } from './blackbox';

const DATA = 'joint.pos[0]\tstatus.is_fall\n0.5\t0\n0.6\t1\n';
const base = { dataTxt: DATA, logTxt: '', videoTxt: '', session: '08_32_18', date: '20260911' };

describe('serialFromZipName — 새 이름/옛 이름', () => {
  it('S/N 이 든 새 이름에서 뽑는다', () => {
    expect(serialFromZipName('blackbox-RBQ0007-20260911_08_32_18.zip')).toBe('RBQ0007');
    expect(serialFromZipName('/sdcard/Download/blackbox-RBQ0007-20260911_08_32_18.zip')).toBe('RBQ0007');
  });
  it('S/N 없이 저장된 옛 이름은 null — 폴더명을 S/N 으로 오인하지 않는다', () => {
    expect(serialFromZipName('blackbox-20260911_08_32_18.zip')).toBeNull();
  });
  it('전혀 다른 이름도 null', () => {
    expect(serialFromZipName('무언가.zip')).toBeNull();
    expect(serialFromZipName('')).toBeNull();
  });
});

describe('세션 신원 우선순위', () => {
  const meta = JSON.stringify({ data_start_epoch_ms: 1789083108692, robot_serial: 'FROM-META' });

  it('meta.json 에 있으면 그것이 이긴다 — 로봇을 바꿔 붙여도 세션은 제 주인을 안다', () => {
    const s = buildSession({ ...base, metaTxt: meta,
      robotFallback: { serial: 'FROM-CONN', via: 'connection' } });
    expect(s.robot).toEqual({ serial: 'FROM-META', via: 'meta' });
  });

  it('meta 에 없으면 폴백을 쓴다(옛 세션)', () => {
    const noSn = JSON.stringify({ data_start_epoch_ms: 1789083108692 });
    const s = buildSession({ ...base, metaTxt: noSn,
      robotFallback: { serial: 'FROM-CONN', via: 'connection' } });
    expect(s.robot).toEqual({ serial: 'FROM-CONN', via: 'connection' });
  });

  it('meta.json 자체가 없어도 폴백으로 산다 — 세션 로드는 그대로 된다', () => {
    const s = buildSession({ ...base, metaTxt: '',
      robotFallback: { serial: 'FROM-NAME', via: 'filename' } });
    expect(s.robot).toEqual({ serial: 'FROM-NAME', via: 'filename' });
    expect(s.frameCount).toBe(2);
  });

  it('아무 단서도 없으면 미상 — 임의로 채우지 않는다', () => {
    const s = buildSession({ ...base, metaTxt: '' });
    expect(s.robot).toBeUndefined();
    expect(s.frameCount).toBe(2);
  });

  it('빈 문자열 시리얼은 신원이 아니다', () => {
    const blank = JSON.stringify({ data_start_epoch_ms: 1, robot_serial: '   ' });
    expect(buildSession({ ...base, metaTxt: blank }).robot).toBeUndefined();
  });
});

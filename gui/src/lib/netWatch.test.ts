import { describe, it, expect, vi } from 'vitest';

function makeWatcher(reconnect: () => void, demo = false) {
  let fp: string | null = null;
  return (next: string) => {
    if (next === fp) return;
    const first = fp === null;
    fp = next;
    if (first || demo) return;
    reconnect();
  };
}

describe('netWatch 지문', () => {
  it('첫 표본은 기준만 잡고 재연결하지 않는다', () => {
    const r = vi.fn();
    makeWatcher(r)('wlan0:192.168.0.21/255.255.255.0');
    expect(r).not.toHaveBeenCalled();
  });

  it('같은 망이 계속 보고돼도 아무것도 하지 않는다', () => {
    const r = vi.fn();
    const w = makeWatcher(r);
    const id = 'wlan0:192.168.0.21/255.255.255.0';
    w(id); w(id); w(id); w(id);
    expect(r).not.toHaveBeenCalled();
  });

  it('망이 바뀌면 즉시 재연결한다', () => {
    const r = vi.fn();
    const w = makeWatcher(r);
    w('wlan0:192.168.0.21/255.255.255.0');
    w('wlan0:10.20.30.134/255.255.255.0');
    expect(r).toHaveBeenCalledTimes(1);
  });

  it('바뀐 뒤 그 망에 머무르면 다시 부르지 않는다', () => {
    const r = vi.fn();
    const w = makeWatcher(r);
    w('a'); w('b'); w('b'); w('b');
    expect(r).toHaveBeenCalledTimes(1);
  });

  it('데모 모드에서는 재연결하지 않는다 — 가짜 루프를 끊으면 화면이 죽는다', () => {
    const r = vi.fn();
    const w = makeWatcher(r, true);
    w('a'); w('b');
    expect(r).not.toHaveBeenCalled();
  });
});

function robotFacing(id: string, robotIp: string) {
  const net = robotIp ? robotIp.split('.').slice(0, 3).join('.') : '';
  if (!net) return id;
  return id.split(',').filter((e) => e.split(':')[1]?.startsWith(`${net}.`)).join(',');
}

describe('robotFacing — 로봇에 닿는 주소만', () => {
  const ID = 'enp4s0:10.20.30.134/255.255.255.0,wlan0:192.168.0.11/255.255.255.0';

  it('로봇 망 항목만 남긴다', () => {
    expect(robotFacing(ID, '192.168.0.10')).toBe('wlan0:192.168.0.11/255.255.255.0');
  });

  it('⚠무관한 NIC 가 사라져도 지문이 그대로다 — 주행 중 제어를 끊지 않는다', () => {
    const before = robotFacing(ID, '192.168.0.10');
    const after = robotFacing('wlan0:192.168.0.11/255.255.255.0', '192.168.0.10');
    expect(after).toBe(before);
  });

  it('로봇 망을 벗어나면 빈 값이 되어 변화로 잡힌다', () => {
    const before = robotFacing(ID, '192.168.0.10');
    const after = robotFacing('wlan0:192.168.99.5/255.255.255.0', '192.168.0.10');
    expect(after).not.toBe(before);
    expect(after).toBe('');
  });

  it('대상 IP 를 모르면 전체를 본다 — 좁힐 근거가 없다', () => {
    expect(robotFacing(ID, '')).toBe(ID);
  });
});

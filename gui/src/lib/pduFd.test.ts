import { describe, it, expect } from 'vitest';
import { FD_PORT, FD_PORTS, normalizePduFd, batteryFaults, SIDE_HUBS, SIDE_HUB_RESULT, sideHubRaw } from './pduFd';

const RESP = {
  pdu_fd: {
    port_out: [
      { index: 0, name: 'LAN_12V_1', state: 1, voltage: 12.1, current: 0.3 },
      { index: 1, name: 'LAN_12V_2', state: -1, voltage: 0, current: 0 },
      { index: 2, name: 'LAN_12V_3', state: 0 },
    ],
    port_in: [{ index: 0, name: 'EMO', state: 0, voltage: 0, current: 0 }],
    battery: [{ index: 0, name: 'LEFT', voltage: 52.1, current: -3.2, soc: 78, detect: true, uv: true }],
    temperature: [{ index: 0, name: 'BAT_LEFT', value: 31 }],
  },
  timestamp: '2026-09-15T10:00:00Z', status: 'ok',
};

describe('normalizePduFd', () => {
  const s = normalizePduFd(RESP);
  it('port_out 상태 1/0/-1 그대로, 누락 수치는 0', () => {
    expect(s.portOut.map((p) => p.state)).toEqual([1, -1, 0]);
    expect(s.portOut[2].voltage).toBe(0);
    expect(s.portOut[0].voltage).toBeCloseTo(12.1);
  });
  it('배터리 플래그 — 켜진 것만 이름으로', () => {
    expect(s.battery[0].soc).toBe(78);
    expect(batteryFaults(s.battery[0])).toEqual(['UV']);
  });
  it('배열 누락·비객체 응답은 빈 배열(화면이 죽지 않음)', () => {
    expect(normalizePduFd(undefined)).toEqual({ portOut: [], portIn: [], battery: [], temperature: [], sideHub: [] });
    expect(normalizePduFd({ pdu_fd: { port_out: 'x' as unknown as [] } }).portOut).toEqual([]);
  });
});

describe('side_hub — 사이드 보드 USB 허브 리셋 결과', () => {
  it('칸 그대로, 옛 데몬(칸 없음)은 빈 배열', () => {
    const s = normalizePduFd({ pdu_fd: { port_out: [], side_hub: [
      { side: 'front', seq: 3, running: false, result: 0, cfg: 0, rst: 1, vid: 0x0451, pid: 0x8440 },
      { side: 'hind', seq: 1, running: true, result: 2 },
    ] } });
    expect(s.sideHub[0]).toEqual({ side: 'front', seq: 3, running: false, result: 0, cfg: 0, rst: 1, vid: 0x0451, pid: 0x8440 });
    expect(s.sideHub[1]).toMatchObject({ side: 'hind', seq: 1, running: true, result: 2, vid: 0 });
    expect(sideHubRaw(s.sideHub[0])).toBe('rst 01 · cfg 00 · 0451:8440');
    expect(normalizePduFd({ pdu_fd: { port_out: [] } }).sideHub).toEqual([]);
  });
  it('결과 문구는 rcl SIDE_HUB_OUTCOME 0..9 전부, 행은 전방 · 후방', () => {
    for (let r = 0; r <= 9; r++) expect(SIDE_HUB_RESULT[r]).toBeTruthy();
    expect(SIDE_HUBS.map((h) => h.side)).toEqual(['front', 'hind']);
  });
});

describe('FD_PORTS 표 불변식', () => {
  it('인덱스는 유일하고 이름과 FD_PORT 값이 일치', () => {
    const idx = FD_PORTS.map((p) => p.index);
    expect(new Set(idx).size).toBe(idx.length);
    for (const p of FD_PORTS) expect(FD_PORT[p.name]).toBe(p.index);
  });
  it('PortOutIndex 19개 중 TBD_12V(P5, 미확인)만 빠지고 PDU_48V 만 읽기 전용', () => {
    expect(FD_PORT.NO_PORT_OUT).toBe(19);
    expect(FD_PORTS.length).toBe(18);
    expect(FD_PORTS.map((p) => p.name)).not.toContain('TBD_12V');
    expect(FD_PORTS.filter((p) => p.group === 'top' && p.nm.startsWith('12V 포트')).map((p) => p.nm)).toEqual(['12V 포트 1', '12V 포트 2', '12V 포트 3', '12V 포트 4']);
    expect(FD_PORTS.filter((p) => p.nm.startsWith('12V 포트')).every((p) => p.askOff)).toBe(true);
    expect(FD_PORTS.filter((p) => p.readOnly).map((p) => p.name)).toEqual(['PDU_48V']);
  });
});

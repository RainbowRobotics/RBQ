import { describe, it, expect, beforeEach, vi } from 'vitest';

vi.mock('./logUploadCommon', () => ({ SB_URL: 'https://x.supabase.co', SB_KEY: 'anon' }));

let liveRobotId = '';
vi.mock('./remoteIce', () => ({ robotId: () => liveRobotId }));

const issued: { pin: string; serial: string }[] = [];
let nextResult: unknown = null;

vi.mock('./remoteLogin', () => ({
  issueTicket: async (pin: string, serial: string) => {
    issued.push({ pin, serial });
    return nextResult;
  },
}));

const { currentTicket, setTicketSession, clearTicketSession, ticketExpiry } = await import('./connectTicket');

const ok = (t: string, minutes: number) => ({
  ok: true as const,
  ticket: { ticket: t, robotId: liveRobotId, expiresAt: new Date(Date.now() + minutes * 60_000).toISOString() },
});

const ROBOTS = [
  { robotId: 'RBQ103', robotSerial: 'RBQ103' },
  { robotId: 'RBQ105', robotSerial: 'RBQ105' },
];

beforeEach(() => { issued.length = 0; clearTicketSession(); liveRobotId = 'RBQ103'; });

describe('currentTicket', () => {
  it('로그인 안 했으면 빈 문자열 — 서버를 부르지 않는다', async () => {
    expect(await currentTicket('c1')).toBe('');
    expect(issued).toHaveLength(0);
  });

  it('지금 붙으려는 로봇의 시리얼로 발급받는다', async () => {
    setTicketSession('0000', ROBOTS);
    nextResult = ok('T1', 5);
    expect(await currentTicket('c1')).toBe('T1');
    expect(issued).toEqual([{ pin: '0000', serial: 'RBQ103' }]);
  });

  it('★로봇이 바뀌면 새로 받는다 — 다른 로봇 표를 내면 랑데부가 거부한다', async () => {
    setTicketSession('0000', ROBOTS);
    nextResult = ok('T1', 5);
    await currentTicket('c1');
    liveRobotId = 'RBQ105';
    nextResult = ok('T2', 5);
    expect(await currentTicket('c1')).toBe('T2');
    expect(issued.map((i) => i.serial)).toEqual(['RBQ103', 'RBQ105']);
  });

  it('같은 로봇이고 유효하면 재사용한다 — 매번 왕복하면 연결이 느려진다', async () => {
    setTicketSession('0000', ROBOTS);
    nextResult = ok('T1', 5);
    await currentTicket('c1');
    await currentTicket('c2');
    expect(issued).toHaveLength(1);
  });

  it('만료가 임박하면 새로 받는다 — 내는 순간 이미 지났으면 소용없다', async () => {
    setTicketSession('0000', ROBOTS);
    nextResult = ok('T1', 0.2);
    await currentTicket('c1');
    nextResult = ok('T2', 5);
    expect(await currentTicket('c1')).toBe('T2');
    expect(issued).toHaveLength(2);
  });

  it('이 계정에 없는 로봇이면 발급하지 않는다', async () => {
    setTicketSession('0000', ROBOTS);
    liveRobotId = 'RBQ999';
    expect(await currentTicket('c1')).toBe('');
    expect(issued).toHaveLength(0);
  });

  it('붙을 로봇이 정해지지 않았으면 발급하지 않는다', async () => {
    setTicketSession('0000', ROBOTS);
    liveRobotId = '';
    expect(await currentTicket('c1')).toBe('');
    expect(issued).toHaveLength(0);
  });

  it('발급 실패는 빈 문자열 — 연결을 여기서 막지 않는다', async () => {
    setTicketSession('0000', ROBOTS);
    nextResult = { ok: false, reason: 'denied' };
    expect(await currentTicket('c1')).toBe('');
  });

  it('해제하면 발급하지 않는다', async () => {
    setTicketSession('0000', ROBOTS);
    clearTicketSession();
    expect(await currentTicket('c1')).toBe('');
    expect(ticketExpiry()).toBeNull();
  });

  it('매칭이 없으면(목록이 비면) 발급하지 않는다', async () => {
    setTicketSession('0000', []);
    expect(await currentTicket('c1')).toBe('');
    expect(issued).toHaveLength(0);
  });
});

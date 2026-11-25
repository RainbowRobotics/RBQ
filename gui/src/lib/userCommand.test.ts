import { describe, it, expect, vi, beforeEach } from 'vitest';
const calls: { target: number; cmd: number; para: unknown }[] = [];
let fail: unknown = null;
vi.mock('./rest', () => ({
  rest: {
    commandStruct: (_ip: string, target: number, cmd: number, para: unknown) => {
      calls.push({ target, cmd, para });
      const e = fail; fail = null;
      return e ? Promise.reject(e) : Promise.resolve({});
    },
  },
}));
import { sendUserCommand } from './userCommand';
import { useStandGate } from './standGate';

const gate409 = Object.assign(new Error('HTTP 409'), { status: 409, detail: JSON.stringify({ error: 'equipment_without_payload', ids: ['0x150'] }) });
const flush = () => new Promise((r) => setTimeout(r, 0));

describe('USER_COMMAND stand gate — [Auto Recovery] 처럼 user_command 로 일으키는 명령도 창을 띄운다', () => {
  beforeEach(() => { calls.length = 0; fail = null; useStandGate.setState({ pending: null }); vi.spyOn(console, 'warn').mockImplementation(() => {}); });

  it('409 stand gate 면 확인 창 — [그래도 세우기]는 같은 명령을 force 로 한 번만', async () => {
    fail = gate409;
    sendUserCommand(1, 2);
    await flush();
    const p = useStandGate.getState().pending;
    expect(p?.reason).toBe('equipment_without_payload');
    p!.force();
    await flush();
    expect(calls).toEqual([{ target: 1, cmd: 2, para: {} }, { target: 1, cmd: 2, para: { force: true } }]);
  });

  it('force 로 보낸 것이 또 거절되면 창을 다시 띄우지 않는다', async () => {
    fail = gate409;
    sendUserCommand(1, 2);
    await flush();
    const p = useStandGate.getState().pending!;
    useStandGate.setState({ pending: null });
    fail = gate409;
    p.force();
    await flush();
    expect(useStandGate.getState().pending).toBeNull();
  });

  it('다른 실패(403 등)는 창 없이 경고만', async () => {
    fail = Object.assign(new Error('HTTP 403'), { status: 403, detail: '{"error":"not owner"}' });
    sendUserCommand(1, 2);
    await flush();
    expect(useStandGate.getState().pending).toBeNull();
    expect(console.warn).toHaveBeenCalled();
  });
});

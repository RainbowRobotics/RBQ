import { describe, expect, it, vi } from 'vitest';

const native = vi.hoisted(() => ({
  probe: vi.fn(),
  relay: vi.fn(async (_h: string, _p: number, _fp: string) => 41234),
  close: vi.fn(),
}));
vi.mock('../../modules/robot-tls/src/RobotTlsModule', () => ({ default: native }));

import { robotTlsPlatform as p } from './robotTlsPlatform';

describe('robotTlsPlatform', () => {
  it('base_after_relay', async () => {
    await p.engage('10.0.0.1', 8080, 'aa');
    expect(native.relay).toHaveBeenCalledWith('10.0.0.1', 8080, 'aa');
    expect(p.base('10.0.0.1', 8080)).toBe('http://127.0.0.1:41234');
  });

  it('base_after_plain', async () => {
    await p.engage('10.0.0.2', 8081, 'aa');
    await p.engage('10.0.0.2', 8081, null);
    expect(native.close).toHaveBeenCalledWith('10.0.0.2', 8081);
    expect(p.base('10.0.0.2', 8081)).toBe('http://10.0.0.2:8081');
  });

  it('base_after_block', async () => {
    await p.engage('10.0.0.3', 8082, 'aa');
    p.block('10.0.0.3', 8082);
    expect(native.close).toHaveBeenCalledWith('10.0.0.3', 8082);
    expect(p.base('10.0.0.3', 8082)).toBe('https://10.0.0.3:8082');
  });

  it('kept_only_relay_or_block', async () => {
    await p.engage('10.0.1.1', 8080, 'F');
    await p.engage('10.0.1.1', 8081, null);
    p.block('10.0.1.1', 8082);
    expect([8080, 8081, 8082].map((port) => p.kept('10.0.1.1', port))).toEqual([true, false, true]);
    expect(p.kept('10.0.1.2', 8080)).toBe(false);
    expect([8080, 8081, 8082].map((port) => p.pinnedFp('10.0.1.1', port))).toEqual(['F', undefined, undefined]);
    expect(p.pinnedFp('10.0.1.2', 8080)).toBeUndefined();
  });
  it('base_untouched_is_undefined', () => {
    expect(p.base('10.0.0.4', 8080)).toBeUndefined();
  });

  it('engage_same_fp_keeps_base', async () => {
    native.relay.mockClear();
    await p.engage('10.0.0.5', 8080, 'aa');
    const first = p.base('10.0.0.5', 8080);
    await p.engage('10.0.0.5', 8080, 'aa');
    expect(native.relay).toHaveBeenCalledTimes(2);
    expect(p.base('10.0.0.5', 8080)).toBe(first);
    expect(native.close).not.toHaveBeenCalledWith('10.0.0.5', 8080);
  });

  it('engage_after_relay_died_takes_new_port', async () => {
    native.relay.mockResolvedValueOnce(41234).mockResolvedValueOnce(50001);
    await p.engage('10.0.0.8', 8080, 'aa');
    await p.engage('10.0.0.8', 8080, 'aa');
    expect(p.base('10.0.0.8', 8080)).toBe('http://127.0.0.1:50001');
  });

  it('block_during_relay_wins', async () => {
    let open!: (port: number) => void;
    native.relay.mockImplementationOnce(() => new Promise<number>((r) => { open = r; }));
    native.close.mockClear();
    const pending = p.engage('10.0.0.9', 8080, 'aa');
    p.block('10.0.0.9', 8080);
    open(41234);
    await pending;
    expect(p.base('10.0.0.9', 8080)).toBe('https://10.0.0.9:8080');
    expect(native.close).toHaveBeenCalledTimes(2);
  });

  it('relay_failure_blocks', async () => {
    native.relay.mockRejectedValueOnce(new Error('bind'));
    await expect(p.engage('10.0.0.6', 8080, 'aa')).rejects.toThrow('bind');
    expect(p.base('10.0.0.6', 8080)).toBe('https://10.0.0.6:8080');
  });

  it('probe_passes_serial', async () => {
    native.probe.mockResolvedValueOnce({ kind: 'tls', fp: 'ff', serial: 'S' });
    expect(await p.probe('10.0.0.7', 8080)).toEqual({ kind: 'tls', fp: 'ff', serial: 'S' });
    expect(native.probe).toHaveBeenCalledWith('10.0.0.7', 8080);
  });
});

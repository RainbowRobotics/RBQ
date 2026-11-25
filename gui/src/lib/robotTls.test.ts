import { describe, it, expect, beforeEach, vi } from 'vitest';
import { decide, pinKey, secureRobot, settleRobot, setTlsPinSource, setTlsIssueHandler, isPinnedSerial, RELEARN_PIN, type Probe, type TlsPlatform, type TlsIssue } from './robotTls';

const tls = (fp: string, serial: string | null = null): Probe => ({ kind: 'tls', fp, serial });

let probes: Record<string, Probe>;
let calls: string[];
let issues: TlsIssue[];
let p: TlsPlatform;
const noSerialKeyFor = async (s: string, ip: string) => s || `addr:${ip}`;

function setup(o: { selectedKey?: string | null; selectedSsid?: string | null; pins?: Record<string, string>; ssid?: string | null }) {
  setTlsPinSource(() => ({
    selectedKey: o.selectedKey ?? null, selectedSsid: o.selectedSsid ?? null, pins: o.pins ?? {},
    keyFor: noSerialKeyFor, currentSsid: async () => o.ssid ?? null,
  }));
}

beforeEach(() => {
  probes = {}; calls = []; issues = [];
  p = {
    probe: vi.fn(async (h: string, port: number) => probes[`${h}:${port}`] ?? { kind: 'down' as const }),
    engage: vi.fn(async (h: string, port: number, fp: string | null) => { calls.push(`engage ${h}:${port} ${fp}`); }),
    block: vi.fn((h: string, port: number) => { calls.push(`block ${h}:${port}`); }),
    kept: () => false,
    pinnedFp: () => undefined,
  };
  setTlsIssueHandler((i) => issues.push(i));
});

describe('decide', () => {
  it('decide_table', () => {
    expect(decide(undefined, tls('F'))).toBe('learn');
    expect(decide('F', tls('F'))).toBe('ok');
    expect(decide('G', tls('F'))).toBe('changed');
    expect(decide(undefined, { kind: 'plain' })).toBe('fallback');
    expect(decide('F', { kind: 'plain' })).toBe('refuse-old');
    expect(decide('F', { kind: 'down' })).toBe('down');
    expect(decide(undefined, { kind: 'down' })).toBe('down');
  });
  it('decide_relearn_pin', () => {
    expect(decide(RELEARN_PIN, tls('F'))).toBe('learn');
    expect(decide(RELEARN_PIN, { kind: 'plain' })).toBe('refuse-old');
    expect(decide(RELEARN_PIN, { kind: 'down' })).toBe('down');
  });
  it('pinKey', () => { expect(pinKey('S', 8080)).toBe('S@8080'); });
});

describe('secureRobot', () => {
  it('engages_tls_fp', async () => {
    setup({ selectedKey: 'S' });
    probes['ip:8080'] = tls('F', 'S');
    expect(await secureRobot('ip', '', p)).toEqual({ result: 'ok', key: 'S' });
    expect(calls).toContain('engage ip:8080 F');
  });
  it('plain_unseen_engages_plain', async () => {
    setup({ selectedKey: 'S' });
    probes['ip:8080'] = { kind: 'plain' };
    expect((await secureRobot('ip', '', p)).result).toBe('ok');
    expect(calls).toContain('engage ip:8080 null');
    expect(issues).toEqual([]);
  });
  it('down_keeps_only_relay_or_block', async () => {
    setup({ selectedKey: 'S' });
    probes['ip:8080'] = tls('F', 'S');
    p.kept = (_h, port) => port === 8081 || port === 8082;
    expect((await secureRobot('ip', '', p)).result).toBe('ok');
    expect(calls).toEqual(['engage ip:8080 F']);
    p.kept = () => false;
    calls.length = 0;
    await secureRobot('ip', '', p);
    expect(calls).toEqual(['engage ip:8080 F', 'block ip:8081', 'block ip:8082']);
    expect(issues).toEqual([]);
  });
  it('down_blocks_plain_route_of_another_robot', async () => {
    setup({ selectedKey: 'S', pins: { 'S@8081': 'F1' } });
    probes['ip:8080'] = tls('F', 'S');
    p.kept = () => false;
    await secureRobot('ip', '', p);
    expect(calls).toContain('block ip:8081');
  });
  it('reports_through_given_reporter', async () => {
    setup({ pins: { 'S@8080': 'G' } });
    probes['ip:8080'] = tls('F', 'S');
    const mine: TlsIssue[] = [];
    await secureRobot('ip', '', p, (i) => mine.push(i));
    expect(mine).toEqual([{ kind: 'changed', entryKey: 'S', port: 8080, fp: 'F', forget: [] }]);
    expect(issues).toEqual([]);
  });
  it('refuses_pinned_plain', async () => {
    probes['ip:8080'] = { kind: 'plain' };
    setup({ selectedKey: 'S', selectedSsid: 'A', ssid: 'A', pins: { 'S@8080': 'F' } });
    expect((await secureRobot('ip', '', p)).result).toBe('refused');
    expect(p.block).toHaveBeenCalledWith('ip', 8080);
    expect(p.engage).not.toHaveBeenCalled();
    expect(issues).toEqual([{ kind: 'refuse-old', entryKey: 'S' }]);
    issues.length = 0;
    setup({ selectedKey: 'S', selectedSsid: 'A', ssid: null, pins: { 'S@8080': 'F' } });
    expect((await secureRobot('ip', '', p)).result).toBe('refused');
  });
  it('plain_at_live_relay_of_selected_robot_is_down', async () => {
    setup({ selectedKey: 'S', selectedSsid: 'A', ssid: 'A', pins: { 'S@8080': 'F', 'S@8081': 'F' } });
    probes['ip:8080'] = { kind: 'plain' };
    probes['ip:8081'] = { kind: 'plain' };
    p.kept = (_h, port) => port !== 8082;
    p.pinnedFp = (_h, port) => (port === 8082 ? undefined : 'F');
    expect((await secureRobot('ip', '', p, undefined, { live: true })).result).toBe('ok');
    expect(issues).toEqual([]);
    expect(calls).toEqual(['block ip:8082']);
  });
  it('plain_on_pinned_route_is_refuse_old_when_not_live', async () => {
    setup({ selectedKey: 'S', selectedSsid: 'A', ssid: 'A', pins: { 'S@8080': 'F', 'S@8081': 'F' } });
    probes['ip:8080'] = { kind: 'plain' };
    probes['ip:8081'] = { kind: 'plain' };
    p.kept = (_h, port) => port !== 8082;
    p.pinnedFp = (_h, port) => (port === 8082 ? undefined : 'F');
    expect((await secureRobot('ip', '', p, undefined, { live: false })).result).toBe('refused');
    expect(p.block).toHaveBeenCalledWith('ip', 8080);
    expect(p.engage).not.toHaveBeenCalled();
    expect(issues).toEqual([{ kind: 'refuse-old', entryKey: 'S' }]);
  });
  it('plain_at_other_robots_relay_judged_as_before', async () => {
    setup({ selectedKey: 'Y' });
    probes['ip:8080'] = { kind: 'plain' };
    p.kept = () => true;
    p.pinnedFp = () => 'FX';
    expect((await secureRobot('ip', '', p)).result).toBe('ok');
    expect(calls).toContain('engage ip:8080 null');
    setup({ selectedKey: 'Z', selectedSsid: 'A', ssid: 'A', pins: { 'Z@8080': 'FZ' } });
    calls.length = 0;
    expect((await secureRobot('ip', '', p)).result).toBe('refused');
    expect(issues).toEqual([{ kind: 'refuse-old', entryKey: 'Z' }]);
  });
  it('plain_other_ssid_falls_back', async () => {
    probes['ip:8080'] = { kind: 'plain' };
    setup({ selectedKey: 'S', selectedSsid: 'RBQ_A', ssid: 'RBQ_B', pins: { 'S@8080': 'F' } });
    expect((await secureRobot('ip', '', p)).result).toBe('ok');
    expect(calls).toContain('engage ip:8080 null');
    expect(issues).toEqual([]);
  });
  it('refuses_changed_before_any_request', async () => {
    probes['ip:8080'] = tls('F', 'S');
    setup({ selectedKey: 'S', pins: { 'S@8080': 'G' } });
    expect((await secureRobot('ip', '', p)).result).toBe('refused');
    expect(p.engage).not.toHaveBeenCalled();
    expect(issues).toEqual([{ kind: 'changed', entryKey: 'S', port: 8080, fp: 'F', forget: [] }]);
  });
  it('changed_reports_forget_for_down_pinned_ports', async () => {
    probes['ip:8080'] = tls('F', 'S');
    setup({ selectedKey: 'S', pins: { 'S@8080': 'G', 'S@8081': 'G1' } });
    expect((await secureRobot('ip', '', p)).result).toBe('refused');
    expect(issues).toEqual([{ kind: 'changed', entryKey: 'S', port: 8080, fp: 'F', forget: [8081] }]);
  });
  it('masked_plain_port_not_forgotten', async () => {
    setup({ selectedKey: 'S', selectedSsid: 'A', ssid: 'A', pins: { 'S@8080': 'G', 'S@8081': 'F1', 'S@8082': 'F2' } });
    probes['ip:8080'] = tls('F', 'S');
    probes['ip:8081'] = { kind: 'plain' };
    p.kept = (_h, port) => port === 8081;
    p.pinnedFp = (_h, port) => (port === 8081 ? 'F1' : undefined);
    expect((await secureRobot('ip', '', p, undefined, { live: true })).result).toBe('refused');
    expect(issues).toEqual([{ kind: 'changed', entryKey: 'S', port: 8080, fp: 'F', forget: [8082] }]);
    expect(calls).not.toContain('block ip:8081');
    issues.length = 0; calls.length = 0;
    setup({ selectedKey: 'S', selectedSsid: 'A', ssid: 'A', pins: { 'S@8080': 'F', 'S@8081': 'F1', 'S@8082': RELEARN_PIN } });
    expect((await secureRobot('ip', '', p, undefined, { live: false })).result).toBe('refused');
    expect(calls).toContain('block ip:8081');
    expect(issues).toEqual([{ kind: 'refuse-old', entryKey: 'S' }]);
  });
  it('relearn_pin_refuses_plain_then_learns_tls', async () => {
    setup({ selectedKey: 'S', selectedSsid: 'A', ssid: 'A', pins: { 'S@8080': 'F', 'S@8081': RELEARN_PIN } });
    probes['ip:8080'] = tls('F', 'S');
    probes['ip:8081'] = { kind: 'plain' };
    p.pinnedFp = () => RELEARN_PIN;
    expect((await secureRobot('ip', '', p, undefined, { live: true })).result).toBe('refused');
    expect(calls).not.toContain('engage ip:8081 null');
    expect(issues).toEqual([{ kind: 'refuse-old', entryKey: 'S' }]);
    issues.length = 0; calls.length = 0;
    probes['ip:8081'] = tls('F1');
    expect((await secureRobot('ip', '', p)).result).toBe('ok');
    expect(calls).toContain('engage ip:8081 F1');
    expect(settleRobot('S', 'ip')).toEqual({ learn: { 8081: 'F1' }, refused: false });
    expect(isPinnedSerial('S')).toBe(true);
  });
  it('changed_forget_empty_when_key_guessed', async () => {
    setup({ selectedKey: 'S', pins: { 'S@8080': 'G', 'S@8081': 'G1', 'S@8082': 'G2' } });
    probes['ip:8081'] = tls('F1');
    expect((await secureRobot('ip', '', p)).result).toBe('refused');
    expect(issues).toEqual([{ kind: 'changed', entryKey: 'S', port: 8081, fp: 'F1', forget: [] }]);
  });
  it('keys_by_reported_serial', async () => {
    probes['ip:8080'] = tls('FB', 'B');
    setup({ selectedKey: 'A', pins: { 'A@8080': 'FA', 'B@8080': 'FB' } });
    expect(await secureRobot('ip', '', p)).toEqual({ result: 'ok', key: 'B' });
    expect(calls).toContain('engage ip:8080 FB');
    expect(issues).toEqual([]);
  });
  it('new_serial_learns_even_if_selected_pinned', async () => {
    probes['ip:8080'] = tls('FC', 'C');
    setup({ selectedKey: 'A', pins: { 'A@8080': 'FA' } });
    expect(await secureRobot('ip', '', p)).toEqual({ result: 'ok', key: 'C' });
    expect(issues).toEqual([]);
    expect(settleRobot('C', 'ip').learn).toEqual({ 8080: 'FC' });
  });
  it('empty_serial_uses_noSerialKey', async () => {
    probes['ip:8080'] = tls('F', '');
    setup({ selectedKey: 'S' });
    expect((await secureRobot('ip', '', p)).key).toBe('addr:ip');
  });
  it('serial_unknown_uses_selected', async () => {
    probes['ip:8080'] = tls('F', null);
    setup({ selectedKey: 'S', pins: { 'S@8080': 'G' } });
    expect((await secureRobot('ip', '', p)).result).toBe('refused');
    expect(issues).toEqual([{ kind: 'changed', entryKey: 'S', port: 8080, fp: 'F', forget: [] }]);
  });
  it('probes_vision_ports_on_vision_ip', async () => {
    probes['ip:8080'] = tls('F0', 'S');
    probes['10.0.0.2:8081'] = tls('F1'); probes['10.0.0.2:8082'] = tls('F2');
    setup({});
    expect((await secureRobot('ip', '10.0.0.2', p)).result).toBe('ok');
    expect(calls).toEqual(expect.arrayContaining(['engage ip:8080 F0', 'engage 10.0.0.2:8081 F1', 'engage 10.0.0.2:8082 F2']));
  });
});

describe('settleRobot', () => {
  it('settle_learns_under_noSerialKey', async () => {
    probes['192.168.100.109:8080'] = tls('F', '');
    setup({});
    await secureRobot('192.168.100.109', '', p);
    expect(settleRobot('addr:192.168.100.109', '192.168.100.109').learn).toEqual({ 8080: 'F' });
  });
  it('settle_changed_refuses_without_block', async () => {
    probes['ip:8080'] = tls('F', 'S');
    setup({});
    await secureRobot('ip', '', p);
    issues.length = 0; (p.block as any).mockClear();
    setup({ pins: { 'S@8080': 'G' } });
    expect(settleRobot('S', 'ip')).toEqual({ learn: {}, refused: true });
    expect(issues).toEqual([{ kind: 'changed', entryKey: 'S', port: 8080, fp: 'F', forget: [] }]);
    expect(p.block).not.toHaveBeenCalled();
  });
  it('settle_forget_uses_raw_probe', async () => {
    setup({ selectedKey: 'S', pins: { 'S@8080': 'F', 'S@8081': 'F1' } });
    probes['ip:8080'] = tls('F', 'S');
    probes['ip:8081'] = { kind: 'plain' };
    p.kept = (_h, port) => port === 8081;
    p.pinnedFp = (_h, port) => (port === 8081 ? 'F1' : undefined);
    expect((await secureRobot('ip', '', p, undefined, { live: true })).result).toBe('ok');
    issues.length = 0;
    setup({ pins: { 'S@8080': 'G', 'S@8081': 'F1', 'S@8082': 'F2' } });
    expect(settleRobot('S', 'ip').refused).toBe(true);
    expect(issues).toEqual([{ kind: 'changed', entryKey: 'S', port: 8080, fp: 'F', forget: [8082] }]);
  });
  it('settle_changed_reports_forget', async () => {
    probes['ip:8080'] = tls('F', 'S');
    setup({});
    await secureRobot('ip', '', p);
    issues.length = 0;
    setup({ pins: { 'S@8080': 'G', 'S@8081': 'G1' } });
    expect(settleRobot('S', 'ip').refused).toBe(true);
    expect(issues).toEqual([{ kind: 'changed', entryKey: 'S', port: 8080, fp: 'F', forget: [8081] }]);
  });
});

describe('settleRobot 범위', () => {
  it('settle_without_secure_is_noop', () => {
    setup({ pins: { 'S@8080': 'G' } });
    expect(settleRobot('S', 'ip')).toEqual({ learn: {}, refused: false });
    expect(issues).toEqual([]);
  });
  it('settle_other_ip_is_noop', async () => {
    probes['10.0.0.1:8080'] = tls('FA', 'A');
    setup({ pins: { 'B@8080': 'FB' } });
    await secureRobot('10.0.0.1', '', p);
    expect(settleRobot('B', '10.0.0.2')).toEqual({ learn: {}, refused: false });
    expect(issues).toEqual([]);
  });
  it('settle_consumes_probes', async () => {
    probes['ip:8080'] = tls('F', 'S');
    setup({});
    await secureRobot('ip', '', p);
    expect(settleRobot('S', 'ip').learn).toEqual({ 8080: 'F' });
    expect(settleRobot('S', 'ip')).toEqual({ learn: {}, refused: false });
  });
});

describe('isPinnedSerial', () => {
  it('지문이 하나라도 있으면 참', () => {
    setup({ pins: { 'S@8080': 'F' } });
    expect(isPinnedSerial('S')).toBe(true);
    expect(isPinnedSerial('T')).toBe(false);
  });
});

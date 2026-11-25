import { describe, it, expect, vi } from 'vitest';

const native = vi.hoisted(() => ({ probe: vi.fn(), relay: vi.fn(async () => 41234), close: vi.fn() }));
vi.mock('../../modules/robot-tls/src/RobotTlsModule', () => ({ default: native }));
const robot = vi.hoisted(() => ({ visionIp: '' }));
vi.mock('@/store/robot', () => ({ useRobot: { getState: () => robot } }));

import { restBase, streamerBase, slamnavBase } from './endpoints';
import { robotTlsPlatform as p } from './robotTlsPlatform';
import { targetOf } from './robotTunnel';

describe('endpoints — 판정이 정한 경로만, 손 안 댄 포트는 https(실패로 닫힘, R14)', () => {
  it('restBase_uses_engaged_base', async () => {
    await p.engage('192.168.0.10', 8080, 'aa');
    expect(restBase('192.168.0.10')).toBe('http://127.0.0.1:41234');
  });

  it('restBase_untouched_fails_closed', () => {
    expect(restBase('192.168.0.11')).toBe('https://192.168.0.11:8080');
  });

  it('restBase_plain_after_fallback', async () => {
    await p.engage('192.168.0.12', 8080, null);
    expect(restBase('192.168.0.12')).toBe('http://192.168.0.12:8080');
  });

  it('streamerBase_uses_vision_ip_port_8081', async () => {
    robot.visionIp = '10.0.0.2';
    expect(streamerBase('192.168.0.13')).toBe('https://10.0.0.2:8081');
    await p.engage('10.0.0.2', 8081, 'bb');
    expect(streamerBase('192.168.0.13')).toBe('http://127.0.0.1:41234');
    robot.visionIp = '';
  });

  it('slamnavBase_uses_vision_ip_port_8082', async () => {
    robot.visionIp = '';
    expect(slamnavBase('192.168.0.14')).toBe('https://192.168.0.14:8082');
    p.block('192.168.0.14', 8082);
    expect(slamnavBase('192.168.0.14')).toBe('https://192.168.0.14:8082');
  });

  it('remote_tunnel_takes_port_and_path_only', () => {
    expect(targetOf(`${restBase('RBQ10-001')}/api/version`)).toEqual({ port: 8080, path: '/api/version' });
    expect(targetOf(`${streamerBase('RBQ10-001')}/api/x`)).toEqual({ port: 8081, path: '/api/x' });
  });
});

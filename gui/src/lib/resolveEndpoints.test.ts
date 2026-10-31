import { describe, it, expect } from 'vitest';
import { resolveEndpoints } from './resolveEndpoints';

describe('resolveEndpoints', () => {
  it('visionIp 미설정이면 motion 상속', () => {
    expect(resolveEndpoints('192.168.0.10')).toEqual({ motion: '192.168.0.10', vision: '192.168.0.10' });
  });
  it('visionIp 공백/스페이스면 상속', () => {
    expect(resolveEndpoints('192.168.0.10', '  ').vision).toBe('192.168.0.10');
    expect(resolveEndpoints('192.168.0.10', '').vision).toBe('192.168.0.10');
  });
  it('visionIp 명시하면 분리', () => {
    expect(resolveEndpoints('192.168.0.10', '192.168.0.11')).toEqual({ motion: '192.168.0.10', vision: '192.168.0.11' });
  });
});

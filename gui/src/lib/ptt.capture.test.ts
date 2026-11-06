import { describe, it, expect, vi, beforeEach } from 'vitest';

const calls: string[] = [];
vi.mock('@/lib/webrtcClient', () => ({
  webrtcClient: {
    setSpeak: async (on: boolean) => { calls.push(`mic:${on}`); return 'ok'; },
    ensureConnected: () => {},
    setListenGate: () => {},
  },
}));
vi.mock('@/lib/commandBus', () => ({
  streamerCommand: async (_m: string, path: string, body: any) => { calls.push(`${path.split('/').pop()}:${body?.state ?? ''}`); return {}; },
  onStreamerCommandReady: () => {},
}));
vi.mock('@/lib/demoFlag', () => ({ isDemo: () => false }));
vi.mock('@/lib/resolveEndpoints', () => ({ resolveEndpoints: () => ({ vision: '1.2.3.4' }) }));
vi.mock('@/store/robot', () => ({ useRobot: { getState: () => ({ ip: '1.2.3.4', visionIp: '' }) } }));
vi.mock('@/store/settings', () => ({ useSettings: { getState: () => ({ audio: {}, setAudio: () => {} }) } }));

import { ptt } from '@/lib/ptt';

describe('ptt — 녹음 캡처와 워키토키 분리', () => {
  beforeEach(() => { calls.length = 0; });

  it('녹음 중 워키토키를 눌렀다 떼도 마이크는 녹음이 끝날 때까지 켜져 있다', async () => {
    await ptt.setCapture(true);
    expect(calls).toEqual(['ptt_talk:stop', 'mic:true']);
    await ptt.startSpeak();
    ptt.stopSpeak();
    await Promise.resolve();
    expect(calls).toEqual(['ptt_talk:stop', 'mic:true', 'ptt_talk:start', 'ptt_talk:stop']);
    await ptt.setCapture(false);
    expect(calls.at(-1)).toBe('mic:false');
  });

  it('녹음이 없으면 워키토키를 뗄 때 마이크가 꺼진다', async () => {
    await ptt.startSpeak();
    ptt.stopSpeak();
    await Promise.resolve();
    expect(calls).toEqual(['ptt_talk:start', 'mic:true', 'ptt_talk:stop', 'mic:false']);
  });
});

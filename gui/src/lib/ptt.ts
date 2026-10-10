import { create } from 'zustand';
import { streamerCommand, onStreamerCommandReady } from '@/lib/commandBus';
import type { RobotAudioStatus } from '@/lib/rest';
import { isDemo } from '@/lib/demoFlag';
import { webrtcClient } from '@/lib/webrtcClient';
import { resolveEndpoints } from '@/lib/resolveEndpoints';
import { useRobot } from '@/store/robot';
import { useSettings } from '@/store/settings';

type PttState = {
  listen: boolean;
  speaking: boolean;
  error: null | 'denied' | 'unavailable';
  status: RobotAudioStatus | null;
};
export const usePtt = create<PttState>(() => ({ listen: false, speaking: false, error: null, status: null }));

function put(path: string, body: object) {
  if (isDemo()) return;
  streamerCommand('PUT', path, body).then((r: any) => applyAudioReadback(r?.audio)).catch(() => {});
}

export function applyAudioReadback(a: ({ speaker_volume?: number; mic_volume?: number } & RobotAudioStatus) | undefined) {
  if (!a) return;
  if (typeof a.output === 'string') usePtt.setState({ status: { mic_present: a.mic_present, mic_signal: a.mic_signal, speaker_present: a.speaker_present, ptz: a.ptz, output: a.output, input: a.input } });
  const st = useSettings.getState();
  if (typeof a.speaker_volume === 'number' && st.audio.robotSpk !== a.speaker_volume) st.setAudio('robotSpk', a.speaker_volume);
  if (typeof a.mic_volume === 'number' && st.audio.robotMic !== a.mic_volume) st.setAudio('robotMic', a.mic_volume);
}

function ensurePeer() {
  const { ip, visionIp } = useRobot.getState();
  if (ip) webrtcClient.ensureConnected(resolveEndpoints(ip, visionIp).vision);
}

let speakSeq = 0;

export const ptt = {
  setListen(on: boolean) {
    if (on) ensurePeer();
    usePtt.setState({ listen: on });
    webrtcClient.setListenGate(on);
    put('/api/audio/ptt_listen', { state: on ? 'start' : 'stop' });
  },
  async startSpeak() {
    ensurePeer();
    const seq = ++speakSeq;
    const r = await webrtcClient.setSpeak(true);
    if (seq !== speakSeq) { webrtcClient.setSpeak(false); return; }
    usePtt.setState({ speaking: r === 'ok', error: r === 'ok' ? null : r });
  },
  stopSpeak() {
    ++speakSeq;
    webrtcClient.setSpeak(false);
    usePtt.setState({ speaking: false });
  },
  setRobotSpeakerVolume(pct: number) {
    put('/api/audio/speaker_volume', { percent: Math.round(pct) });
  },
  setRobotMicVolume(pct: number) {
    put('/api/audio/mic_volume', { percent: Math.round(pct) });
  },
  refreshStatus() {
    if (isDemo()) return;
    streamerCommand('GET', '/api/audio/status').then((r: any) => applyAudioReadback(r?.audio)).catch(() => {});
  },
  setAppVolume(pct: number) {
    webrtcClient.setAppVolume(pct);
  },
};

onStreamerCommandReady(() => ptt.refreshStatus());

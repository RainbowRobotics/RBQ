
class DesktopAudio {
  private ctx: AudioContext | null = null;
  private gain: GainNode | null = null;
  private cursor = 0;
  private listen = false;
  private volume = 1.0;
  private micStream: MediaStream | null = null;
  private proc: ScriptProcessorNode | null = null;
  private micSrc: MediaStreamAudioSourceNode | null = null;
  private speaking = false;
  private agc = { gain: 1 };
  private sender: ((frame: Uint8Array) => void) | null = null;

  setSender(fn: ((frame: Uint8Array) => void) | null) { this.sender = fn; }

  private kickInstalled = false;
  private ensureCtx(): AudioContext {
    if (!this.ctx) {
      this.ctx = new AudioContext();
      this.gain = this.ctx.createGain();
      this.gain.gain.value = this.volume;
      this.gain.connect(this.ctx.destination);
    }
    if (this.ctx.state === 'suspended') {
      this.ctx.resume().catch(() => {});
      if (!this.kickInstalled && typeof document !== 'undefined') {
        this.kickInstalled = true;
        const kick = () => {
          const c = this.ctx;
          if (!c || c.state !== 'suspended') { cleanup(); return; }
          c.resume().then(() => { if (c.state === 'running') cleanup(); }).catch(() => {});
        };
        const cleanup = () => {
          for (const t of ['touchstart', 'touchend', 'mousedown', 'click'])
            document.removeEventListener(t, kick, true);
          this.kickInstalled = false;
        };
        for (const t of ['touchstart', 'touchend', 'mousedown', 'click'])
          document.addEventListener(t, kick, true);
      }
    }
    return this.ctx;
  }

  playPcm(bytes: Uint8Array) {
    if (!this.listen) return;
    const n = bytes.byteLength >> 1;
    if (!n) return;
    const ctx = this.ensureCtx();
    const aligned = new Uint8Array(n * 2);
    aligned.set(bytes.subarray(0, n * 2));
    const i16 = new Int16Array(aligned.buffer);
    const buf = ctx.createBuffer(1, n, 48000);
    const ch = buf.getChannelData(0);
    for (let i = 0; i < n; i++) ch[i] = i16[i] / 32768;
    const src = ctx.createBufferSource();
    src.buffer = buf;
    src.connect(this.gain!);
    const now = ctx.currentTime;
    if (this.cursor < now + 0.05) this.cursor = now + 0.05;
    src.start(this.cursor);
    this.cursor += buf.duration;
  }

  async startSpeak(): Promise<'ok' | 'denied' | 'unavailable'> {
    this.ensureCtx();
    if (!this.sender) return 'unavailable';
    if (!navigator.mediaDevices?.getUserMedia) return 'denied';
    try {
      if (!this.micStream) this.micStream = await navigator.mediaDevices.getUserMedia({ audio: { autoGainControl: true, noiseSuppression: true, echoCancellation: true } });
    } catch {
      return 'denied';
    }
    const ctx = this.ensureCtx();
    if (!this.proc) {
      this.micSrc = ctx.createMediaStreamSource(this.micStream);
      this.proc = ctx.createScriptProcessor(1024, 1, 1);
      this.micSrc.connect(this.proc);
      const mute = ctx.createGain();
      mute.gain.value = 0;
      this.proc.connect(mute);
      mute.connect(ctx.destination);
      this.proc.onaudioprocess = (e) => {
        if (!this.speaking || !this.sender) return;
        const input = agcProcess(e.inputBuffer.getChannelData(0), this.agc);
        this.sender(f32ToS16Resampled(input, e.inputBuffer.sampleRate, 48000));
      };
    }
    this.speaking = true;
    return 'ok';
  }

  stopSpeak() { this.speaking = false; }

  setListen(on: boolean) {
    this.listen = on;
    if (on) this.ensureCtx();
  }

  setVolume(pct: number) {
    this.volume = Math.max(0, Math.min(100, pct)) / 100;
    if (this.gain) this.gain.gain.value = this.volume;
  }

  resetMic() {
    this.speaking = false;
    try { this.proc?.disconnect(); } catch { }
    try { this.micSrc?.disconnect(); } catch { }
    this.proc = null;
    this.micSrc = null;
    for (const t of this.micStream?.getTracks() ?? []) { try { t.stop(); } catch { } }
    this.micStream = null;
  }

  stop() {
    this.resetMic();
    this.sender = null;
    this.cursor = 0;
  }
}

export function agcProcess(input: Float32Array, st: { gain: number }): Float32Array {
  let sum = 0;
  for (let i = 0; i < input.length; i++) sum += input[i] * input[i];
  const rms = Math.sqrt(sum / Math.max(1, input.length));
  if (rms > 0.00316) {
    const want = Math.min(8, Math.max(1, 0.1 / rms));
    st.gain += (want - st.gain) * (want < st.gain ? 0.5 : 0.05);
  }
  const out = new Float32Array(input.length);
  for (let i = 0; i < input.length; i++) out[i] = Math.tanh(input[i] * st.gain);
  return out;
}

function f32ToS16Resampled(input: Float32Array, fromRate: number, toRate: number): Uint8Array {
  const outLen = Math.floor((input.length * toRate) / fromRate);
  const out = new Int16Array(outLen);
  const ratio = fromRate / toRate;
  for (let i = 0; i < outLen; i++) {
    const pos = i * ratio;
    const i0 = Math.floor(pos);
    const i1 = Math.min(i0 + 1, input.length - 1);
    const s = input[i0] + (input[i1] - input[i0]) * (pos - i0);
    out[i] = Math.max(-32768, Math.min(32767, Math.round(s * 32767)));
  }
  return new Uint8Array(out.buffer);
}

export const desktopAudio = new DesktopAudio();

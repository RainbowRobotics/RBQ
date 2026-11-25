import { NativeModules, Platform } from 'react-native';

type AudioBridge = {
  setManualAudio?: (manual: boolean) => void;
  setAudioEnabled?: (enabled: boolean) => void;
};

const bridge: AudioBridge = (NativeModules as { WebRTCModule?: AudioBridge }).WebRTCModule ?? {};

export const iosManualAudioSupported =
  Platform.OS === 'ios' && typeof bridge.setManualAudio === 'function' && typeof bridge.setAudioEnabled === 'function';

let armed = false;
let enabled = true;

export const iosAudioSession = {
  arm() {
    if (!iosManualAudioSupported || armed) return;
    armed = true;
    try {
      bridge.setManualAudio!(true);
      bridge.setAudioEnabled!(false);
      enabled = false;
    } catch {
      armed = false;
    }
  },
  setEnabled(on: boolean) {
    if (!iosManualAudioSupported || !armed || enabled === on) return;
    enabled = on;
    try { bridge.setAudioEnabled!(on); } catch {}
  },
};

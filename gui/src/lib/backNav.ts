import { robotKinds } from '@/modules/registry';

export type BackInput = 'hw' | 'esc' | 'pad';
export type BackAction = 'pop' | 'hub' | 'quit' | null;

const TOP = ['/hub', '/pin'];
const ARM = ['/arm', '/arm-door', '/arm-manual', '/arm-motion'];
const PAD_INPUT = ['/', '/gamepad', ...ARM];

export function backAction(input: BackInput, path: string, canGoBack: boolean, canQuit: boolean, standalone = false): BackAction {
  if (input === 'pad' && (PAD_INPUT.includes(path) || robotKinds.some((k) => k.route === path))) return null;
  if (input === 'esc' && ARM.includes(path)) return null;
  if (canGoBack) return 'pop';
  if (standalone) return null;
  if (!TOP.includes(path)) return 'hub';
  return canQuit ? 'quit' : null;
}

import { describe, it, expect } from 'vitest';
import { fileURLToPath } from 'node:url';
import { ESLint } from 'eslint';

const eslint = new ESLint({ cwd: fileURLToPath(new URL('../..', import.meta.url)) });
async function errors(code: string, filePath = 'src/x.tsx'): Promise<string[]> {
  const [r] = await eslint.lintText(code, { filePath });
  return r.messages.map((m) => `${m.line}:${m.ruleId}`);
}

describe('앱 린트 규칙', () => {
  it('함수 안에서 만든 훅이 콜백을 품으면 막는다', async () => {
    expect(await errors(`import { useEffect } from 'react';
export function screenState(store: { setState: (f: (s: { open: number }) => object) => void }) {
  const useScreen = () => useEffect(() => {
    store.setState((s) => ({ open: s.open + 1 }));
  }, []);
  const useThin = () => useJobScreen(store);
  return { useScreen, useThin };
}
declare function useJobScreen(s: unknown): void;`)).toEqual(['3:no-restricted-syntax', '4:no-restricted-syntax']);
  });

  it('react-native Modal 은 supportedOrientations 를 넘겨야 한다 — 공용 Modal 은 그대로', async () => {
    expect(await errors(`import { Modal as RNModal } from 'react-native';
import { Modal, MODAL_ORIENTATIONS } from '@/components/ui/overlays';
export const A = () => <RNModal visible />;
export const B = () => <RNModal visible supportedOrientations={MODAL_ORIENTATIONS} />;
export const C = () => <Modal visible />;`)).toEqual(['3:rbq/rn-modal-orientations']);
  });

  it('BackHandler 에 거는 것은 BackNav 에서만 — exitApp 은 어디서나', async () => {
    const code = `import { BackHandler } from 'react-native';
BackHandler.addEventListener('hardwareBackPress', () => true);
BackHandler.exitApp();`;
    expect(await errors(code)).toEqual(['2:no-restricted-syntax']);
    expect(await errors(code, 'src/components/BackNav.tsx')).toEqual([]);
  });
});

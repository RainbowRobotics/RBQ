import { describe, it, expect, vi } from 'vitest';

const file = new Blob([new Uint8Array(4)]);
const asked: object[] = [];
vi.mock('expo-document-picker', () => ({
  getDocumentAsync: async (o: object) => {
    asked.push(o);
    return { canceled: false, assets: [{ uri: 'blob:http://x/2', name: 'rbq-usb-v1.tar', size: 4, file }] };
  },
}));
vi.mock('./platformInfo', () => ({ getPlatformInfo: async () => ({ steamos: false, gaming: false }) }));
const remembered: [string, Blob][] = [];
vi.mock('@/lib/fileStream', () => ({ rememberPicked: (uri: string, f: Blob) => remembered.push([uri, f]) }));
import { pickDocument } from './pickDocument';

describe('파일 고르기', () => {
  it('웹 선택기가 준 File 을 그 blob: 주소로 기억해 둔다 — 위치 지정 읽기가 파일을 통째로 복사하지 않게', async () => {
    await pickDocument();
    expect(remembered).toEqual([['blob:http://x/2', file]]);
  });
  it('base64 를 끄고 연다 — 고른 파일을 통째로 읽지 않게', async () => {
    await pickDocument({ copyToCacheDirectory: true });
    expect(asked.at(-1)).toMatchObject({ base64: false, copyToCacheDirectory: true });
  });
});

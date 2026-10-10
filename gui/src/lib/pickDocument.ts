import * as DocumentPicker from 'expo-document-picker';
import { create } from 'zustand';
import { getPlatformInfo } from './platformInfo';

type Result = DocumentPicker.DocumentPickerResult;
type Req = { type?: string; resolve: (r: Result) => void };

export const useLocalPicker = create<{ req: Req | null }>(() => ({ req: null }));

const AUDIO = /\.(mp3|wav|m4a|aac|ogg|oga|opus|flac)$/i;

export const acceptsName = (type: string | undefined, name: string) => (type === 'audio/*' ? AUDIO.test(name) : true);

export async function pickDocument(opts: { type?: string; copyToCacheDirectory?: boolean } = {}): Promise<Result> {
  const p = await getPlatformInfo();
  if (!(p.steamos && p.gaming)) return DocumentPicker.getDocumentAsync(opts);
  return new Promise<Result>((resolve) => {
    useLocalPicker.getState().req?.resolve({ canceled: true, assets: null });
    useLocalPicker.setState({ req: { type: opts.type, resolve } });
  });
}

export function finishLocalPick(file: { path: string; name: string; size: number } | null) {
  const req = useLocalPicker.getState().req;
  useLocalPicker.setState({ req: null });
  if (!req) return;
  if (!file) { req.resolve({ canceled: true, assets: null }); return; }
  req.resolve({
    canceled: false,
    assets: [{ uri: `/local/file?path=${encodeURIComponent(file.path)}`, name: file.name, size: file.size, mimeType: undefined, lastModified: 0 }],
  } as Result);
}

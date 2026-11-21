import { Asset } from 'expo-asset';

export async function assetBytes(mod: number): Promise<ArrayBuffer> {
  const asset = Asset.fromModule(mod);
  await asset.downloadAsync();
  return (await fetch(asset.localUri ?? asset.uri)).arrayBuffer();
}

import { Asset } from 'expo-asset';
// @ts-ignore
import createDecoderModule from 'draco3d/draco_decoder_nodejs.js';

export const DRACO_SUPPORT = { available: true, reason: '' } as const;

export type DecodedCloud = {
  positions: Float32Array;
  reflectivity: Float32Array | null;
  count: number;
};

let modulePromise: Promise<any> | null = null;
function dracoModule(): Promise<any> {
  if (!modulePromise) {
    modulePromise = (async () => {
      // @ts-ignore
      const asset = Asset.fromModule(require('draco3d/draco_decoder.wasm'));
      await asset.downloadAsync();
      const res = await fetch(asset.localUri ?? asset.uri);
      const wasmBinary = await res.arrayBuffer();
      return createDecoderModule({ wasmBinary });
    })();
  }
  return modulePromise;
}

export async function decodeDracoPointCloud(bytes: Uint8Array): Promise<DecodedCloud | null> {
  const draco = await dracoModule();
  const decoder = new draco.Decoder();
  const buf = new draco.DecoderBuffer();
  buf.Init(bytes, bytes.length);
  let pc: any = null;
  try {
    if (decoder.GetEncodedGeometryType(buf) !== draco.POINT_CLOUD) return null;
    pc = new draco.PointCloud();
    const status = decoder.DecodeBufferToPointCloud(buf, pc);
    if (!status.ok() || pc.num_points() === 0) return null;
    const count = pc.num_points();

    const readAttr = (attrType: any, comps: number): Float32Array | null => {
      const id = decoder.GetAttributeId(pc, attrType);
      if (id < 0) return null;
      const attr = decoder.GetAttribute(pc, id);
      const arr = new draco.DracoFloat32Array();
      decoder.GetAttributeFloatForAllPoints(pc, attr, arr);
      const out = new Float32Array(count * comps);
      for (let i = 0; i < out.length; i++) out[i] = arr.GetValue(i);
      draco.destroy(arr);
      return out;
    };

    const positions = readAttr(draco.POSITION, 3);
    if (!positions) return null;
    const reflectivity = readAttr(draco.GENERIC, 1);
    return { positions, reflectivity, count };
  } catch {
    return null;
  } finally {
    if (pc) draco.destroy(pc);
    draco.destroy(buf);
    draco.destroy(decoder);
  }
}

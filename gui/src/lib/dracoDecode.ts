
export const DRACO_SUPPORT = {
  available: false,
  reason: 'native(Hermes)엔 WebAssembly가 없어 Draco 디코드 불가 — LiDAR 포인트클라우드는 웹/데스크탑 전용',
} as const;

export type DecodedCloud = {
  positions: Float32Array;
  reflectivity: Float32Array | null;
  count: number;
};

export async function decodeDracoPointCloud(_bytes: Uint8Array): Promise<DecodedCloud | null> {
  return null;
}

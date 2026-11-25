export function stripTransportCc(sdp: string): string {
  const eol = sdp.includes('\r\n') ? '\r\n' : '\n';
  return sdp
    .split(eol)
    .filter((l) => !/^a=rtcp-fb:\d+ transport-cc/.test(l)
      && !(l.startsWith('a=extmap:') && l.includes('transport-wide-cc')))
    .join(eol);
}

export function robotAnswerSdp(ans: { sdp?: unknown; features?: unknown }): string {
  const sdp = String(ans.sdp);
  return Array.isArray(ans.features) && ans.features.includes('transport-cc') ? sdp : stripTransportCc(sdp);
}

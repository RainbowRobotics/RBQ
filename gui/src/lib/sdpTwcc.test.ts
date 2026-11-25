import { describe, it, expect } from 'vitest';
import { robotAnswerSdp, stripTransportCc } from './sdpTwcc';

const answer = [
  'v=0', 'm=video 9 UDP/TLS/RTP/SAVPF 96',
  'a=extmap:3 http://www.ietf.org/id/draft-holmer-rmcat-transport-wide-cc-extensions-01',
  'a=extmap:6 http://www.webrtc.org/experiments/rtp-hdrext/playout-delay',
  'a=rtpmap:96 H264/90000', 'a=rtcp-fb:96 nack', 'a=rtcp-fb:96 nack pli', 'a=rtcp-fb:96 goog-remb', 'a=rtcp-fb:96 transport-cc', '',
].join('\r\n');

describe('transport-cc — offer 엔 늘 싣고, 로봇이 할 수 있다고 한 answer 에만 남긴다', () => {
  it('strip 은 transport-cc 피드백 줄과 확장만 지우고 나머지(REMB·NACK·PLI·다른 확장)는 둔다', () => {
    const s = stripTransportCc(answer);
    expect(s).not.toMatch(/transport-cc|transport-wide-cc/);
    expect(s).toContain('a=rtcp-fb:96 goog-remb');
    expect(s).toContain('a=rtcp-fb:96 nack pli');
    expect(s).toContain('playout-delay');
    expect(s.endsWith('\r\n')).toBe(true);
  });
  it('줄바꿈이 \\n 뿐인 answer 도 지운다(중계가 줄바꿈을 바꾼 경우) — 못 지우면 옛 로봇에서 REMB 가 끊긴다', () => {
    const s = stripTransportCc(answer.replace(/\r\n/g, '\n'));
    expect(s).not.toMatch(/transport-cc|transport-wide-cc/);
    expect(s).toContain('a=rtcp-fb:96 goog-remb');
  });
  it('features 에 transport-cc 가 있는 로봇의 answer 는 그대로', () => {
    expect(robotAnswerSdp({ sdp: answer, features: ['command-dc', 'spectate', 'transport-cc'] })).toBe(answer);
  });
  it('옛 로봇(features 없음·transport-cc 없음·배열 아님)은 answer 에서 지운다 — 앱이 REMB 로 돌아간다', () => {
    for (const features of [undefined, ['command-dc', 'spectate'], 'transport-cc']) {
      expect(robotAnswerSdp({ sdp: answer, features })).toBe(stripTransportCc(answer));
    }
  });
});

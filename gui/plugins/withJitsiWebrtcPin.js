const { withProjectBuildGradle } = require('expo/config-plugins');

const PIN = 'org.jitsi:webrtc:124.0.0';
const MARK = '// rbq: jitsi-webrtc pin';

const BLOCK = `
${MARK}
allprojects {
  configurations.all {
    resolutionStrategy {
      // react-native-webrtc 의 'org.jitsi:webrtc:124.+' 를 고정 버전으로 바꾼다.
      // 열린 범위가 사라지므로 Gradle 이 jitpack 에 메타데이터를 조회하지 않는다.
      force '${PIN}'
    }
  }
}
`;

module.exports = function withJitsiWebrtcPin(config) {
  return withProjectBuildGradle(config, (cfg) => {
    if (cfg.modResults.language !== 'groovy') {
      throw new Error(`withJitsiWebrtcPin: build.gradle 이 groovy 가 아니다(${cfg.modResults.language})`);
    }
    if (cfg.modResults.contents.includes(MARK)) return cfg;
    cfg.modResults.contents += BLOCK;
    return cfg;
  });
};

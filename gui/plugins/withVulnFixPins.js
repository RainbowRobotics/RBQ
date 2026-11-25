const { withProjectBuildGradle } = require('expo/config-plugins');

const PINS = { 'com.google.code.gson:gson': '2.8.9', 'commons-io:commons-io': '2.14.0' };
const MARK = '// rbq: vuln-fix pins';

const BLOCK = `
${MARK}
def rbqVulnPins = [${Object.entries(PINS).map(([k, v]) => `'${k}': '${v}'`).join(', ')}]
def rbqOlder = { String v, String pin ->
  def a = v.tokenize('.')*.toInteger(), b = pin.tokenize('.')*.toInteger()
  for (int i = 0; i < Math.max(a.size(), b.size()); i++) {
    int x = i < a.size() ? a[i] : 0, y = i < b.size() ? b[i] : 0
    if (x != y) return x < y
  }
  return false
}
allprojects {
  configurations.all {
    resolutionStrategy.eachDependency { d ->
      def key = "\${d.requested.group}:\${d.requested.name}".toString()
      def pin = rbqVulnPins[key]
      if (pin == null) return
      def v = d.requested.version
      if (!(v ==~ /\\d+(\\.\\d+)*/)) throw new GradleException("withVulnFixPins: cannot compare requested version '\${v}' of \${key} - see gui/plugins/withVulnFixPins.js")
      if (rbqOlder(v, pin)) { d.useVersion(pin); d.because('rbq: vuln-fix pin (gui/plugins/withVulnFixPins.js)') }
    }
  }
}
`;

module.exports = function withVulnFixPins(config) {
  return withProjectBuildGradle(config, (cfg) => {
    if (cfg.modResults.language !== 'groovy') {
      throw new Error(`withVulnFixPins: build.gradle 이 groovy 가 아니다(${cfg.modResults.language})`);
    }
    if (cfg.modResults.contents.includes(MARK)) return cfg;
    cfg.modResults.contents += BLOCK;
    return cfg;
  });
};

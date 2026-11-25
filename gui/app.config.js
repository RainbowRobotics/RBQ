const { execSync } = require('child_process');

function gitDescribe() {
  let v = '';
  try {
    v = execSync('git describe --tags --always --dirty', {
      cwd: __dirname, stdio: ['ignore', 'pipe', 'ignore'],
    }).toString().trim();
  } catch {
    return '';
  }
  return /^v?\d+\.\d+\.\d+/.test(v) ? v : '';
}

const CHANNELS = ['release', 'nightly'];
function resolveChannel() {
  const c = process.env.RBQ_CHANNEL;
  if (c) {
    if (!CHANNELS.includes(c)) {
      throw new Error(`RBQ_CHANNEL 값이 잘못됐다: ${c} (가능: ${CHANNELS.join(' / ')})`);
    }
    return c;
  }
  return process.env.RBQ_NIGHTLY === '1' ? 'nightly' : null;
}

module.exports = ({ config }) => {
  const v = (process.env.RBQ_VERSION || gitDescribe()).replace(/^v/, '');
  const channel = resolveChannel();

  if (process.env.RBQ_NIGHTLY === '1') {
    config = {
      ...config,
      name: 'RBQ Nightly',
      scheme: 'rbqnightly',
      icon: './assets/images/icon-nightly.png',
      android: {
        ...config.android,
        package: 'com.rainbowrobotics.rbq.nightly',
        adaptiveIcon: {
          ...config.android.adaptiveIcon,
          foregroundImage: './assets/images/android-icon-foreground-nightly.png',
        },
      },
    };
  }

  if (process.env.RBQ_PLAY === '1') {
    config = {
      ...config,
      android: {
        ...config.android,
        permissions: (config.android?.permissions ?? [])
          .filter((p) => p !== 'android.permission.REQUEST_INSTALL_PACKAGES'),
      },
      extra: { ...(config.extra ?? {}), playBuild: true },
    };
  }

  const fs = require('fs');
  const path = require('path');
  const gone = (f) => f && !fs.existsSync(path.join(__dirname, f));
  if (gone(config.android?.googleServicesFile)) config = { ...config, android: { ...config.android, googleServicesFile: undefined } };
  if (gone(config.ios?.googleServicesFile)) config = { ...config, ios: { ...config.ios, googleServicesFile: undefined } };

  if (channel) config = { ...config, extra: { ...(config.extra ?? {}), channel } };

  if (!v) return config;
  const [maj = 0, min = 0, pat = 0] = v.split('-')[0].split('.').map(Number);
  const code = maj * 1000000 + min * 1000 + pat;
  return {
    ...config,
    version: v,
    android: { ...config.android, ...(code > 0 ? { versionCode: code } : {}) },
  };
};

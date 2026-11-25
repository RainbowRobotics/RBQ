const fs = require('fs');
const path = require('path');

exports.default = async ({ appOutDir }) => {
  const bin = path.join(appOutDir, 'rbq');
  fs.renameSync(bin, `${bin}.bin`);
  fs.writeFileSync(bin, `#!/bin/sh
# Steam injects its overlay (LD_PRELOAD gameoverlayrenderer.so) into non-Steam shortcuts. Inside Electron it hangs the
# browser process (no GPU process, no window) or kills the zygote/GPU process. Drop it, and Steam's input method too
# (same as the Steam build's rbq.sh) so only the app keyboard shows.
unset LD_PRELOAD GTK_IM_MODULE QT_IM_MODULE
exec "$(dirname "$0")/rbq.bin" "$@"
`, { mode: 0o755 });
};

#!/usr/bin/env bash
# rbq_sim setup is unified in scripts/install/isaacsim.bash (the single setup command:
# apt deps + resources symlink + conda env / Isaac Sim / IsaacLab + editable pip installs).
# This shim forwards to it so the per-package path still works. Run as a normal user.
set -e
SCRIPTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPTS_DIR/../../.." && pwd)"
exec bash "$REPO_ROOT/scripts/install/isaacsim.bash" "$@"

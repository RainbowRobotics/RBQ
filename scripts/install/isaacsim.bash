#!/usr/bin/env bash
set -e

# Auto-accept the Omniverse EULA so kit-kernel boots (IsaacLab install, isaacsim imports) run headless.
export OMNI_KIT_ACCEPT_EULA=YES

# Unified setup for the Isaac Sim integration sim (rbq_sim). Two modes:
#
#   bash scripts/install/isaacsim.bash              # FULL host setup (run as a NORMAL user):
#       [1] system/apt deps (sudo)   [2] conda env (rbq-lab) + Isaac Sim 5.1.0 + IsaacLab v2.3.2
#       (delegated to rbq_lab/scripts/setup.bash if missing)
#       [3] editable-install rbq_lab + rbq_sdk_py + rbq_sim
#     -> then run everything with one line: bash scripts/issac_sim.bash
#
#   bash scripts/install/isaacsim.bash --deps-only  # ONLY apt/system deps (Vulkan/GL/X,
#       gnome-terminal, bin/ runtime libs). Run as root by docker/Dockerfile.isaacsim at build
#       time, and invoked via sudo as step [1] of the full setup. Single source of truth for system deps.
#
# INTENTIONALLY INDEPENDENT of the top-level scripts/setup.bash (C++ stack): Isaac's heavy
# conda/Isaac base env is opt-in. Set SKIP_APT=1 to skip step [1] if the host already has the deps.
#
# NOTE: the full runtime-library closure of the bin/ binaries is the list in docker/Dockerfile.mujoco.
# This installs the common subset; if a bin/ app reports a missing .so on first run, add it here.

install_apt() {
    export DEBIAN_FRONTEND=noninteractive
    apt-get update && apt-get install -y --no-install-recommends \
        sudo locales ca-certificates wget git curl \
        gnome-terminal dbus-x11 dconf-cli \
        libcanberra-gtk-module libcanberra-gtk3-module \
        libvulkan1 vulkan-tools libvulkan-dev \
        libglvnd0 libgl1 libglx0 libegl1 libgl1-mesa-dri libglfw3 libglu1-mesa \
        libxkbcommon0 libxkbcommon-x11-0 \
        libxcb1 libxcb-glx0 libxcb-icccm4 libxcb-image0 libxcb-keysyms1 libxcb-randr0 \
        libxcb-render0 libxcb-render-util0 libxcb-shape0 libxcb-shm0 libxcb-sync1 libxcb-util1 \
        libxcb-xfixes0 libxcb-xinerama0 libxcb-xinput0 libxcb-xkb1 \
        libx11-6 libx11-xcb1 libxext6 libxrender1 libxi6 libxrandr2 libxfixes3 libxcursor1 \
        libsm6 libice6 \
        gstreamer1.0-gl gstreamer1.0-libav gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
        gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-x \
        libglib2.0-0
    locale-gen en_US.UTF-8 && dpkg-reconfigure locales
}

# --- deps-only mode (docker build / the sudo'd apt step of the full setup) ---
if [ "${1:-}" = "--deps-only" ]; then
    install_apt
    exit 0
fi

# --- full host setup: must run as the user (conda/pip run as the user, apt is sudo'd) ---
if [ "$EUID" -eq 0 ]; then
    echo "Run the full setup as a normal user (it sudo's the apt step itself). Exiting."
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
# configure.bash exports CONDA_DIR / ENV_NAME / CONDA_PROFILE / LAB_DIR / SIM_DIR / SIMULATOR_DIR.
source "$REPO_ROOT/rbq_simulator/rbq_sim/scripts/configure.bash"

# [1] System/apt deps (sudo). Idempotent. SKIP_APT=1 to skip.
if [ "${SKIP_APT:-0}" != "1" ]; then
    echo "⚙️  [1/3] System deps (sudo): isaacsim.bash --deps-only ..."
    sudo bash "$SCRIPT_DIR/isaacsim.bash" --deps-only
else
    echo "⏭️  [1/3] SKIP_APT=1 — skipping system/apt deps."
fi

# Propagate non-interactive flags into the rbq_lab setup subprocess (keeps rbq_lab unedited): conda ToS
# auto-accept, Omniverse EULA (above), and setuptools<81 so IsaacLab's flatdict==4.0.1 builds (pkg_resources).
export CONDA_PLUGINS_AUTO_ACCEPT_TOS=yes
PIP_BUILD_CONSTRAINTS="${TMPDIR:-/tmp}/rbq_isaac_pip_constraints.txt"
echo "setuptools<81" > "$PIP_BUILD_CONSTRAINTS"
export PIP_CONSTRAINT="$PIP_BUILD_CONSTRAINTS"

# [2] Conda env (rbq-lab) + Isaac Sim 5.1.0 + IsaacLab v2.3.2 — reuse rbq_lab's (single source).
#     Skipped when CONDA_DIR points at a baked docker image.
if [ ! -f "$CONDA_PROFILE" ] || ! ("$CONDA_DIR/bin/conda" env list 2>/dev/null | awk '{print $1}' | grep -Fxq "$ENV_NAME"); then
    echo "⚙️  [2/3] conda env '$ENV_NAME' not found — running rbq_lab setup (Isaac Sim + IsaacLab)..."
    ( cd "$LAB_DIR" && bash scripts/setup.bash )
fi
source "$CONDA_PROFILE"
conda activate "$ENV_NAME"

# [3] Editable installs into the env.
echo "⚙️  [3/3] pip install -e rbq_lab / rbq_sdk_py / rbq_sim ..."
pip install -e "$LAB_DIR"
if [ -d "$REPO_ROOT/rbq_sdk/python" ]; then
    # rbq_sdk_py builds cyclonedds==0.10.2 from sdist, which needs the native lib via CYCLONEDDS_HOME;
    # point it at the C++ stack's CycloneDDS install (/usr/local), honoring an existing value first.
    if [ -z "${CYCLONEDDS_HOME:-}" ]; then
        for d in /usr/local /opt/ros/humble; do
            if [ -f "$d/include/dds/dds.h" ] && ls "$d"/lib*/libddsc.so* >/dev/null 2>&1; then
                export CYCLONEDDS_HOME="$d"; break
            fi
        done
    fi
    if [ -z "${CYCLONEDDS_HOME:-}" ]; then
        echo "⚠️  CycloneDDS not found (set CYCLONEDDS_HOME); rbq_sdk_py build will likely fail." >&2
    else
        echo "ℹ️  Using CYCLONEDDS_HOME=$CYCLONEDDS_HOME for rbq_sdk_py."
    fi
    pip install -e "$REPO_ROOT/rbq_sdk/python"
fi
pip install -e "$SIM_DIR"

# rbq_sdk_py's unpinned numpy/opencv clobber Isaac Sim's hard pins (numpy==1.26.0 etc.); restore them
# last so they win (opencv 4.10 --no-deps keeps numpy 1.x; cv2 is unused by the rbq_sim path).
echo "⚙️  Reconciling deps to Isaac Sim's pins (numpy/click/psutil/torchaudio/opencv) ..."
pip install "numpy==1.26.0" "click==8.1.7" "psutil==5.9.8" "torchaudio==2.7.0"
pip install --no-deps "opencv-python==4.10.0.84"

echo "✅ rbq_sim setup complete. Run with: bash scripts/issac_sim.bash"

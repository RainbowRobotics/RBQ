#!/usr/bin/env bash
set -e

if [ "$EUID" -eq 0 ]; then
    echo "Do not run this script with sudo. Exiting..."
    exit 1
fi

# Dynamically set Docker image name based on Git branch
RAW_BRANCH_NAME=$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo "unknown")
SANITIZED_BRANCH_NAME=$(echo "$RAW_BRANCH_NAME" | sed 's#[/_]#-#g' | tr '[:upper:]' '[:lower:]')
IMAGE_NAME="rbq-examples-${SANITIZED_BRANCH_NAME}"
DOCKER_DIR=".docker"
NO_CACHE=false
NO_CHECK=false
TARGET_ARCH=""          # empty = host native (behaves exactly as before)
CMD_ARGS=()

# Use plain docker if the invoking user can already drive it
if docker info &>/dev/null; then
    DOCKER="docker"
    SUDO=""
else
    DOCKER="sudo docker"
    SUDO="sudo"
fi

cleanup_on_failure() {
    if [[ $? -ne 0 ]]; then
        echo "Build failed. Cleaning up $DOCKER_DIR/$BUILD_OUT and $DOCKER_DIR/$BIN_OUT..."
        $SUDO rm -rf "$DOCKER_DIR/$BUILD_OUT" "$DOCKER_DIR/$BIN_OUT"
    fi
}
trap cleanup_on_failure EXIT

print_help() {
    echo "Usage: bash scripts/docker/run.bash [OPTIONS]"
    echo "Options:"
    echo "  --help                  Display this help message and exit."
    echo "  --no-check              Bypass docker image check."
    echo "  --no-cache              Clean build directory and bypass cache."
    echo "  --arch <amd64|arm64>    Target architecture. DEFAULT host native; a foreign arch auto-registers"
    echo "                          binfmt/QEMU. Output is always bin-<arch> (bin-x86_64, bin-aarch64)."
    echo "  --use-make              Use Make instead of Ninja."
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --help)         print_help; exit 0 ;;
        --no-check)     NO_CHECK=true; shift ;;
        --no-cache)     NO_CACHE=true; shift ;;
        --arch)         TARGET_ARCH="${2:?--arch requires a value}"; shift 2 ;;
        --use-make)     CMD_ARGS+=("--use-make"); shift ;;
        *) echo "Unknown argument: $1"; print_help; exit 1 ;;
    esac
done

# Every architecture is treated alike: its own image, build tree and bin-<arch> output.
HOST_ARCH=$(dpkg --print-architecture 2>/dev/null || echo amd64)
TARGET_ARCH="${TARGET_ARCH:-$HOST_ARCH}"
case "$TARGET_ARCH" in
    amd64) BIN_ARCH=x86_64;  ELF_MACHINE=3e00 ;;
    arm64) BIN_ARCH=aarch64; ELF_MACHINE=b700 ;;
    *) echo "Unknown --arch $TARGET_ARCH (use amd64 or arm64)"; exit 1 ;;
esac
IMAGE_NAME="$IMAGE_NAME-$TARGET_ARCH"
BIN_OUT="bin-$BIN_ARCH"
BUILD_OUT="build-$BIN_ARCH"
PLATFORM_ARGS=(--platform "linux/$TARGET_ARCH")

if [[ "$NO_CACHE" == "true" ]]; then
    echo "No-cache option enabled. Removing $DOCKER_DIR/$BUILD_OUT and $DOCKER_DIR/$BIN_OUT..."
    rm -rf "$DOCKER_DIR/$BUILD_OUT" "$DOCKER_DIR/$BIN_OUT" 2>/dev/null \
        || sudo rm -rf "$DOCKER_DIR/$BUILD_OUT" "$DOCKER_DIR/$BIN_OUT"
fi

if [[ "$NO_CHECK" == "false" && -n "$SUDO" ]]; then
    # --- Remove conflicting Docker packages ---
    NEED_RESTART=false
    if snap list docker &>/dev/null 2>&1; then
        echo "Removing snap Docker to avoid conflicts..."
        sudo snap remove --purge docker
        NEED_RESTART=true
    fi
    for pkg in docker.io docker-doc docker-compose podman-docker; do
        if dpkg -s "$pkg" &>/dev/null 2>&1; then
            echo "Removing conflicting package: $pkg"
            sudo apt-get remove -y "$pkg"
            NEED_RESTART=true
        fi
    done

    # --- Docker CE install ---
    if ! dpkg -s docker-ce &>/dev/null 2>&1; then
        echo "Docker CE not found. Installing..."
        sudo apt-get update
        sudo apt-get install -y ca-certificates curl gnupg
        sudo install -m 0755 -d /etc/apt/keyrings
        curl -fsSL https://download.docker.com/linux/ubuntu/gpg \
            | sudo gpg --dearmor --yes -o /etc/apt/keyrings/docker.gpg
        sudo chmod a+r /etc/apt/keyrings/docker.gpg
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] \
https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
            sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
        sudo apt-get update
        sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin
        NEED_RESTART=true
    fi

    # --- Ensure Docker daemon is running ---
    if [[ "$NEED_RESTART" == "true" ]] || ! sudo docker info &>/dev/null 2>&1; then
        sudo systemctl enable docker &>/dev/null
        sudo systemctl stop docker docker.socket &>/dev/null
        sudo systemctl start docker.socket
        sudo systemctl start docker
    fi
    for i in $(seq 1 10); do
        sudo docker info &>/dev/null && break
        sleep 1
    done
    if ! sudo docker info &>/dev/null; then
        echo "ERROR: Docker daemon failed to start."
        exit 1
    fi
fi

if [[ "$TARGET_ARCH" != "$HOST_ARCH" ]]; then
    case "$TARGET_ARCH" in
        arm64) QEMU_HANDLER="qemu-aarch64" ;;
        amd64) QEMU_HANDLER="qemu-x86_64" ;;
    esac
    BINFMT_ENTRY="/proc/sys/fs/binfmt_misc/$QEMU_HANDLER"
    binfmt_ready() { [[ -e "$BINFMT_ENTRY" ]] && grep -qx "enabled" "$BINFMT_ENTRY" 2>/dev/null; }

    if [[ -n "${DOCKER_HOST:-}" || "$($DOCKER context show 2>/dev/null || echo default)" != "default" ]]; then
        echo "ℹ️  Remote/non-default docker context - skipping binfmt check (local /proc is not that daemon's kernel)."
    elif ! binfmt_ready; then
        echo "binfmt/QEMU handler missing or disabled ($QEMU_HANDLER) - registering now."
        $DOCKER run --privileged --rm tonistiigi/binfmt:latest --install "$TARGET_ARCH" || {
            echo "ERROR: binfmt registration failed."
            echo "       Check whether 'docker run --privileged' is permitted on this host."
            exit 1
        }
        if ! binfmt_ready; then
            echo "ERROR: registration finished but $QEMU_HANDLER is still not ready."
            exit 1
        fi
        echo "✅ binfmt/QEMU registered ($QEMU_HANDLER)"
    fi
fi

if [[ "$NO_CHECK" == "false" ]]; then
    echo "Docker container build starting..."
    $DOCKER build "${PLATFORM_ARGS[@]}" --file scripts/docker/Dockerfile --network host -t $IMAGE_NAME ..
fi
CMD="bash scripts/build.bash ${CMD_ARGS[@]}"
mkdir -p "$DOCKER_DIR/$BIN_OUT" "$DOCKER_DIR/$BUILD_OUT"
$DOCKER run \
    --rm "${PLATFORM_ARGS[@]}" \
    --user $(id -u):$(id -g) \
    -e RBQ_BIN_ARCH="$BIN_ARCH" \
    --cap-add SYS_ADMIN \
    --device /dev/fuse \
    --security-opt apparmor:unconfined \
    --network host \
    -v ${PWD}/$DOCKER_DIR/$BIN_OUT:/workspace/$BIN_OUT \
    -v ${PWD}/$DOCKER_DIR/$BUILD_OUT:/workspace/build \
    -v ${PWD}/src:/workspace/src \
    -v ${PWD}/CMakeLists.txt:/workspace/CMakeLists.txt \
    -v ${PWD}/scripts/build.bash:/workspace/scripts/build.bash \
    $IMAGE_NAME bash -c "$CMD"
trap - EXIT
echo "Docker container executed successfully."
if [ -d $DOCKER_DIR/$BIN_OUT ]; then
    for f in "$DOCKER_DIR/$BIN_OUT"/*; do
        [[ -f "$f" ]] && [[ "$(head -c4 "$f" | od -An -tx1 | tr -d ' ')" == "7f454c46" ]] || continue
        machine=$(od -An -tx1 -j18 -N2 "$f" | tr -d ' ')
        if [[ "$machine" != "$ELF_MACHINE" ]]; then
            echo "ERROR: $(basename "$f") is not $BIN_ARCH (e_machine=$machine)."
            exit 1
        fi
    done
    if [[ -d $BIN_OUT ]]; then
        echo "Removing existing $BIN_OUT directory..."
        rm -rf $BIN_OUT
    fi
    echo "Moving $DOCKER_DIR/$BIN_OUT to $BIN_OUT ..."
    mv "$DOCKER_DIR/$BIN_OUT" "$BIN_OUT"
    echo "✅ Move operation completed successfully."
fi

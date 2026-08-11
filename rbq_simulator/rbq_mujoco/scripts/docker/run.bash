#!/usr/bin/env bash
set -e

if [ "$EUID" -eq 0 ]; then
    echo "Do not run this script with sudo. Exiting..."
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RBQ_SDK_CPP_DIR="$(cd "$SCRIPT_DIR/../../../../rbq_sdk/cpp/rbq_sdk_cpp" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

RAW_BRANCH_NAME=$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo "unknown")
SANITIZED_BRANCH_NAME=$(echo "$RAW_BRANCH_NAME" | sed 's#[/_]#-#g' | tr '[:upper:]' '[:lower:]')
DOCKER_NAME="rbq-mujoco-${SANITIZED_BRANCH_NAME}"

if docker info &>/dev/null; then
    DOCKER="docker"
    SUDO=""
else
    DOCKER="sudo docker"
    SUDO="sudo"
fi

print_help() {
    echo "Usage: bash scripts/docker/run.bash [OPTIONS]"
    echo ""
    echo "Launch a Docker container with the rbq_mujoco mounted, and build the project inside the container."
    echo "X11 GUI forwarding is enabled by default."
    echo ""
    echo "Options:"
    echo "  --help          Display this help message and exit."
    echo "  --no-check      Skip checking if the Docker image exists."
    echo "  --headless      Disable X11 GUI forwarding. (default: false)"
    echo "  --build         Build the Docker image only, do not run the simulator."
    echo "  --test [sec]    Build, then run a headless self-test for [sec] seconds"
    echo "                  (default: 3) and exit with its status. Software GL by default."
    echo "  --gpu           With --test, render on the GPU instead of software GL."
}

HEADLESS=false
NO_CHECK=false
BUILD_ONLY=false
TEST_ONLY=false
USE_GPU=false
SELFTEST_SEC=3

while [[ $# -gt 0 ]]; do
    case "$1" in
        --help)    print_help; exit 0 ;;
        --headless)  HEADLESS=true; shift ;;
        --no-check) NO_CHECK=true; shift ;;
        --build) BUILD_ONLY=true; shift ;;
        --test)
            TEST_ONLY=true; shift
            if [[ $# -gt 0 && "$1" != -* ]]; then SELFTEST_SEC="$1"; shift; fi
            ;;
        --gpu) USE_GPU=true; shift ;;
        *) echo "Unknown argument: $1"; print_help; exit 1 ;;
    esac
done

if [[ "$BUILD_ONLY" == "true" && "$TEST_ONLY" == "true" ]]; then
    echo "ERROR: --build and --test are mutually exclusive."
    exit 1
fi
if [[ "$USE_GPU" == "true" && "$TEST_ONLY" == "false" ]]; then
    echo "WARNING: --gpu only applies to --test; ignoring."
    USE_GPU=false
fi

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
if [[ "$NEED_RESTART" == "true" ]] || ! $DOCKER info &>/dev/null 2>&1; then
    sudo systemctl enable docker &>/dev/null
    sudo systemctl stop docker docker.socket &>/dev/null
    sudo systemctl start docker.socket
    sudo systemctl start docker
fi
for i in $(seq 1 10); do
    $DOCKER info &>/dev/null && break
    sleep 1
done
if ! $DOCKER info &>/dev/null; then
    echo "ERROR: Docker daemon failed to start."
    exit 1
fi

# --- NVIDIA Container Toolkit install ---
if ! dpkg -s nvidia-container-toolkit &>/dev/null; then
    echo "NVIDIA Container Toolkit not found. Installing..."
    curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
        sudo gpg --dearmor --yes -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
    curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
        sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
        sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list > /dev/null
    sudo apt-get update
    sudo apt-get install -y nvidia-container-toolkit
    sudo nvidia-ctk runtime configure --runtime=docker
    sudo systemctl stop docker docker.socket &>/dev/null
    sudo systemctl start docker.socket
    sudo systemctl start docker
fi

# --- X11 / GUI ---
# --build/--test 는 호스트 X11을 쓰지 않는다. DISPLAY_ARGS는 맨 아래 대화형 실행에만
# 쓰이고, --test 는 컨테이너 안에서 xvfb로 렌더한다. 그런데도 여기를 타면 CI에서
# **무한 대기**한다 — 러너가 데스크톱 세션에서 기동되면 DISPLAY=:0 과 gdm의
# XAUTHORITY를 물려받는데, 그 디스플레이에 붙지 못하는 xhost가 영원히 안 끝난다
# (실측: exit 124로 타임아웃, 로그 한 줄 없이 멈춤 → run 30592068328 3시간 타임아웃).
# 그래서 GUI가 실제로 필요할 때만 X11을 건드린다. xhost 자체에도 타임아웃을 건다.
DISPLAY_ARGS=()
if [[ "$HEADLESS" == "false" && "$TEST_ONLY" == "false" && "$BUILD_ONLY" == "false" ]]; then
    timeout 5 xhost +local:docker &>/dev/null || true
    DISPLAY_ARGS+=(
        -e DISPLAY="$DISPLAY"
        -v /tmp/.X11-unix:/tmp/.X11-unix
    )
    if [[ -n "$XAUTHORITY" && -f "$XAUTHORITY" ]]; then
        DISPLAY_ARGS+=(
            -e XAUTHORITY="$XAUTHORITY"
            -v "$XAUTHORITY":"$XAUTHORITY":ro
        )
    fi
fi

# --- GPU support ---
if command -v nvidia-smi &>/dev/null && nvidia-smi &>/dev/null; then
    DISPLAY_ARGS+=(--gpus all)
fi

# --- Image build ---
if [[ "$NO_CHECK" == "false" ]]; then
    echo "Docker images build starting..."
    mkdir -p $PROJECT_DIR/temp && cp -r "$RBQ_SDK_CPP_DIR" $PROJECT_DIR/temp/
    $DOCKER build --file "$SCRIPT_DIR/Dockerfile" --network host -t "$DOCKER_NAME" .
    rm -rf $PROJECT_DIR/temp
fi

if [[ "$BUILD_ONLY" == "true" ]]; then
    echo "Docker image built successfully."
    exit 0
fi

if [[ "$TEST_ONLY" == "true" ]]; then
    TEST_ARGS=()
    if [[ "$USE_GPU" == "true" ]]; then
        echo "Running headless self-test (${SELFTEST_SEC}s, GPU rendering)..."
        TEST_ARGS+=(--gpus all)
    else
        echo "Running headless self-test (${SELFTEST_SEC}s, software GL)..."
        TEST_ARGS+=(-e LIBGL_ALWAYS_SOFTWARE=1)
    fi
    # GLFW needs an X display even when rendering offscreen; provide one via Xvfb.
    # Docker options (incl. --entrypoint and TEST_ARGS) MUST precede the image
    # name; the binary's own args follow it. --init reaps Xvfb so the container
    # exits cleanly (xvfb-run as PID 1 otherwise hangs after the binary exits).
    # No -it: release.bash runs this non-interactively.
    set +e
    $DOCKER run --rm --init \
        --cap-add SYS_ADMIN \
        --network host \
        "${TEST_ARGS[@]}" \
        --entrypoint xvfb-run \
        "$DOCKER_NAME" \
        -a /workspace/bin/rbq_mujoco -p /workspace/resources/model/default.xml --selftest "$SELFTEST_SEC"
    STATUS=$?
    set -e
    if [[ $STATUS -eq 0 ]]; then
        echo "✅ Self-test passed."
    else
        echo "❌ Self-test failed (exit $STATUS)."
    fi
    exit $STATUS
fi

$DOCKER run -it \
    --cap-add SYS_ADMIN \
    --network host \
    "${DISPLAY_ARGS[@]}" \
    "$DOCKER_NAME"

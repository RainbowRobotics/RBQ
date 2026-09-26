#!/usr/bin/env bash
set -e

if [ "$EUID" -eq 0 ]; then
    echo "Do not run this script with sudo. Exiting..."
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"              # rbq_gym/
RESOURCES_DIR="$(cd "$PROJECT_DIR/../../resources" && pwd)" # for urdf

RAW_BRANCH_NAME=$(git -C "$PROJECT_DIR" rev-parse --abbrev-ref HEAD 2>/dev/null || echo "unknown")
SANITIZED_BRANCH_NAME=$(echo "$RAW_BRANCH_NAME" | sed 's#[/_]#-#g' | tr '[:upper:]' '[:lower:]')
DOCKER_NAME="rbq-gym-${SANITIZED_BRANCH_NAME}"

if docker info &>/dev/null; then
    DOCKER="docker"
else
    DOCKER="sudo docker"
fi

print_help() {
    echo "Usage: bash scripts/docker/run.bash [OPTIONS] [train/play args...]"
    echo ""
    echo "Options:"
    echo "  --help          Display this help message and exit."
    echo "  --no-check      Skip checking/building the Docker image; run directly."
    echo "  --build         Build the Docker image only, do not run."
    echo "  --no-cache      Force a clean image rebuild (bypass Docker layer cache)."
    echo "  --play          Run scripts/play.bash instead of scripts/train.bash."
    echo "  --shell         Drop into an interactive shell in the container instead."
    echo "  All other arguments are forwarded to train.bash/play.bash (--task, --num_envs, --headless, ...)."
}

NO_CHECK=false
BUILD_ONLY=false
NO_CACHE_ARG=()
RUN_TARGET="train"
CMD_ARGS=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --help) print_help; exit 0 ;;
        --no-check) NO_CHECK=true; shift ;;
        --build) BUILD_ONLY=true; shift ;;
        --no-cache) NO_CACHE_ARG=(--no-cache); shift ;;
        --play) RUN_TARGET="play"; shift ;;
        --shell) RUN_TARGET="shell"; shift ;;
        *) CMD_ARGS+=("$1"); shift ;;
    esac
done

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

# --- GPU check ---
if ! command -v nvidia-smi &>/dev/null || ! nvidia-smi &>/dev/null; then
    echo "ERROR: No NVIDIA GPU detected (nvidia-smi failed). IsaacGym requires a GPU — aborting."
    exit 1
fi

# --- Image build ---
if [[ "$NO_CHECK" == "false" ]]; then
    echo "[gym-docker] Building image: $DOCKER_NAME"
    $DOCKER build "${NO_CACHE_ARG[@]}" --file "$SCRIPT_DIR/Dockerfile" --network host -t "$DOCKER_NAME" "$PROJECT_DIR"
fi

if [[ "$BUILD_ONLY" == "true" ]]; then
    echo "✅ Docker image built successfully."
    exit 0
fi

mkdir -p "$PROJECT_DIR/.docker/home" "$PROJECT_DIR/logs"

DISPLAY_ARGS=()
HAS_DISPLAY=false
if [[ -n "${DISPLAY:-}" && -S "/tmp/.X11-unix/X${DISPLAY#:}" ]]; then
    HAS_DISPLAY=true
fi

if [[ "$HAS_DISPLAY" == "true" ]]; then
    timeout 5 xhost +local:docker &>/dev/null || true
    DISPLAY_ARGS+=(-e DISPLAY="$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix)
    if [[ -n "${XAUTHORITY:-}" && -f "$XAUTHORITY" ]]; then
        DISPLAY_ARGS+=(-e XAUTHORITY="$XAUTHORITY" -v "$XAUTHORITY":"$XAUTHORITY":ro)
    fi
elif [[ "$RUN_TARGET" != "shell" ]]; then
    if [[ ! " ${CMD_ARGS[*]} " == *" --headless "* ]]; then
        echo "[gym-docker] No usable DISPLAY on host — adding --headless (viewer/keyboard control need a real X server)."
        CMD_ARGS+=(--headless)
    fi
fi

case "$RUN_TARGET" in
    train) CONTAINER_CMD="bash scripts/train.bash ${CMD_ARGS[*]}" ;;
    play)  CONTAINER_CMD="bash scripts/play.bash ${CMD_ARGS[*]}" ;;
    shell) CONTAINER_CMD="bash" ;;
esac

echo "[gym-docker] Running: $CONTAINER_CMD"
$DOCKER run --rm -it \
    --gpus all \
    --network host \
    --user "$(id -u):$(id -g)" \
    -e HOME=/workspace/cache/home \
    "${DISPLAY_ARGS[@]}" \
    -v "$PROJECT_DIR":/workspace/rbq_simulator/rbq_gym \
    -v "$RESOURCES_DIR":/workspace/resources:ro \
    -v "$PROJECT_DIR/.docker":/workspace/cache \
    "$DOCKER_NAME" bash -c "cd /workspace/rbq_simulator/rbq_gym && $CONTAINER_CMD"

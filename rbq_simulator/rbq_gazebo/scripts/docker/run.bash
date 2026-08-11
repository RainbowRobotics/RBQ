#!/usr/bin/env bash
set -e

if [ "$EUID" -eq 0 ]; then
    echo "Do not run this script with sudo. Exiting..."
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
RBQ_SDK_CPP_DIR="$(cd "$SCRIPT_DIR/../../../../rbq_sdk/cpp/rbq_sdk_cpp" && pwd)"

# Image/container name derived from the git branch (matches rbq_mujoco).
RAW_BRANCH_NAME=$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo "unknown")
SANITIZED_BRANCH_NAME=$(echo "$RAW_BRANCH_NAME" | sed 's#[/_]#-#g' | tr '[:upper:]' '[:lower:]')
DOCKER_NAME="rbq-gazebo-${SANITIZED_BRANCH_NAME}"

print_help() {
    echo "Usage: bash scripts/docker/run.bash [OPTIONS] [-- SIM_ARGS]"
    echo ""
    echo "Build the rbq_gazebo simulator image and run the Gazebo simulator"
    echo ""
    echo "Options:"
    echo "  --help          Display this help message and exit."
    echo "  --no-check      Skip the image build/exists check (run the existing image)."
    echo "  --headless      Disable X11 forwarding and run the simulator headless."
    echo ""
    echo "Any other argument (or anything after --) is forwarded to the simulator, e.g.:"
    echo "  --world <name>         ex) empty/raceway/depot/warehouse/maze"
    echo "  --livox / --ouster     LiDAR"
}

HEADLESS=false
NO_CHECK=false
SIM_ARGS=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --help)     print_help; exit 0 ;;
        --no-check) NO_CHECK=true; shift ;;
        --headless) HEADLESS=true; SIM_ARGS+=("--headless"); shift ;;
        --)         shift; SIM_ARGS+=("$@"); break ;;
        *)          SIM_ARGS+=("$1"); shift ;;   # forward everything else to the simulator
    esac
done

if docker info &>/dev/null; then
    DOCKER="docker"
else
    DOCKER="sudo docker"
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

# --- Image build ---
if [[ "$NO_CHECK" == "false" ]]; then
    echo "Docker images build starting..."
    mkdir -p $PROJECT_DIR/temp && cp -r "$RBQ_SDK_CPP_DIR" $PROJECT_DIR/temp/
    $DOCKER build --file "$SCRIPT_DIR/Dockerfile" --network host -t "$DOCKER_NAME" .
    rm -rf $PROJECT_DIR/temp
fi

# --- X11 / GUI ---
DISPLAY_ARGS=()
if [[ "$HEADLESS" == "false" ]]; then
    xhost +local:docker 2>/dev/null || true
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
    DISPLAY_ARGS+=(--gpus all -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all)
fi

echo "Running simulator: $DOCKER_NAME ${SIM_ARGS[*]}"
$DOCKER run -it \
    --cap-add SYS_ADMIN \
    --cap-add NET_ADMIN \
    --network host \
    -e NO_AT_BRIDGE=1 \
    "${DISPLAY_ARGS[@]}" \
    "$DOCKER_NAME" "${SIM_ARGS[@]}"

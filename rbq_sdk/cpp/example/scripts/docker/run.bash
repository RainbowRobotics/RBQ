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
CMD_ARGS=()

cleanup_on_failure() {
    if [[ $? -ne 0 ]]; then
        echo "Build failed. Cleaning up $DOCKER_DIR..."
        sudo chown -R $(logname):$(logname) ${PWD}
        rm -rf "$DOCKER_DIR"
    fi
}
trap cleanup_on_failure EXIT

print_help() {
    echo "Usage: bash scripts/docker/run.bash [OPTIONS]"
    echo "Options:"
    echo "  --help                  Display this help message and exit."
    echo "  --no-check              Bypass docker image check."
    echo "  --no-cache              Clean build directory and bypass cache."
    echo "  --use-make              Use Make instead of Ninja."
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --help)         print_help; exit 0 ;;
        --no-check)     NO_CHECK=true; shift ;;
        --no-cache)     NO_CACHE=true; shift ;;
        --use-make)     CMD_ARGS+=("--use-make"); shift ;;
        *) echo "Unknown argument: $1"; print_help; exit 1 ;;
    esac
done

if [[ "$NO_CACHE" == "true" ]]; then
    echo "No-cache option enabled. Removing $DOCKER_DIR..."
    sudo rm -rf "$DOCKER_DIR"
fi
if [[ "$NO_CHECK" == "false" ]]; then
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

    echo "Docker container build starting..."
    sudo docker build --file scripts/docker/Dockerfile --network host -t $IMAGE_NAME ..
fi
CMD="bash scripts/build.bash ${CMD_ARGS[@]}"
sudo docker run \
    --rm \
    --cap-add SYS_ADMIN \
    --device /dev/fuse \
    --security-opt apparmor:unconfined \
    --network host \
    -v ${PWD}/$DOCKER_DIR/bin:/workspace/bin \
    -v ${PWD}/$DOCKER_DIR/build:/workspace/build \
    -v ${PWD}/src:/workspace/src \
    -v ${PWD}/CMakeLists.txt:/workspace/CMakeLists.txt \
    -v ${PWD}/scripts/build.bash:/workspace/scripts/build.bash \
    $IMAGE_NAME bash -c "$CMD"
trap - EXIT
echo "Docker container executed successfully."
sudo chown -R $(logname):$(logname) ${PWD}/..
if [[ -d bin ]]; then
    echo "Removing existing bin directory..."
    rm -rf bin
fi
echo "Copying $DOCKER_DIR/bin to bin ..."
if [ -d $DOCKER_DIR/bin ]; then
    mv $DOCKER_DIR/bin .
fi
echo "Copy operation completed successfully."

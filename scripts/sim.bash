#!/usr/bin/env bash
# Simulate the quadruped in Docker, from a release tree. See --help.
# Same options as this repository's scripts/sim.bash; the public tree gets this one instead
# (mirror_sync.bash copies scripts/rbq/ up to scripts/). Pattern taken from scripts/rbh/ on develop.
set -euo pipefail

SHELL_ONLY=false
NO_BUILD=false

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SELF="${BASH_SOURCE[0]}"
IMAGE="rbq-sim"
CONTAINER="rbq_sim"
IFACE="lo"
ROBOT=""
PAYLOAD=""
VISION=false
SLAM=false
MOTION=true
SIM=true
BIN_DIR="bin-$(uname -m)"

print_help() {
    cat <<EOF
Usage: bash $SELF [OPTIONS]

Runs the simulated RBQ in a container: Motion --sim (which starts Network) and Mujoco, plus
Vision and mediamtx with --vision and SLAM with --slam. The image carries the Ubuntu 22.04
libraries the released binaries link against, so nothing has to be installed on this PC.

Options:
  --help                  Display this help message and exit.
  -i, --interface <name>  CycloneDDS network interface. DEFAULT lo
  -r, --robot <flag>      Robot variant: rb1 | wheel | lims_ex
  -p, --payload <name>    Attach a payload (ptz|livox|ouster).
  --no-motion             Skip the Motion.
  --no-sim                Skip the Mujoco simulator.
  --vision                Run the vision modules (Vision, mediamtx).
  --slam                  Run SLAM.
  --no-gui                Accepted for compatibility; this tree has no motion GUI.
  --shell                 Open a shell in the image instead of starting the simulation.
  --no-build              Skip the image build (use the existing $IMAGE image).

Operator UI   the RBQ app, connected to localhost. Network serves the robot's REST/WS on the
              host's ports.
Control       from this PC with the SDK, over DDS on the chosen interface.
Stopping      Ctrl-C stops everything. Logs land in log/sim-docker/.
Display       the simulator draws on your X server (XWayland on Wayland); the script allows
              local clients itself. If no window appears, it prints what Mujoco said. Snap
              Docker cannot forward X at all - use Docker CE.
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --help)         print_help; exit 0 ;;
        -i|--interface) IFACE="$2"; shift 2 ;;
        -r|--robot)     ROBOT="${2#--}"; shift 2 ;;
        -p|--payload)   PAYLOAD="$2"; shift 2 ;;
        --vision)       VISION=true; shift ;;
        --slam)         SLAM=true; shift ;;
        --no-motion)    MOTION=false; shift ;;
        --no-sim)       SIM=false; shift ;;
        --no-gui)       shift ;;
        --rviz)         echo "--rviz needs ROS 2 Humble and the rbq_description workspace, which this"
                        echo "Docker simulation does not carry. Run RViz on the host against the simulation."; exit 1 ;;
        --shell)        SHELL_ONLY=true; shift ;;
        --no-build)     NO_BUILD=true; shift ;;
        *) echo "Unknown argument: $1"; print_help; exit 1 ;;
    esac
done
case "$ROBOT" in ""|rb1|wheel|lims_ex) ;; *) echo "Unknown robot: $ROBOT (rb1|wheel|lims_ex)"; exit 1 ;; esac
case "$PAYLOAD" in ""|ptz|livox|ouster) ;; *) echo "Unknown payload: $PAYLOAD (ptz|livox|ouster)"; exit 1 ;; esac

# The tree to simulate is the one holding bin-<arch>: its scripts/ here, scripts/rbq/ in this
# repository - so search upwards rather than count directories.
ROOT_DIR="$SCRIPT_DIR"
until [[ -d "$ROOT_DIR/$BIN_DIR" ]]; do
    [[ "$ROOT_DIR" == "/" ]] && { echo "ERROR: no $BIN_DIR/ above $SCRIPT_DIR - is this a release tree for $(uname -m)?"; exit 1; }
    ROOT_DIR="$(dirname "$ROOT_DIR")"
done
cd "$ROOT_DIR"

[[ -x "$BIN_DIR/Motion" && -x "$BIN_DIR/Mujoco" ]] \
    || { echo "ERROR: $BIN_DIR/Motion or Mujoco missing - is this a release tree for $(uname -m)?"; exit 1; }

# Yes to a question, but only when someone is there to answer it: no terminal means No, quietly.
confirm() {
    local reply
    { read -r -p "$1 [y/N] " reply < /dev/tty; } 2>/dev/null || return 1
    [[ "$reply" == [yY] || "$reply" == [yY][eE][sS] ]]
}

# Replaces snap Docker (or no Docker) with Docker CE from Docker's own apt repository - the same
# steps Rainbow's developer setup runs. Asked for first, never done behind the user's back.
install_docker_ce() {
    echo "[INFO] Installing Docker CE (sudo will ask for your password)..."
    snap list docker &>/dev/null && sudo snap remove --purge docker
    for pkg in docker.io docker-doc docker-compose podman-docker; do
        dpkg -s "$pkg" &>/dev/null && sudo apt-get remove -y "$pkg"
    done
    sudo apt-get update
    sudo apt-get install -y ca-certificates curl gnupg
    sudo install -m 0755 -d /etc/apt/keyrings
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg \
        | sudo gpg --dearmor --yes -o /etc/apt/keyrings/docker.gpg
    sudo chmod a+r /etc/apt/keyrings/docker.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] \
https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo "$VERSION_CODENAME") stable" \
        | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
    sudo apt-get update
    sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin
    sudo systemctl enable docker &>/dev/null || true
    sudo systemctl start docker.socket docker
    DOCKER="docker"
    docker info &>/dev/null || DOCKER="sudo docker"
    $DOCKER info &>/dev/null || { echo "ERROR: Docker still does not answer after the install."; exit 1; }
    echo "[INFO] Docker CE is up."
}

# Ends this session so a new one picks up the group. gnome-session-quit asks the desktop to log
# out; loginctl is the fallback on other desktops and on a plain console.
end_session() {
    if command -v gnome-session-quit &>/dev/null; then
        gnome-session-quit --logout --no-prompt && exit 0
    fi
    if command -v loginctl &>/dev/null; then
        loginctl terminate-user "$USER" && exit 0
    fi
    echo "Could not end the session from here."
    confirm "Reboot now instead?" && { sudo reboot; exit 0; }
    echo "Log out by hand when convenient - until then this script uses sudo for Docker."
}

# Needing sudo for Docker means this user is outside the 'docker' group - whether this script just
# installed Docker or someone else did. The group is only read when a session starts, so fixing it
# properly means logging in again.
offer_docker_group() {
    id -nG "$USER" 2>/dev/null | tr ' ' '\n' | grep -qx docker && return 0
    echo
    echo "Docker commands need sudo until your user belongs to the 'docker' group."
    echo "Note that membership of that group is equivalent to root access on this machine."
    confirm "Add $USER to the 'docker' group?" || { echo "Leaving it - this script will use sudo."; return 0; }
    sudo usermod -aG docker "$USER" || { echo "usermod failed - carrying on with sudo."; return 0; }
    echo "Added. Linux reads group membership at login, so it applies to your next session."
    echo "Logging out or rebooting now ends this terminal, and the simulation will not start."
    if confirm "Log out now?"; then
        end_session
    elif confirm "Reboot now?"; then
        sudo reboot
        exit 0
    else
        echo "Fine - the group applies after your next login. Continuing with sudo for now."
    fi
}

# Only Ubuntu/Debian can be fixed from here; elsewhere the docs are the answer.
can_install() { command -v apt-get &>/dev/null && [[ -e /etc/os-release ]]; }

DOCKER="docker"
docker info &>/dev/null || DOCKER="sudo docker"
if ! $DOCKER info &>/dev/null; then
    echo "Docker does not answer - it is either not installed, or its daemon is stopped."
    echo "This script needs it: the simulation runs in a container built from scripts/Dockerfile."
    if can_install && confirm "Install Docker CE now (removes snap/older docker packages)?"; then
        install_docker_ce
    else
        echo "ERROR: no Docker. Start it with 'sudo systemctl start docker', or install it:"
        echo "       https://docs.docker.com/engine/install/"
        exit 1
    fi
fi

# Snap Docker runs with a private /tmp, so the X socket bind-mounted below lands nowhere and the
# simulator window never appears - the run looks fine and nothing is drawn. It cannot be worked
# around from here; the only fix is a Docker that can see the real /tmp.
if $DOCKER info --format '{{.DockerRootDir}}' 2>/dev/null | grep -q '/var/snap/'; then
    echo "This is snap Docker. Its /tmp belongs to the snap, so the X socket (/tmp/.X11-unix) cannot"
    echo "be passed into the container and the simulator window will never appear."
    echo "The fix is Docker CE from Docker's apt repository: snap Docker is removed, docker.io and"
    echo "friends are removed, and docker-ce is installed. Images you built under snap Docker are"
    echo "not carried over - this script rebuilds its own."
    if can_install && confirm "Replace snap Docker with Docker CE now?"; then
        install_docker_ce
    else
        echo "WARNING: continuing on snap Docker - expect no simulator window."
        echo "         https://docs.docker.com/engine/install/ubuntu/"
        echo
    fi
fi

# Reaching Docker only through sudo is worth fixing once, rather than every run.
[[ "$DOCKER" == "sudo docker" ]] && offer_docker_group

XAUTH=""
# X11 only when Mujoco runs: --no-sim draws nothing, and a headless host (SSH, VM, CI) must still
# be able to run the stack for an external simulator.
if [[ "$SIM" == "true" ]]; then
    # The simulator is a window, so no display is a failure worth naming rather than a black screen.
    if [[ -z "${DISPLAY:-}" ]]; then
        echo "ERROR: DISPLAY is not set - the simulator has nowhere to draw."
        [[ -n "${WAYLAND_DISPLAY:-}" ]] && echo "       This is a Wayland session; XWayland provides DISPLAY (usually :0)."
        exit 1
    fi
    [[ -d /tmp/.X11-unix ]] || {
        echo "ERROR: /tmp/.X11-unix does not exist - no X server to draw on."
        exit 1
    }
    # The container connects as root, which the X server rejects unless local clients are allowed.
    xhost +local:docker &>/dev/null || true

    # The cookie: $XAUTHORITY when set (GDM keeps it under /run/user/<uid>), ~/.Xauthority otherwise.
    # Mount it where it already is, so the path inside matches what XAUTHORITY says.
    XAUTH="${XAUTHORITY:-$HOME/.Xauthority}"
fi

# The image recipe travels with this script, wherever it sits.
DOCKERFILE="$SCRIPT_DIR/Dockerfile"
[[ -f "$DOCKERFILE" ]] || { echo "ERROR: no Dockerfile next to $SELF"; exit 1; }

if [[ "$NO_BUILD" == "false" ]]; then
    echo "[INFO] Building the $IMAGE image (first time only, a few minutes)..."
    $DOCKER build -f "$DOCKERFILE" -t "$IMAGE" "$SCRIPT_DIR"
fi

# A previous run may have left the container behind (killed rather than stopped). Clear a dead one
# out of the way; refuse to fight a live one.
if $DOCKER ps -q -f name="^$CONTAINER$" | grep -q .; then
    echo "ERROR: a simulation is already running (container $CONTAINER). Stop it first:"
    echo "       $DOCKER stop $CONTAINER"
    exit 1
fi
$DOCKER rm $CONTAINER >/dev/null 2>&1 || true

mkdir -p log/sim-docker

# What runs inside: the binaries directly, with the flags scripts/start_*.bash would pass.
# Motion --sim brings up Network. Motion spells the arm --arm; Mujoco takes the robot flag as is.
read -r -d '' INNER <<'EOS' || true
set -u
cd "$BIN_DIR"
# Motion refuses to run as non-root, so the apps write as root into the mounted tree. Give the
# logs back to the user who started this, otherwise their own tree needs sudo to clean.
give_back() { chown -R "$HOST_UID:$HOST_GID" ../log ../configs 2>/dev/null || true; }   # the apps also rewrite configs/*.ini
cleanup() { kill $(jobs -p) 2>/dev/null; give_back; }
trap 'cleanup; exit 0' INT TERM   # Ctrl-C is how this is meant to end
trap cleanup EXIT                 # anything else keeps its own status

IF=(--interface "$SIM_IFACE")
MOTION=(--sim "${IF[@]}"); MUJOCO=("${IF[@]}")
case "$SIM_ROBOT" in
    rb1|lims_ex) MOTION+=(--arm);   MUJOCO+=("--$SIM_ROBOT") ;;
    wheel)       MOTION+=(--wheel); MUJOCO+=(--wheel) ;;
esac
# ptz is a payload Motion drives; lidars only exist in the simulator (as in scripts/sim.bash).
case "$SIM_PAYLOAD" in
    ptz)          MOTION+=(--payload ptz); MUJOCO+=(--payload ptz) ;;
    livox|ouster) MUJOCO+=(--payload "$SIM_PAYLOAD") ;;
esac
[[ "$SIM_VISION" == true ]] && MUJOCO+=(--vision)

if [[ "$SIM_MOTION" == true ]]; then
    ./Motion "${MOTION[@]}" > ../log/sim-docker/motion.log 2>&1 &
    echo "[sim] Motion ${MOTION[*]} (log/sim-docker/motion.log)"
fi

# Nothing to wait for on the DDS side - discovery pairs them whenever each side comes up - but X
# needs this. On a display with no direct rendering (Xvfb, a remote desktop) opening Mujoco's
# window while Motion's stack is still starting makes its first MIT-SHM request fail
# ("X Error ... X_ShmPutImage") and Mujoco dies with no window, which looks like Docker's fault.
sleep 3

mujoco=""
if [[ "$SIM_SIM" == true ]]; then
    ./Mujoco "${MUJOCO[@]}" > ../log/sim-docker/mujoco.log 2>&1 &
    mujoco=$!
    echo "[sim] Mujoco ${MUJOCO[*]} (log/sim-docker/mujoco.log)"
fi

if [[ "$SIM_VISION" == true ]]; then
    (cd ../resources/mediamtx && exec ./mediamtx) > ../log/sim-docker/mediamtx.log 2>&1 &
    echo "[sim] mediamtx (log/sim-docker/mediamtx.log)"
    ./Vision --sim "${IF[@]}" > ../log/sim-docker/vision.log 2>&1 &
    echo "[sim] Vision --sim (log/sim-docker/vision.log)"
fi
if [[ "$SIM_SLAM" == true ]]; then
    ./SLAMNAV_3D "${IF[@]}" > ../log/sim-docker/slam.log 2>&1 &
    echo "[sim] SLAMNAV_3D (log/sim-docker/slam.log)"
fi
sleep 3

# The window is the whole point, and when it fails to open Mujoco exits within a second or two.
# Say what it said, instead of leaving a running simulation nobody can see.
if [[ -n "$mujoco" ]] && ! kill -0 $mujoco 2>/dev/null; then
    echo "[sim] Mujoco exited - no simulator window. Its last words:"
    tail -5 ../log/sim-docker/mujoco.log | sed 's/^/[sim]   /'
    echo "[sim] Usually X: the container could not reach your display (snap Docker, no xhost, or"
    echo "[sim]   a Wayland session without XWayland). See bash $SIM_SCRIPT --help."
    exit 1   # no window, no simulation - do not leave Motion running as if there were one
fi

echo "[sim] running - Ctrl-C to stop"
wait
EOS

DOCKER_ARGS=(
    --rm
    --name "$CONTAINER"
    # Motion asks for real-time scheduling; without these it runs but warns.
    --cap-add SYS_NICE --ulimit rtprio=99
    # DDS on loopback: host networking lets the SDK examples on the PC reach the simulated robot.
    --network host
    # No --ipc host on purpose. The apps' shared memory (and Qt's single-instance guard) then stay
    # inside the container: a killed run cannot block the next one, nor disturb shm on the host.
    -e DISPLAY="${DISPLAY:-}"
    -e BIN_DIR="$BIN_DIR"
    -e SIM_SCRIPT="$SELF"
    -e HOST_UID="$(id -u)"
    -e HOST_GID="$(id -g)"
    -e SIM_IFACE="$IFACE"
    -e SIM_ROBOT="$ROBOT"
    -e SIM_PAYLOAD="$PAYLOAD"
    -e SIM_VISION="$VISION"
    -e SIM_SLAM="$SLAM"
    -e SIM_MOTION="$MOTION"
    -e SIM_SIM="$SIM"
    -v "$ROOT_DIR":/rbq_ws
)
# -t only with a real terminal: without it docker refuses ("stdin is not a terminal") in CI or pipes.
[[ -t 0 ]] && DOCKER_ARGS+=(-it) || DOCKER_ARGS+=(-i)
[[ -d /tmp/.X11-unix ]] && DOCKER_ARGS+=(-v /tmp/.X11-unix:/tmp/.X11-unix)
[[ -n "$XAUTH" && -f "$XAUTH" ]] && DOCKER_ARGS+=(-e XAUTHORITY="$XAUTH" -v "$XAUTH":"$XAUTH":ro)
# Pass the GPU through when the host has one; software GL otherwise. --gpus needs the NVIDIA
# container toolkit registered with Docker - asking for it without that fails the whole run.
[[ -d /dev/dri ]] && DOCKER_ARGS+=(--device /dev/dri)
# The toolkit can be registered on a PC with no NVIDIA driver (a build server, a GPU removed later):
# then --gpus fails the whole run (libnvidia-ml.so.1 not found), so ask the driver too.
$DOCKER info --format '{{.Runtimes}}' 2>/dev/null | grep -q nvidia \
    && nvidia-smi -L &>/dev/null && DOCKER_ARGS+=(--gpus all)

if [[ "$SHELL_ONLY" == "true" ]]; then
    exec $DOCKER run "${DOCKER_ARGS[@]}" "$IMAGE" bash
fi

exec $DOCKER run "${DOCKER_ARGS[@]}" "$IMAGE" bash -c "$INNER"

#!/bin/bash

APP_NAME="Mujoco"
APP_PATH="bin-$(uname -m)"
# The nightly ships bin-x86_64/ and bin-aarch64/ side by side; pick this machine's.
# build.bash (host, devcontainer, or via docker/run.bash) always writes bin-<arch>/
# directly - fail loudly rather than falling through to a confusing "binary not found".
if [ ! -d "$APP_PATH" ]; then
    echo "ERROR: $APP_PATH not found - this build carries no binaries for $(uname -m)." >&2
    exit 1
fi
CMD_ARGS=()

echo -ne "\033]0;$APP_NAME\007"

print_help() {
    echo "Usage: bash scripts/start_mujoco.bash [OPTIONS]"
    echo "Options:"
    echo "  --help                    Display this help message and exit."
    echo "  --rb1                     RB1 Arm enabled. DEFAULT false"
    echo "  --rba-1                   RBA-1 Arm enabled. DEFAULT false"
    echo "  --lims_ex                 LIMS_EX enabled. DEFAULT false"
    echo "  --wheel                   RBQ WHEEL enabled. DEFAULT false"
    echo "  --platform <name>         Robot platform: model ../resources/model/<name>_environment.xml (rbq10 = rbq_*). DEFAULT rbq10"
    echo "  --rbh                     RBH humanoid (RBH_SCH_v1) instead of RBQ. DEFAULT false"
    echo "  --vision                  Vision enabled. DEFAULT false"
    echo "  --slam                    Use the SLAM simulation environment. DEFAULT false"
    echo "  --payload <name>          Attach a sim payload (ptz|livox|ouster) --sim only."
    echo "  -i, --interface <name>    CycloneDDS network interface. DEFAULT lo"
}

while [[ $# -gt 0 ]]; do
    case $1 in
        --help) print_help; exit 0 ;;
        --rb1) CMD_ARGS+=("--rb1"); shift ;;
        --rba-1|--rba1) CMD_ARGS+=("--rba-1"); shift ;;
        --lims_ex) CMD_ARGS+=("--lims_ex"); shift ;;
        --wheel) CMD_ARGS+=("--wheel"); shift ;;
        --platform) CMD_ARGS+=("--platform" "$2"); shift 2 ;;
        --rbh) CMD_ARGS+=("--rbh"); shift ;;
        --vision) CMD_ARGS+=("--vision"); shift ;;
        --slam) CMD_ARGS+=("--slam"); shift ;;
        --payload) CMD_ARGS+=("--payload" "$2"); shift 2 ;;
        -i|--interface) CMD_ARGS+=("--interface" "$2"); shift 2 ;;
        *) echo "Unknown argument: $1"; print_help; exit 1 ;;
    esac
done

# 이 PC(같은 PID 이름공간)의 $1 만 찾는다 — 개발 PC 에선 도커 시뮬(CI 러너·scn-here)의 Motion·Mujoco 도 같은 이름으로 보여
# "이미 실행 중"으로 막혔다(11-23). NSpid 칸 수가 나와 같으면 같은 이름공간. 못 읽으면 내 것으로 친다(예전 동작). rcl/ProcessManager.h 와 같다
own_pids() {
    local me p n
    me=$(awk '/^NSpid:/{print NF}' /proc/$$/status 2>/dev/null)
    for p in $(pgrep -x "$1"); do
        [ -r "/proc/$p/status" ] || continue
        n=$(awk '/^NSpid:/{print NF}' "/proc/$p/status" 2>/dev/null)
        { [ -z "$me" ] || [ -z "$n" ] || [ "$n" = "$me" ]; } && echo "$p"
    done
}

if [ -n "$(own_pids "$APP_NAME")" ]; then
    echo "$APP_NAME is already running. Please close it before starting a new instance."
    sleep 10
    exit 1
fi

if [ ! -f "$APP_PATH/$APP_NAME" ]; then
    echo "$APP_NAME application not found."
    echo "Compile it first."
    sleep 10
    exit 1
fi

cd $APP_PATH

while true; do
    pid=$(own_pids "$APP_NAME")
    if [ -z "$pid" ]; then
        sudo ./"$APP_NAME" "${CMD_ARGS[@]}"
    fi
    sleep 2
done

#!/bin/bash
# SDK (cyclonedds pinned in pyproject.toml, Python 3.12) <-> robot role (cyclonedds 0.10.2, Python 3.10),
# two containers on one Docker network — the LAN path a customer PC uses, not lo. See interop.py.
# The robot's Network runs Cyclone DDS 0.10.2 (C++); 0.10.2's Python reads the same type information.
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"   # rbq_sdk/python
PIN=$(grep -o '"cyclonedds==[^"]*"' "$PROJECT_DIR/pyproject.toml" | tr -d '"')
[ -n "$PIN" ] || { echo "❌ no exact cyclonedds pin in pyproject.toml"; exit 1; }

NAME="rbq-sdk-interop-$$"
docker network create "$NAME" >/dev/null
trap 'docker rm -f "$NAME-robot" "$NAME-sdk" >/dev/null 2>&1 || true; docker network rm "$NAME" >/dev/null 2>&1 || true' EXIT

run() {  # $1=role $2=image $3=pip spec — the source is mounted, only cyclonedds is installed
    docker run --name "$NAME-$1" --network "$NAME" -v "$PROJECT_DIR:/src:ro" -w /src \
        -e PYTHONPATH=/src -e PYTHONDONTWRITEBYTECODE=1 -e PIP_DISABLE_PIP_VERSION_CHECK=1 \
        -e INTEROP_IFACE=eth0 "$2" \
        sh -c "pip install -q --root-user-action=ignore '$3' && python scripts/docker/interop.py $1"
}

echo "SDK $PIN (python 3.12) <-> robot role cyclonedds==0.10.2 (python 3.10)"
run robot python:3.10-slim cyclonedds==0.10.2 > /tmp/"$NAME"-robot.log 2>&1 &
ROBOT=$!
SDK_RC=0; run sdk python:3.12-slim "$PIN" || SDK_RC=$?
ROBOT_RC=0; wait "$ROBOT" || ROBOT_RC=$?
cat /tmp/"$NAME"-robot.log; rm -f /tmp/"$NAME"-robot.log

if [ "$SDK_RC" -eq 0 ] && [ "$ROBOT_RC" -eq 0 ]; then
    echo "✅ every SDK type goes both ways between $PIN and the robot's 0.10.2"
else
    # 139 on the robot side = 0.10 crashed on type information it cannot parse (INT8/UINT8 codes) — dds_compat.py
    echo "❌ SDK <-> robot interop failed (sdk exit $SDK_RC, robot exit $ROBOT_RC)"
    exit 1
fi

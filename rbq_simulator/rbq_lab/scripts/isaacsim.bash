#!/bin/bash

SCRIPTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPTS_DIR/activate.bash"

echo -ne "\033]0;isaacsim\007"

echo "Running isaacsim"
eval "isaacsim"

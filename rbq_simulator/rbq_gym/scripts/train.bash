#!/bin/bash

print_help() {
    echo "Usage: $0 [options]"
    echo ""
    echo "Options:"
    echo "  --task TASK_NAME    set task name"
    echo "  --num_envs NUM      set number of environments"
    echo "  --headless          set headless mode"
    echo "  --device DEV        set simulation device (e.g. cuda:0, cpu)"
    echo "  --help              show this help"
    echo ""
}

CMD_ARGS=()
DEFAULT_TASK=true

while [[ $# -gt 0 ]]; do
    case $1 in
        --help) print_help; exit 0 ;;
        --task) CMD_ARGS+=(" $1 $2"); DEFAULT_TASK=false; shift 2 ;;
        --num_envs) CMD_ARGS+=(" $1 $2"); shift 2 ;;
        --device) CMD_ARGS+=(" $1 $2"); shift 2 ;;
        --headless) CMD_ARGS+=(" $1"); shift ;;
        --tracking) CMD_ARGS+=(" $1"); shift ;;
        *) echo "❌ Unknown argument: $1"; print_help; exit 1 ;;
    esac
done

if [[ "${DEFAULT_TASK,,}" == "true" ]]; then
    CMD_ARGS+=(" --task rbq10")
fi

SCRIPTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GYM_DIR="$(dirname "$SCRIPTS_DIR")"
PACKAGE_NAME=$(grep "name=" "$GYM_DIR/setup.py" | awk -F"['\"]" '{print $2}')
CMD="$SCRIPTS_DIR/python.bash $GYM_DIR/$PACKAGE_NAME/train.py ${CMD_ARGS[@]}"
bash $CMD 

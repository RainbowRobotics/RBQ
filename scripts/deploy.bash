#!/bin/bash

REMOTE_DEVICE="rbq@192.168.0.10"
REMOTE_HOME_DIR="~/rbq_ws"
REMOTE_BIN_DIR="$REMOTE_HOME_DIR/bin"
REMOTE_SCRIPTS_DIR="$REMOTE_HOME_DIR/scripts"
REMOTE_RESOURCES_DIR="$REMOTE_HOME_DIR/resources"
BINARY_DIR="bin"
SCRIPTS_DIR="scripts"
SELECTED_BINARIES=()
LOCAL_MODE=false

print_help() {
    echo "Usage: ./deploy.bash [OPTIONS]"
    echo "  --help             Display this help message and exit."
    echo "  --pro [FILES...]    Specify binary names for selective deployment."
    echo "  --device [USER@IP] Set the remote device (default: $REMOTE_DEVICE)."
    echo "  --local            Deploy files locally on the same PC instead of remote transfer."
}

exec > >(tee -a deploy.log) 2>&1

while [[ $# -gt 0 ]]; do
    case $1 in
        --help) print_help; exit 0 ;;
        --device) shift; REMOTE_DEVICE="$1"; shift ;;
        --pro) shift; while [[ $# -gt 0 && ! $1 =~ ^- ]]; do SELECTED_BINARIES+=("$1"); shift; done ;;
        --local) LOCAL_MODE=true; shift ;;
        *) echo "Unknown argument: $1"; print_help; exit 1 ;;
    esac
done

BINARIES_TO_DEPLOY=()
SCRIPTS_TO_DEPLOY=()
RESOURCES_TO_DEPLOY=()

if [ ${#SELECTED_BINARIES[@]} -eq 0 ]; then
    if [ ! -d "$BINARY_DIR" ]; then
        echo -e "\e[31mError: bin directory $BINARY_DIR not found.\e[0m"
        exit 1
    fi
    echo "Identifying binaries..."
    IFS=$'\n' BINARIES=($(find "$BINARY_DIR" -maxdepth 1 \( -type f -o -type l \)))
    BINARIES_TO_DEPLOY=("${BINARIES[@]}")

    SCRIPT_FILES=("RBQ.bash" "start_motion.bash" "start_vision.bash" "start_mediamtx.bash" "start_ros_driver.bash")
    if [ ! -d "$SCRIPTS_DIR" ]; then
        echo -e "\e[31mError: scripts directory $SCRIPTS_DIR not found.\e[0m"
        exit 1
    fi
    for SCRIPT in "${SCRIPT_FILES[@]}"; do
        if [ -f "$SCRIPTS_DIR/$SCRIPT" ]; then
            SCRIPTS_TO_DEPLOY+=("$SCRIPTS_DIR/$SCRIPT")
            echo "  - Added $SCRIPT"
        else
            echo -e "\e[33mWarning: $SCRIPT not found in $SCRIPTS_DIR, skipping.\e[0m"
        fi
    done
    
    RESOURCES_MEDIAMTX_DIR="resources/mediamtx"
    if [ ! -d "$RESOURCES_MEDIAMTX_DIR" ]; then
        echo -e "\e[31mError: mediamtx directory $RESOURCES_MEDIAMTX_DIR not found.\e[0m"
        exit 1
    fi
    RESOURCES_TO_DEPLOY+=("$RESOURCES_MEDIAMTX_DIR")

    if [ ${#BINARIES_TO_DEPLOY[@]} -eq 0 ] && [ ${#SCRIPTS_TO_DEPLOY[@]} -eq 0 ] && [ ${#RESOURCES_TO_DEPLOY[@]} -eq 0 ]; then
        echo -e "\e[31mError: No files to deploy.\e[0m"
        exit 1
    fi
else
    if [ ! -d "$BINARY_DIR" ]; then
        echo -e "\e[31mError: bin directory $BINARY_DIR not found.\e[0m"
        exit 1
    fi
    for FILE in "${SELECTED_BINARIES[@]}"; do
        if [ -f "$BINARY_DIR/$FILE" ]; then
            BINARIES_TO_DEPLOY+=("$BINARY_DIR/$FILE")
        else
            echo -e "\e[33mWarning: $FILE not found, skipping.\e[0m"
        fi
    done
    if [ ${#BINARIES_TO_DEPLOY[@]} -eq 0 ]; then
        echo -e "\e[31mError: No files to deploy.\e[0m"
        exit 1
    fi
fi

if [ "$LOCAL_MODE" = true ]; then
    # Local deployment - use local rbq_ws directory
    # Get the real user's home directory (not root's if running as sudo)
    REAL_HOME="$HOME"
    if [ -n "$SUDO_USER" ]; then
        # Running as sudo, get the original user's home directory
        REAL_HOME=$(getent passwd "$SUDO_USER" | cut -d: -f6)
        if [ -z "$REAL_HOME" ]; then
            # Fallback: try to get from environment
            REAL_HOME=$(eval echo ~$SUDO_USER)
        fi
    fi
    
    # If still empty, use current HOME as fallback
    if [ -z "$REAL_HOME" ]; then
        REAL_HOME="$HOME"
    fi
    
    LOCAL_HOME_DIR="$REAL_HOME/rbq_ws"
    LOCAL_BIN_DIR="$LOCAL_HOME_DIR/bin"
    LOCAL_SCRIPTS_DIR="$LOCAL_HOME_DIR/scripts"
    LOCAL_RESOURCES_DIR="$LOCAL_HOME_DIR/resources"
    echo "Local deployment mode: deploying to $LOCAL_HOME_DIR"
    
    # Create local directories if they don't exist
    mkdir -p "$LOCAL_BIN_DIR"
    mkdir -p "$LOCAL_SCRIPTS_DIR"
    mkdir -p "$LOCAL_RESOURCES_DIR"
else
    # Remote deployment - setup SSH
    echo -e "\e[33mEnter SSH password.\e[0m"
    read -s -p "SSH Password: " SSH_PASSWORD
    echo ""
    SSH_AUTH="sshpass -p $SSH_PASSWORD ssh"

    if [ ! -f "$HOME/.ssh/known_hosts" ]; then
        echo -e "\e[33mNo ~/.ssh/known_hosts yet. Now will ssh to the robot. Type the password then type exit to return\e[0m"
        ssh $REMOTE_DEVICE
    fi
    ssh-keyscan -H $(echo "$REMOTE_DEVICE" | cut -d'@' -f2) >> ~/.ssh/known_hosts 2>/dev/null
fi

if [ "$LOCAL_MODE" = true ]; then
    # Local deployment using cp/rsync locally
    if [ ${#BINARIES_TO_DEPLOY[@]} -gt 0 ]; then
        echo "Copying binaries to $LOCAL_BIN_DIR..."
        if ! rsync -av --progress "${BINARIES_TO_DEPLOY[@]}" "$LOCAL_BIN_DIR/"; then
            echo -e "\e[31mError: Binary copy failed.\e[0m"
            exit 1
        fi
    fi
    if [ ${#SCRIPTS_TO_DEPLOY[@]} -gt 0 ]; then
        echo "Copying scripts to $LOCAL_SCRIPTS_DIR..."
        if ! rsync -av --progress "${SCRIPTS_TO_DEPLOY[@]}" "$LOCAL_SCRIPTS_DIR/"; then
            echo -e "\e[31mError: Script copy failed.\e[0m"
            exit 1
        fi
    fi
    if [ ${#RESOURCES_TO_DEPLOY[@]} -gt 0 ]; then
        echo "Copying resources to $LOCAL_RESOURCES_DIR..."
        if ! rsync -av --progress "${RESOURCES_TO_DEPLOY[@]}" "$LOCAL_RESOURCES_DIR/"; then
            echo -e "\e[31mError: Resources copy failed.\e[0m"
            exit 1
        fi
    fi
else
    # Remote deployment using rsync over SSH
    if [ ${#BINARIES_TO_DEPLOY[@]} -gt 0 ]; then
        echo "File transfer starting..."
        if ! rsync -avz --progress --rsync-path="mkdir -p ${REMOTE_BIN_DIR[@]} && rsync" -e "$SSH_AUTH" "${BINARIES_TO_DEPLOY[@]}" "$REMOTE_DEVICE:$REMOTE_BIN_DIR/"; then
            echo -e "\e[31mError: File transfer failed.\e[0m"
            exit 1
        fi
    fi
    if [ ${#SCRIPTS_TO_DEPLOY[@]} -gt 0 ]; then
        if ! rsync -avz --progress -e "$SSH_AUTH" "${SCRIPTS_TO_DEPLOY[@]}" "$REMOTE_DEVICE:$REMOTE_SCRIPTS_DIR"; then
            echo -e "\e[31mError: Script file transfer failed.\e[0m"
            exit 1
        fi
    fi
    if [ ${#RESOURCES_TO_DEPLOY[@]} -gt 0 ]; then
        if ! rsync -avz --progress -e "$SSH_AUTH" "${RESOURCES_TO_DEPLOY[@]}" "$REMOTE_DEVICE:$REMOTE_RESOURCES_DIR/"; then
            echo -e "\e[31mError: Resources/mediamtx file transfer failed.\e[0m"
            exit 1
        fi
    fi
fi

echo -e "\e[32mDeployment complete.\e[0m"

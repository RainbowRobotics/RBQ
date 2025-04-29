#!/bin/bash

# Default remote connection details
REMOTE_DEVICE="rbq@192.168.0.10"
REMOTE_DIR="~/rbq_ws/bin"
BINARY_DIR="bin"
ID_RSA_KEY=$(ls keys/*_shared_key* 2>/dev/null | head -n 1)
LOGFILE="deploy.log"

# Default argument values
COPY_DEP=false
SELECTED_FILES=()

# Function: Print help message
print_help() {
    echo "Usage: ./deploy.bash [OPTIONS]"
    echo "  --help             Display this help message and exit."
    echo "  -pro [FILES...]    Specify binary names for selective deployment."
    echo "  --device [USER@IP] Set the remote device (default: $REMOTE_DEVICE)."
}

# Start logging
exec > >(tee -a "$LOGFILE") 2>&1

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --help) print_help; exit 0 ;;
        --copy-dep) COPY_DEP=true; shift ;;
        --device) shift; REMOTE_DEVICE="$1"; shift ;;
        -pro) shift; while [[ $# -gt 0 && ! $1 =~ ^- ]]; do SELECTED_FILES+=("$1"); shift; done ;;
        *) echo "Unknown argument: $1"; print_help; exit 1 ;;
    esac
done

# Identify binaries to deploy
echo "Identifying binaries..."
IFS=$'\n' BINARIES=($(find "$BINARY_DIR" -maxdepth 1 -type f))
FILES_TO_DEPLOY=()

# Handle selected binaries
if [ ${#SELECTED_FILES[@]} -eq 0 ]; then
    FILES_TO_DEPLOY=("${BINARIES[@]}")
else
    for FILE in "${SELECTED_FILES[@]}"; do
        if [ -f "$BINARY_DIR/$FILE" ]; then
            FILES_TO_DEPLOY+=("$BINARY_DIR/$FILE")
        else
            echo -e "\e[33mWarning: $FILE not found, skipping.\e[0m"
        fi
    done
fi

# Validate deployment list
if [ ${#FILES_TO_DEPLOY[@]} -eq 0 ]; then
    echo -e "\e[31mError: No files to deploy.\e[0m"
    exit 1
fi

# Check SSH key
if [ -z "$ID_RSA_KEY" ]; then
    echo -e "\e[33mNo SSH key found in keys directory. You may enter your SSH password instead.\e[0m"
    read -s -p "SSH Password: " SSH_PASSWORD
    echo ""
    SSH_AUTH="sshpass -p $SSH_PASSWORD ssh"
else
    echo "Using SSH key: $ID_RSA_KEY"
    SSH_AUTH="ssh -i $ID_RSA_KEY"
fi

# Ensure remote host is known to avoid SSH confirmation issues
ssh-keyscan -H $(echo "$REMOTE_DEVICE" | cut -d'@' -f2) >> ~/.ssh/known_hosts 2>/dev/null

# Check SSH connection
echo "Checking SSH connection: $SSH_AUTH $REMOTE_DEVICE"
#if ! $SSH_AUTH -o BatchMode=yes -o ConnectTimeout=5 "$REMOTE_DEVICE" true; then
#  echo -e "\e[31mError: Unable to establish SSH connection to $REMOTE_DEVICE.\e[0m"
#  exit 1
#fi

# Ensure remote directory exists
$SSH_AUTH "$REMOTE_DEVICE" "mkdir -p '$REMOTE_DIR'"

# Use rsync to transfer files efficiently
echo "Starting file upload with rsync..."
if ! rsync -avz --progress -e "$SSH_AUTH" "${FILES_TO_DEPLOY[@]}" "$REMOTE_DEVICE:$REMOTE_DIR"; then
    echo -e "\e[31mError: File transfer failed.\e[0m"
    exit 1
fi

echo -e "\e[32mDeployment complete.\e[0m"

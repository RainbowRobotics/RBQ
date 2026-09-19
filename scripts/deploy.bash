#!/bin/bash

REMOTE_DEVICE="rbq@192.168.0.10"
REMOTE_HOME_DIR="~/rbq_ws"
REMOTE_SCRIPTS_DIR="$REMOTE_HOME_DIR/scripts"
REMOTE_RESOURCES_DIR="$REMOTE_HOME_DIR/resources"
# The build produces one directory per architecture (bin-x86_64, bin-aarch64).
# Deploy sends only the one matching the target's own `uname -m` - queried locally
# for --local, over SSH otherwise - so a robot never receives binaries it can't run.
BINARY_DIRS=()
SCRIPTS_DIR="scripts"
SELECTED_BINARIES=()
LOCAL_MODE=false

print_help() {
    echo "Usage: ./deploy.bash [OPTIONS]"
    echo "  --help             Display this help message and exit."
    echo "  --pro [FILES...]    Specify binary names for selective deployment."
    echo "  --device [USER@IP] Set the remote device (default: $REMOTE_DEVICE)."
    echo "  --local            Deploy files locally on the same PC instead of remote transfer."
    echo ""
    echo "Only sends the bin-<arch>/ matching the target's uname -m (queried over SSH"
    echo "for remote deploys)."
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

# Figure out the target's architecture before selecting which bin-<arch>/ to send -
# a robot never needs the other architecture's binaries. Local mode: the host IS
# the target, so `uname -m` here already answers it. Remote mode: we have to ask
# the target over SSH, so SSH setup has to happen before binary selection now
# (previously it only ran right before the transfer itself).
if [ "$LOCAL_MODE" = true ]; then
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
    REAL_USER="${SUDO_USER:-$(id -un)}"

    LOCAL_HOME_DIR="$REAL_HOME/rbq_ws"
    LOCAL_SCRIPTS_DIR="$LOCAL_HOME_DIR/scripts"
    LOCAL_RESOURCES_DIR="$LOCAL_HOME_DIR/resources"
    echo "Local deployment mode: deploying to $LOCAL_HOME_DIR"

    # Create local directories if they don't exist. Binaries are NOT pre-created
    # here (no more LOCAL_BIN_DIR) - an empty bin/ would make start_*.bash take its
    # bin/ fallback and print a misleading "not found, compile it first" instead of
    # the correct "no binaries for this arch" when bin-$(uname -m) is missing.
    mkdir -p "$LOCAL_SCRIPTS_DIR"
    mkdir -p "$LOCAL_RESOURCES_DIR"

    TARGET_ARCH="$(uname -m)"
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

    TARGET_ARCH="$($SSH_AUTH $REMOTE_DEVICE uname -m 2>/dev/null | tr -d '\r\n')"
    if [ -z "$TARGET_ARCH" ]; then
        echo -e "\e[31mError: could not determine $REMOTE_DEVICE's architecture (ssh uname -m failed).\e[0m"
        exit 1
    fi
fi
echo "Target architecture: $TARGET_ARCH"

BINARIES_TO_DEPLOY=()
SCRIPTS_TO_DEPLOY=()
RESOURCES_TO_DEPLOY=()

if [ ${#SELECTED_BINARIES[@]} -eq 0 ]; then
    if [ -d "bin-$TARGET_ARCH" ]; then
        BINARY_DIRS=("bin-$TARGET_ARCH")
    else
        echo -e "\e[31mError: bin-$TARGET_ARCH not found - build for the target's architecture first.\e[0m"
        exit 1
    fi
    echo "Identifying binaries in: ${BINARY_DIRS[*]}"
    IFS=$'\n' BINARIES=($(find "${BINARY_DIRS[@]}" -maxdepth 1 \( -type f -o -type l \)))
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
    if [ -d "bin-$TARGET_ARCH" ]; then
        BINARY_DIRS=("bin-$TARGET_ARCH")
    else
        echo -e "\e[31mError: bin-$TARGET_ARCH not found - build for the target's architecture first.\e[0m"
        exit 1
    fi
    for FILE in "${SELECTED_BINARIES[@]}"; do
        FOUND=false
        for D in "${BINARY_DIRS[@]}"; do
            if [ -f "$D/$FILE" ]; then
                BINARIES_TO_DEPLOY+=("$D/$FILE")
                FOUND=true
            fi
        done
        if [ "$FOUND" = false ]; then
            echo -e "\e[33mWarning: $FILE not found in ${BINARY_DIRS[*]}, skipping.\e[0m"
        fi
    done
    if [ ${#BINARIES_TO_DEPLOY[@]} -eq 0 ]; then
        echo -e "\e[31mError: No files to deploy.\e[0m"
        exit 1
    fi
fi

if [ "$LOCAL_MODE" = true ]; then
    # Local deployment using cp/rsync locally
    if [ ${#BINARIES_TO_DEPLOY[@]} -gt 0 ]; then
        echo "Copying binaries to $LOCAL_HOME_DIR..."
        if ! rsync -avR --progress "${BINARIES_TO_DEPLOY[@]}" "$LOCAL_HOME_DIR/"; then
            echo -e "\e[31mError: Binary copy failed.\e[0m"
            exit 1
        fi

        # NOPASSWD sudoers rules for start_motion.bash/start_vision.bash (which run
        # `sudo ./Motion` / `sudo -n ./Vision`) - without this, a fresh local deploy
        # can't start either until Network has run once to self-register the same
        # rule. This path is also what OTA runs, so it must never leave
        # /etc/sudoers in a broken state: write a dedicated drop-in, validate with
        # `visudo -cf` BEFORE installing, then swap in atomically. A distinct
        # filename (not /etc/sudoers.d/rbq) avoids fighting Network's own rule,
        # which only self-registers for the fixed robot deploy path anyway.
        SUDOERS_RULES=""
        for D in "${BINARY_DIRS[@]}"; do
            for APP in Motion Vision; do
                APP_PATH="$LOCAL_HOME_DIR/$D/$APP"
                [ -f "$APP_PATH" ] || continue
                SUDOERS_RULES+="$REAL_USER ALL=(ALL) NOPASSWD: $APP_PATH"$'\n'
            done
        done
        if [ -n "$SUDOERS_RULES" ]; then
            echo "Configuring sudoers for local start scripts..."
            # sudo ignores files whose name contains a `.` in /etc/sudoers.d/, and `visudo -cf`
            # still reports success, so a dotted username would be silently dropped: substitute it.
            SUDOERS_DROPIN="/etc/sudoers.d/rbq-${REAL_USER//./-}"
            SUDOERS_TMP="$SUDOERS_DROPIN.new"
            {
                echo "# Generated by scripts/deploy.bash --local - regenerated on every deploy. Do not edit."
                printf '%s' "$SUDOERS_RULES"
            } | sudo tee "$SUDOERS_TMP" >/dev/null
            sudo chmod 0440 "$SUDOERS_TMP"
            if sudo visudo -cf "$SUDOERS_TMP"; then
                sudo mv -f "$SUDOERS_TMP" "$SUDOERS_DROPIN"
                echo "Sudoers rules installed at $SUDOERS_DROPIN."
            else
                echo -e "\e[33mWarning: generated sudoers rules failed validation, not installed.\e[0m"
                sudo rm -f "$SUDOERS_TMP"
            fi
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
        if ! rsync -avzR --progress --rsync-path="mkdir -p ${REMOTE_HOME_DIR[@]} && rsync" -e "$SSH_AUTH" "${BINARIES_TO_DEPLOY[@]}" "$REMOTE_DEVICE:$REMOTE_HOME_DIR/"; then
            echo -e "\e[31mError: File transfer failed.\e[0m"
            exit 1
        fi

        # NOPASSWD sudoers rules for the new bin-<arch>/ layout
        echo "Configuring sudoers on $REMOTE_DEVICE for start scripts..."
        if {
            printf '%s\n' "$SSH_PASSWORD"
            cat <<'REMOTE_SUDOERS'
SUDO() { printf '%s\n' "$SUDO_PASS" | sudo -S -p '' "$@"; }
RULES="# Generated by scripts/deploy.bash - regenerated on every deploy. Do not edit."$'\n'
FOUND=false
for APP in Motion Vision; do
    APP_PATH="$HOME/rbq_ws/bin-$(uname -m)/$APP"
    [ -f "$APP_PATH" ] || continue
    RULES+="$(id -un) ALL=(ALL) NOPASSWD: $APP_PATH"$'\n'
    FOUND=true
done
[ "$FOUND" = true ] || exit 0
# sudo ignores dotted sudoers.d names - stages .new safely; keep dots out of the filename
DROPIN="/etc/sudoers.d/rbq-$(id -un | tr . -)"
TMP=$(mktemp) || exit 1
printf '%s' "$RULES" > "$TMP" || { rm -f "$TMP"; exit 1; }
SUDO visudo -cf "$TMP" >/dev/null || { rm -f "$TMP"; exit 1; }
SUDO install -o root -g root -m 0440 "$TMP" "$DROPIN.new" || { rm -f "$TMP"; exit 1; }
rm -f "$TMP"
SUDO mv -f "$DROPIN.new" "$DROPIN"
REMOTE_SUDOERS
        # Do not unquote $SSH_AUTH: IFS is still newline-only here, so it would not word-split.
        } | sshpass -p "$SSH_PASSWORD" ssh "$REMOTE_DEVICE" 'read -rs SUDO_PASS; export SUDO_PASS; bash -s'; then
            echo "Sudoers rules installed on $REMOTE_DEVICE."
        else
            echo -e "\e[33mWarning: could not install sudoers rules on $REMOTE_DEVICE.\e[0m"
            echo -e "\e[33mIf no rule exists yet for bin-<arch>/, Motion/Vision cannot autostart after this deploy.\e[0m"
            echo -e "\e[33mGrant once on the robot with:\e[0m"
            echo -e "\e[33m  for APP in Motion Vision; do echo \"\$USER ALL=(ALL) NOPASSWD: \$HOME/rbq_ws/bin-\$(uname -m)/\$APP\"; done | sudo tee /etc/sudoers.d/rbq-\$USER >/dev/null && sudo chmod 0440 /etc/sudoers.d/rbq-\$USER && sudo visudo -c\e[0m"
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

#!/bin/bash

# === CONFIG ===
IFACE="enp0s31f6"
CFG_PATH="$PWD/configs/HAL/ptp4l-master.cfg"
SUDOERS_FILE="/etc/sudoers.d/ptp-nopass"

# === Step 1: Check for required commands ===
REQUIRED_CMDS=("ptp4l" "phc2sys" "ethtool")
MISSING_CMDS=()

for cmd in "${REQUIRED_CMDS[@]}"; do
    if ! command -v "$cmd" &> /dev/null; then
        MISSING_CMDS+=("$cmd")
    fi
done

if [ ${#MISSING_CMDS[@]} -ne 0 ]; then
    echo "[ERROR] Missing required tools: ${MISSING_CMDS[*]}"
    read -p "Do you want to install them now? (Y/n): " answer
    answer=${answer:-Y}
    if [[ "$answer" =~ ^[Yy]$ ]]; then
        echo "[INFO] Installing: ${MISSING_CMDS[*]}"
        sudo apt update
        sudo apt install -y linuxptp ethtool
    else
        echo "[ABORTED] Required tools not installed. Exiting."
        exit 1
    fi
fi

# === Step 2: Setup passwordless sudo for ptp4l and phc2sys ===
USER_NAME=$(whoami)
PTP4L_PATH=$(command -v ptp4l)
PHC2SYS_PATH=$(command -v phc2sys)

check_sudo_nopass() {
    sudo -n "$PTP4L_PATH" -v 2>/dev/null &&
    sudo -n "$PHC2SYS_PATH" -v 2>/dev/null
}

if ! check_sudo_nopass; then
    echo "[INFO] Adding passwordless sudo for $PTP4L_PATH and $PHC2SYS_PATH..."
    TMP_FILE=$(mktemp)
    echo "$USER_NAME ALL=(ALL) NOPASSWD: $PTP4L_PATH, $PHC2SYS_PATH" > "$TMP_FILE"
    if sudo visudo -cf "$TMP_FILE"; then
        sudo mv "$TMP_FILE" "$SUDOERS_FILE"
        sudo chmod 0440 "$SUDOERS_FILE"
        echo "[✅] Passwordless sudo rule installed at $SUDOERS_FILE"
    else
        echo "[❌] Invalid sudoers syntax. Aborting."
        rm "$TMP_FILE"
        exit 1
    fi
else
    echo "[INFO] Passwordless sudo already configured."
fi

# === Step 3: Trap cleanup on terminal close ===
cleanup() {
    echo "[INFO] Stopping PTP processes..."
    sudo kill "$PTP4L_PID" "$PHC2SYS_PID" 2>/dev/null
    sleep 1
    exit
}
trap cleanup INT TERM EXIT

# === Step 4: Show NIC timestamp capability ===
ethtool -T "$IFACE"
TS_MODE=$(grep -E '^[[:space:]]*time_stamping[[:space:]]+' "$CFG_PATH" | awk '{print $2}')

if [ -z "$TS_MODE" ]; then
    echo "[WARN] time_stamping is not set in $CFG_PATH"
    echo "[WARN] ptp4l will use its default timestamp mode"
else
    echo "[INFO] PTP timestamp mode from config: $TS_MODE"
fi

# === Step 5: Start ptp4l ===
echo "[INFO] Starting ptp4l on $IFACE using $CFG_PATH"
sudo "$PTP4L_PATH" -i "$IFACE" -f "$CFG_PATH" -m -l 6 &
PTP4L_PID=$!
sleep 2

if ! ps -p "$PTP4L_PID" > /dev/null; then
    echo "[ERROR] ptp4l failed to start."
    exit 1
fi

# === Step 6: Start phc2sys ===
echo "[INFO] Starting phc2sys to sync PHC → system clock"
sudo "$PHC2SYS_PATH" -s "$IFACE" -c CLOCK_REALTIME -m -O 0 &
PHC2SYS_PID=$!
sleep 2

if ! ps -p "$PHC2SYS_PID" > /dev/null; then
    echo "[ERROR] phc2sys failed to start."
    sudo kill "$PTP4L_PID"
    exit 1
fi

# === Step 7: Wait (keep terminal alive) ===
wait "$PTP4L_PID" "$PHC2SYS_PID"


#!/bin/bash

# Configuration
USER="champi"
SSH_PASS="circuits"
DEST_PATH="/home/champi/champi_ws/src/champi_robot_ros"
LOCAL_PATH="$HOME/champi_ws/src/champi_robot_ros"
IP_ETH="10.0.0.1"
IP_WIFI="172.0.0.1"

# Handle optional relative path argument
REL_PATH="${1:-}"
if [ -n "$REL_PATH" ]; then
    # Strip leading and trailing slashes for clean path concatenation
    REL_PATH="${REL_PATH#/}"
    REL_PATH="${REL_PATH%/}"
    
    # Append trailing slash only if the local target is a directory
    if [ -d "$LOCAL_PATH/$REL_PATH" ]; then
        SYNC_LOCAL="$LOCAL_PATH/$REL_PATH/"
    else
        SYNC_LOCAL="$LOCAL_PATH/$REL_PATH"
    fi
    SYNC_DEST="$DEST_PATH/$REL_PATH"
else
    SYNC_LOCAL="$LOCAL_PATH/"
    SYNC_DEST="$DEST_PATH"
fi

# Exclude patterns (edit this list!)
EXCLUDES=(
    ".git"
    ".idea"
    ".vscode"
    "__pycache__"
    "stm_main_board"
    "stm_pamis_board"
    "champi_isaac_simu"
    "esp32_PAMIs"
)

# Convert exclude list to rsync --exclude options
EXCLUDE_ARGS=()
for pattern in "${EXCLUDES[@]}"; do
    EXCLUDE_ARGS+=("--exclude=$pattern")
done

# Function to check if a host is reachable
is_reachable() {
    ping -c 1 -W 1 $1 > /dev/null 2>&1
    return $?
}

# Determine which IP is reachable
if is_reachable $IP_ETH; then
    ROBOT_IP=$IP_ETH
elif is_reachable $IP_WIFI; then
    ROBOT_IP=$IP_WIFI
else
    echo "❌ Error: Robot is not reachable on either $IP_WIFI or $IP_ETH"
    exit 1
fi

echo "🔄 Syncing to robot at $ROBOT_IP..."
echo "📂 Local: $SYNC_LOCAL"
echo "📁 Remote: $USER@$ROBOT_IP:$SYNC_DEST"
if [ -n "$REL_PATH" ]; then
    echo "🎯 Subset: $REL_PATH"
fi
echo "🚫 Excludes: ${EXCLUDES[*]}"

# Run rsync with excludes
if ! command -v sshpass &> /dev/null; then
    echo "⚠️  sshpass not found. Install it with: sudo apt install sshpass"
    exit 1
fi

# Ensure the destination directory exists on the remote host before syncing
if [ -n "$REL_PATH" ]; then
    REMOTE_DIR=$(dirname "$SYNC_DEST")
    sshpass -p "$SSH_PASS" ssh -o StrictHostKeyChecking=no "$USER@$ROBOT_IP" "mkdir -p '$REMOTE_DIR'"
fi

sshpass -p "$SSH_PASS" rsync -avz --delete --copy-links \
    -e "ssh -o StrictHostKeyChecking=no" \
    "${EXCLUDE_ARGS[@]}" "$SYNC_LOCAL" "$USER@$ROBOT_IP:$SYNC_DEST"

if [ $? -eq 0 ]; then
    echo "✅ Sync completed successfully!"
else
    echo "❌ Sync failed."
fi
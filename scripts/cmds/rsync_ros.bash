#!/bin/bash

# Configuration
USER="champi"
SSH_PASS="circuits"
DEST_PATH="/home/champi/champi_ws/src/champi_robot_ros"
LOCAL_PATH="$HOME/champi_ws/src/champi_robot_ros"
IP_ETH="10.0.0.1"
IP_WIFI="172.0.0.1"

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
echo "📂 Local: $LOCAL_PATH"
echo "📁 Remote: $USER@$ROBOT_IP:$DEST_PATH"
echo "🚫 Excludes: ${EXCLUDES[*]}"

# Run rsync with excludes
if ! command -v sshpass &> /dev/null; then
    echo "⚠️  sshpass not found. Install it with: sudo apt install sshpass"
    exit 1
fi
sshpass -p "$SSH_PASS" rsync -avz --delete --copy-links \
    -e "ssh -o StrictHostKeyChecking=no" \
    "${EXCLUDE_ARGS[@]}" "$LOCAL_PATH/" "$USER@$ROBOT_IP:$DEST_PATH"

if [ $? -eq 0 ]; then
    echo "✅ Sync completed successfully!"
else
    echo "❌ Sync failed."
fi
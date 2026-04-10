#!/bin/bash

# Get all running ROS2 nodes dynamically and kill them
echo "Fetching running ROS2 nodes..."
node_list=$(ros2 node list 2>/dev/null)

if [ -z "$node_list" ]; then
    echo "No ROS2 nodes found."
    exit 0
fi

while IFS= read -r node; do
    # Strip leading slash(es) and get the bare node name
    node_name="${node#/}"
    # Skip rviz
    if [[ "$node_name" == *rviz* ]]; then
        echo "Skipping node: $node_name"
        continue
    fi
    # Skip foxglove
    if [[ "$node_name" == *foxglove* ]]; then
        echo "Skipping node: $node_name"
        continue
    fi
    # Skip internal ROS2 entities (e.g. transform_listener_impl_*) — they live
    # inside another node's process and have no standalone PID to kill.
    if [[ "$node_name" == transform_listener_impl* ]]; then
        echo "Skipping internal node: $node_name"
        continue
    fi
    # Send SIGTERM first; if the process is still alive after 2 s, escalate to SIGKILL.
    if pkill -TERM -f "$node_name" 2>/dev/null; then
        sleep 2
        if pkill -0 -f "$node_name" 2>/dev/null; then
            pkill -KILL -f "$node_name" 2>/dev/null
            echo "Killed node (SIGKILL): $node_name"
        else
            echo "Killed node (SIGTERM): $node_name"
        fi
    else
        echo "Could not kill node (already dead?): $node_name"
    fi
done <<< "$node_list"

echo "Done."

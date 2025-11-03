#!/bin/sh
alias type-check='echo "🔍 Running type checks..." && python3 -m mypy champi_brain && echo "✅ Type checking complete!"' # only for packages ok, we'll expand later
alias build='echo "📦 Sourcing environment..." && source ~/champi_ws/src/champi_robot_ros/setup/env/env_common.sh && echo "🔨 Building packages..." && ~/champi_ws/src/champi_robot_ros/scripts/cmds/champi_build.sh && type-check'
alias clean='echo "🧹 Cleaning workspace..." && rm -R ~/champi_ws/log ~/champi_ws/build ~/champi_ws/install && echo "✅ Workspace cleaned!"'
alias kill_nodes='~/champi_ws/src/champi_robot_ros/scripts/cmds/kill_nodes.bash'

export RCUTILS_COLORIZED_OUTPUT=1

# argcomplete for ros2 & colcon
eval "$(register-python-argcomplete ros2)"
eval "$(register-python-argcomplete colcon)"
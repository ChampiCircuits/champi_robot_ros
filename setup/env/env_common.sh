#!/bin/sh
alias type-check='echo "🔍 Running type checks..." && python3 -m mypy champi_brain && echo "✅ Type checking complete!"' # only for packages ok, we'll expand later
alias build='echo "📦 Sourcing environment..." && source ~/champi_ws/src/champi_robot_ros/setup/env/env_common.sh && echo "🔨 Building packages..." && ~/champi_ws/src/champi_robot_ros/scripts/cmds/champi_build.sh && type-check'
alias clean='echo "🧹 Cleaning workspace..." && rm -R ~/champi_ws/log ~/champi_ws/build ~/champi_ws/install && echo "✅ Workspace cleaned!"'
alias kill_nodes='~/champi_ws/src/champi_robot_ros/scripts/cmds/kill_nodes.bash'
# TODO j'ai cassé les params optionnels de colcon build pour l'instant, a remettre plus tard
export RCUTILS_COLORIZED_OUTPUT=1


## FORMATTING OF ROS2 LOGS
### see https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Logging.html
### see https://discourse.openrobotics.org/t/better-console-logging/44012
carriage_return="\r"
del='\x1b[1k' # delete everything until this point
r='\x1b[0m' # reset font to normal
bold='\x1b[1m'
faint='\x1b[2m'
italic='\x1b[3m'
underline='\x1b[4m'
slow_blink='\x1b[5m'
rapid_blink='\x1b[6m'
normal='\x1b[22m' # reset bold/faint
export RCUTILS_CONSOLE_OUTPUT_FORMAT="${del}${carriage_return}[${faint}{date_time_with_ms}${normal}] ${bold}[{severity}]${normal} [{name}]: {message}${r}"


# argcomplete for ros2 & colcon
eval "$(register-python-argcomplete ros2)"
eval "$(register-python-argcomplete colcon)"
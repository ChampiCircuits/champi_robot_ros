import os
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():

    # Declare the launch options
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='False',
        description='Launch simulation (True|False)',
    )
    color_arg = DeclareLaunchArgument(
        'color',
        default_value='yellow',
        description='Color of the robot (yellow|blue)',
    )
    
    screen_manager = Node(
            package='champi_brain',
            executable='screen_manager.py',
            name='champi_brain',
            output='screen',
    )

    rviz_markers = Node(
            package='champi_brain',
            executable='rviz_markers.py',
            name='rviz_markers',
            output='screen',
    )

    strat = Node(
            package='champi_brain',
            executable='strategy_engine_node.py',
            name='strategy',
            output='screen',
            parameters=[{
                'color': LaunchConfiguration('color'),
                'sim':LaunchConfiguration('sim')}]
    )

    # sim_act_launch = Node(
    #         package='champi_brain',
    #         executable='simu_act_node.py',
    #         name='simu_act_node',
    #         output='screen',
    #         condition=IfCondition(LaunchConfiguration('sim'))
    # )

    return LaunchDescription([
        color_arg,
        sim_arg,
        # sim_act_launch,
        # screen_manager,
        # rviz_markers,
        strat
    ])
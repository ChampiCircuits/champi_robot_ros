"""
Shared utility for configuring motion parameters from ROS parameters.
"""
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from champi_brain.strategy_dsl import MotionParams


def configure_motion_defaults(node: Node) -> None:
    """
    Configure default motion parameters from ROS parameters.
    
    This function should be called during node initialization, before any
    strategy loading that creates Action instances with MotionParams.
    
    Parameters MUST be provided via a config file (--params-file option).
    No default values are used to ensure explicit configuration.
    
    Args:
        node: The ROS2 node that will declare and read the parameters
        
    Raises:
        RuntimeError: If any required parameter is not found in the config file
    """
    # Parameter names and their expected types
    param_configs = {
        'default_motion_speed': ParameterType.PARAMETER_DOUBLE,
        'default_motion_end_speed': ParameterType.PARAMETER_DOUBLE,
        'default_motion_accel_linear': ParameterType.PARAMETER_DOUBLE,
        'default_motion_accel_angular': ParameterType.PARAMETER_DOUBLE,
        'default_motion_use_collision_avoidance': ParameterType.PARAMETER_BOOL
    }
    
    values = {}
    for name, param_type in param_configs.items():
        # Try to get the parameter from config file
        if not node.has_parameter(name):
            # Declare with dynamic typing to allow loading from config file
            descriptor = ParameterDescriptor(
                type=param_type,
                description=f"Motion parameter: {name}",
                read_only=False,
                dynamic_typing=True
            )
            try:
                node.declare_parameter(name, descriptor=descriptor)
            except Exception:
                pass
        
        # Check if parameter exists and get its value
        if node.has_parameter(name):
            param = node.get_parameter(name)
            if param.type_ == Parameter.Type.NOT_SET:
                raise RuntimeError(
                    f"Parameter '{name}' is not set. "
                    f"Please provide motion parameters via a config file using --params-file option."
                )
            values[name] = param.value
        else:
            raise RuntimeError(
                f"Required parameter '{name}' not found. "
                f"Please provide motion parameters via a config file using --params-file option."
            )
    
    MotionParams.set_defaults(
        speed=values['default_motion_speed'],
        end_speed=values['default_motion_end_speed'],
        accel_linear=values['default_motion_accel_linear'],
        accel_angular=values['default_motion_accel_angular'],
        use_collision_avoidance=values['default_motion_use_collision_avoidance']
    )
    
    node.get_logger().info(
        f"Motion defaults: speed={values['default_motion_speed']}, "
        f"end_speed={values['default_motion_end_speed']}, "
        f"accel_linear={values['default_motion_accel_linear']}, "
        f"accel_angular={values['default_motion_accel_angular']}, "
        f"use_collision_avoidance={values['default_motion_use_collision_avoidance']}"
    )

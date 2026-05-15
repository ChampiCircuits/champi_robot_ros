#!/usr/bin/env python3
import nav_msgs
import rclpy, time
from rclpy.node import Node
from std_msgs.msg import Int64, Int64MultiArray
from nav_msgs.msg import Odometry
from champi_interfaces.msg import STMState, CtrlGoal
from champi_interfaces.srv import SetAutoPlacementEnabled
from std_msgs.msg import Int8, Empty

from enum import Enum, IntEnum
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import base64


class ActuatorCommand(IntEnum):
    RESET_ACTUATORS                = 0
    STOP_ALL_MOTORS                = 1
    ENABLE_ALL_MOTORS              = 2
    GET_READY                      = 3

    THERMOMETER_LOWER_SERVO        = 4
    THERMOMETER_RAISE_SERVO        = 5

    LOWER_LEFT_ARM                 = 6
    GET_READY_LEFT_ARM             = 7
    LET_GO_ELEMENTS_LEFT_ARM       = 8
    
    LOWER_RIGHT_ARM                = 9
    GET_READY_RIGHT_ARM            = 10
    LET_GO_ELEMENTS_RIGHT_ARM      = 11

    STORE_PENDING_MASK             = 12
    
    PUMPS_ON = 13
    PUMPS_OFF = 14


class PagesNode(Node):
    _instance = None

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super(PagesNode, cls).__new__(cls, *args, **kwargs)
        return cls._instance
    
    def __init__(self):
        if not hasattr(self, '_initialized'):  # Pour éviter la réinitialisation
            super().__init__('GUI_diagnostics_node')
            self._initialized = True


            self.sub_odom_otos = self.create_subscription(
                nav_msgs.msg.Odometry,
                '/odom_otos',
                self.odom_otos_callback,
                10)
            self.last_odom_otos_time = -1

            self.sub_odom = self.create_subscription(
                nav_msgs.msg.Odometry,
                '/odom',
                self.odom_callback,
                10)
            self.latest_odom_position: tuple[float, float] | None = None
            self.latest_odom_velocity: tuple[float, float] | None = None

            self.sub_goal_pose = self.create_subscription(
                CtrlGoal,
                '/ctrl_goal',
                self.goal_pose_callback,
                10)
            self.latest_goal_position: tuple[float, float] | None = None

            self.cv_bridge = CvBridge()
            self.latest_viz_image_b64 = None
            self.sub_viz_image = self.create_subscription(
                Image,
                '/viz/image_detection',
                self.viz_image_callback,
                10)

            self.sub_stm_state = self.create_subscription(
                STMState,
                '/STM_state',
                self.stm_state_callback,
                10)
            self.last_stm_state:STMState = None

            self.strategy_pub = self.create_publisher(
                String,
                '/chosen_strategy',
                10
            )

            self.score_subscriber = self.create_subscription(Int8, '/final_score', self.update_score, 10)
            self.sm_state_subscriber = self.create_subscription(String, '/sm_state', self.update_sm_state, 10)

            # Variables used during match
            self.score = 0
            self.time_left = 100
            self.start_time = None
            self.match_started = False
            self.ready_to_start_match = False
            self.latest_sm_state = 'waiting_for_strategy'
            self.auto_placement_enabled = False
            self.sm_state_descriptions = {
                'waiting_for_strategy':        'En attente de stratégie',
                'waiting_for_start':           'Prêt — en attente du départ',
                'running_strategy':            'Stratégie en cours',
                'match_ended':                 'Match terminé',
                'emergency_stop':              'Arrêt d\'urgence actif',
                'auto_placement_wait_still':   'Localisation : attente immobilité',
                'auto_placement_localizing':   'Localisation : scan ArUco',
                'auto_placement_setting_pose': 'Localisation : recalage pose',
                'auto_placement_moving':       'Localisation : déplacement pose initiale',
                'auto_placement_failed':       'Localisation : échec (placer manuellement)',
                'moving_to_goal':              'Déplacement vers objectif',
                'executing_action':            'Exécution d\'une action',
            }

            self.timer = self.create_timer(0.1, self.update)

            self.reset_state_machine_pub =  self.create_publisher(
                Empty,
                '/reset_state_machine',
                10
            )
            self.match_ready_pub = self.create_publisher(
                Empty,
                '/match_ready',
                10
            )
            self.stop_match_pub = self.create_publisher(
                Empty,
                '/stop_match',
                10
            )
            self.actuators_ctrl_pub = self.create_publisher(Int8, '/ctrl/actuators', 10)
            self.set_auto_placement_enabled_client = self.create_client(
                SetAutoPlacementEnabled,
                '/set_auto_placement_enabled'
            )

            self.c = 0
            self.get_logger().info("Node created !")

        self.c += 1
        self.get_logger().info(f"{self.c} inits of the singleton node")

    def send_actuator_action(self, action: ActuatorCommand):
        msg = Int8()
        msg.data = int(action)
        self.actuators_ctrl_pub.publish(msg)

    def update(self):
        if not self.match_started:
            if self.last_stm_state is not None and self.last_stm_state.tirette_released and self.ready_to_start_match:
                self.get_logger().info("\n\n!! MATCH STARTED !!\n\n")

                self.score = 0
                self.time_left = 100
                self.match_started = True
                self.start_time = time.time()


    def reset_all(self):
        self.get_logger().warn('Node has been reset !')
        self.score = 0
        self.time_left = 100
        self.start_time = None
        self.match_started = False
        self.ready_to_start_match = False

        self.get_logger().warn('Resetting the state machine !')
        self.reset_state_machine_pub.publish(Empty())

    def stop_match(self):
        self.get_logger().warn('STOP MATCH requested from web UI')
        self.match_started = False
        self.ready_to_start_match = False
        self.stop_match_pub.publish(Empty())

    def update_score(self, received_score: Int8):
        self.score = received_score.data
        self.get_logger().info(f'received score : {received_score}')

    def odom_otos_callback(self, msg):
        self.last_odom_otos_time = time.time()

    def odom_callback(self, msg: Odometry):
        self.latest_odom_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        )
        self.latest_odom_velocity = (
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
        )

    def goal_pose_callback(self, msg: CtrlGoal):
        self.latest_goal_position = (
            msg.pose.position.x,
            msg.pose.position.y,
        )

    def viz_image_callback(self, msg: Image):
        try:
            cv_img = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            _, buffer = cv2.imencode('.jpg', cv_img)
            self.latest_viz_image_b64 = 'data:image/jpeg;base64,' + base64.b64encode(buffer).decode('utf-8')
        except Exception as e:
            self.get_logger().error(f"Error decoding image: {e}")

    def get_sm_state_description(self) -> str:
        return self.sm_state_descriptions.get(self.latest_sm_state, f'({self.latest_sm_state})')

    def stm_state_callback(self, msg):
        self.last_stm_state = msg

    def update_sm_state(self, msg: String):
        self.latest_sm_state = msg.data

    def pub_strategy(self, strategy):
        self.get_logger().info(f'Strategy set to {strategy}')

        msg = String()
        msg.data = strategy
        self.strategy_pub.publish(msg)
        
    def pub_match_ready(self):
        self.match_ready_pub.publish(Empty())

    def set_auto_placement_enabled(self, enabled: bool) -> bool:
        self.auto_placement_enabled = enabled
        if not self.set_auto_placement_enabled_client.service_is_ready():
            self.get_logger().warn('Service /set_auto_placement_enabled not ready')
            return False

        request = SetAutoPlacementEnabled.Request()
        request.enabled = enabled
        future = self.set_auto_placement_enabled_client.call_async(request)
        future.add_done_callback(self._on_set_auto_placement_done)
        return True

    def _on_set_auto_placement_done(self, future):
        try:
            future.result()
        except Exception as exc:
            self.get_logger().error(f'Failed to call /set_auto_placement_enabled: {exc}')




def init_ros_node() -> Node:
    if not rclpy.ok():
        rclpy.init()
    node = PagesNode()
    return node
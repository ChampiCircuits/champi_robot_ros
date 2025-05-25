#include "champi_hw_interface/hw_interface.h"
#include <champi_hw_interface/hw_actuators.h>

#include "tf2/impl/utils.h"


#define THRESHOLD_REJECT_DIST 0.03

// Normalize angle to be within [-pi, pi]
double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}


HardwareInterfaceNode::HardwareInterfaceNode() : Node("modbus_sender_node")
{
    this->declare_parameter<std::string>("device_ser_no", "3952366C3233");
    this->declare_parameter<int>("baud_rate", 115200);
    this->declare_parameter<int>("slave_id", 1);

    this->device_ser_no_ = this->get_parameter("device_ser_no").as_string();
    this->baud_rate_ = this->get_parameter("baud_rate").as_int();
    this->slave_id_ = this->get_parameter("slave_id").as_int();

    stm_config_.is_set = false;

    stm_config_.holo_drive_config.wheel_radius = this->declare_parameter<double>("stm_config.holo_drive_config.wheel_radius");
    stm_config_.holo_drive_config.base_radius = this->declare_parameter<double>("stm_config.holo_drive_config.base_radius");
    stm_config_.holo_drive_config.max_accel_wheel = this->declare_parameter<double>("stm_config.holo_drive_config.max_accel_wheel");
    stm_config_.holo_drive_config.max_accel_linear = this->declare_parameter<double>("stm_config.holo_drive_config.max_acceleration_linear");
    stm_config_.holo_drive_config.max_decel_linear = this->declare_parameter<double>("stm_config.holo_drive_config.max_deceleration_linear");
    stm_config_.holo_drive_config.max_accel_angular = this->declare_parameter<double>("stm_config.holo_drive_config.max_acceleration_angular");
    stm_config_.holo_drive_config.max_decel_angular = this->declare_parameter<double>("stm_config.holo_drive_config.max_deceleration_angular");
    stm_config_.otos_config.linear_scalar = this->declare_parameter<double>("stm_config.otos_config.linear_scalar");
    stm_config_.otos_config.angular_scalar = this->declare_parameter<double>("stm_config.otos_config.angular_scalar");

    stm_config_.cmd_vel_timeout = this->declare_parameter<double>("stm_config.cmd_vel_timeout");

    cov_pose_odom_otos_ = this->declare_parameter<std::vector<double>>("covariances.pose_otos");
    cov_vel_odom_otos_ = this->declare_parameter<std::vector<double>>("covariances.vel_otos");
    assert(cov_pose_odom_otos_.size() == 6);
    assert(cov_vel_odom_otos_.size() == 6);

    mod_reg::setup_registers();

    while (setup_modbus() != 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to setup modbus, retrying...");
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    setup_stm();

    // Store initial Otos pose for otos_pose (viz) to start to 0.
    read(mod_reg::reg_state);
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, mod_reg::state->otos_pose.theta);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&HardwareInterfaceNode::loop, this));

    latest_twist_ = geometry_msgs::msg::Twist();
    subscriber_twist_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(
        &HardwareInterfaceNode::twist_callback, this, std::placeholders::_1));

    pub_odom_otos_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_otos", 10);

    // ACTUATORS
    subscriber_ctrl_actuators_ = this->create_subscription<std_msgs::msg::Int8>("/ctrl/actuators", 10, std::bind(
        &HardwareInterfaceNode::actuators_control_callback, this, std::placeholders::_1));
    pub_ctrl_actuators_ = this->create_publisher<std_msgs::msg::Int8MultiArray>("/actuators_finished", 10);
    pub_stm_state = this->create_publisher<champi_interfaces::msg::STMState>("/STM_state", 10);
}

HardwareInterfaceNode::~HardwareInterfaceNode()
{
    if (mb_) {
        modbus_close(mb_);
        modbus_free(mb_);
    }
}

nav_msgs::msg::Odometry make_odom(const Vector3 &pose, const Vector3 &vel,
                                  const std::vector<double> &cov_pose,
                                  const std::vector<double> &cov_vel,
                                  const builtin_interfaces::msg::Time stamp) {
  auto msg = nav_msgs::msg::Odometry();
  msg.header.stamp = stamp;
  msg.header.frame_id = "odom";
  msg.child_frame_id = "base_link";

  msg.pose.pose.position.x = pose.x;
  msg.pose.pose.position.y = pose.y;
  msg.pose.pose.position.z = 0.0;

  double pose_theta = normalize_angle(pose.theta);
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, pose_theta);
  msg.pose.pose.orientation.x = q.x();
  msg.pose.pose.orientation.y = q.y();
  msg.pose.pose.orientation.z = q.z();
  msg.pose.pose.orientation.w = q.w();

  msg.twist.twist.linear.x = vel.x;
  msg.twist.twist.linear.y = vel.y;
  msg.twist.twist.angular.z = vel.theta;

  // Covariance
  for (int i = 0; i < 6; i++) {
    msg.pose.covariance[i * 6 + i] = cov_pose[i];
    msg.twist.covariance[i * 6 + i] = cov_vel[i];
  }
  return msg;
}


nav_msgs::msg::Odometry HardwareInterfaceNode::make_odom_otos(const Vector3 &pose, const double dt) const {

    static Vector3 prev_pose = {0, 0, 0};
    static bool first_time = true;

    Vector3 vel = {0, 0, 0};
    if (first_time) {
        first_time = false;
    }
    else {
        const double delta_x = pose.x - prev_pose.x;
        const double delta_y = pose.y - prev_pose.y;
        const double delta_theta = normalize_angle(pose.theta - prev_pose.theta);

        // This is for when we call set_pose, the otos pose changes
        const double cos_theta = std::cos(prev_pose.theta);
        const double sin_theta = std::sin(prev_pose.theta);

        vel.x = (delta_x * cos_theta + delta_y * sin_theta) / dt;
        vel.y = (-delta_x * sin_theta + delta_y * cos_theta) / dt;
        vel.theta = delta_theta / dt;

        if (vel.theta > 50.0 || vel.theta < -50.0) {
            RCLCPP_ERROR(this->get_logger(), "Velocity theta is too high: %f", vel.theta); // just in case if it reappears
        }
    }
    prev_pose = pose;

    return make_odom(
        pose,
        vel,
        cov_pose_odom_otos_,
        cov_vel_odom_otos_,
        this->now());
}


void HardwareInterfaceNode::loop() {

    static rclcpp::Time last_time;
    static bool first_time = true;
    if (first_time) {
        last_time = this->now();
        first_time = false;
        return;
    }
    auto dt = (this->now() - last_time).seconds();
    last_time = this->now();

    // Read
    read(mod_reg::reg_state);

    if (mod_reg::state->safe_check_counter != latest_safe_check_counter_value) {
        latest_safe_check_counter_value = mod_reg::state->safe_check_counter;
        auto odom_otos = make_odom_otos(mod_reg::state->otos_pose, dt);
        pub_odom_otos_->publish(odom_otos);

        geometry_msgs::msg::TransformStamped transform_stamped;

        // Write
        mod_reg::cmd->is_read = false;
        mod_reg::cmd->cmd_vel.x = latest_twist_.linear.x;
        mod_reg::cmd->cmd_vel.y = latest_twist_.linear.y;
        mod_reg::cmd->cmd_vel.theta = latest_twist_.angular.z;

        write(mod_reg::reg_cmd);
    }
    else {
        RCLCPP_DEBUG(this->get_logger(), "Safe check counter is still the same: %d",
                     static_cast<int>(mod_reg::state->safe_check_counter));
    }

    check_for_actuators_state();
    read_stm_state();
}

void HardwareInterfaceNode::write( mod_reg::register_metadata &reg_meta) const {
    int result;
    int nb_attempts = 0;
    do {
        result = modbus_write_registers(this->mb_, reg_meta.address, reg_meta.size, reg_meta.ptr);
        nb_attempts++;
    }
    while (result == -1 && nb_attempts++ < MODBUS_MAX_RETRIES);

    if (nb_attempts > 3) {
        RCLCPP_ERROR(this->get_logger(), "Write data after %d attempts", nb_attempts);
    }

    if (result == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write data: error num: %d, message: %s", errno, modbus_strerror(errno));
        exit(1);
    }
}

void HardwareInterfaceNode::read( mod_reg::register_metadata &reg_meta) const {

    int result;
    int nb_attempts = 0;
    do {
        result = modbus_read_registers(this->mb_, reg_meta.address, reg_meta.size, reg_meta.ptr);
        nb_attempts++;
    }
    while (result == -1 && nb_attempts++ < MODBUS_MAX_RETRIES);

    if (nb_attempts > 3) {
        RCLCPP_ERROR(this->get_logger(), "Read data after %d attempts", nb_attempts);
    }

    if (result == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to read data: error num: %d, message: %s", errno, modbus_strerror(errno));
        exit(1);
    }
}


void HardwareInterfaceNode::check_for_actuators_state() const // TODO mettre a 5Hz
{
    std::string states_string;
    read(mod_reg::reg_actuators);
    for (int i=0; i<ACTUATORS_COUNT; i++)
    {
        const ActuatorState state = static_cast<ActuatorState>(mod_reg::actuators->requests[i]);
        states_string += to_string(state) + " ";

        if (state == ActuatorState::DONE)
        {
            // set state to NOTHING
            mod_reg::actuators->requests[i] = static_cast<uint8_t>(ActuatorState::NOTHING);
            this->write(mod_reg::reg_actuators);
            RCLCPP_INFO(this->get_logger(), "Actuator %s is done", to_string(static_cast<ActuatorCommand>(i)).c_str());

            // pub to topic
            auto msg = std_msgs::msg::Int8MultiArray();
            // 1. Initialiser les données
            msg.data.resize(ACTUATORS_COUNT);

            // 2. Définir la structure (layout) du tableau
            msg.layout.dim.resize(1);             // 1 dimension (1D array)
            msg.layout.dim[0].label = "actuator_states";  // facultatif, pour info
            msg.layout.dim[0].size = ACTUATORS_COUNT;     // taille totale du tableau
            msg.layout.dim[0].stride = 7;                 // stride = nb d’éléments pour "sauter" une ligne (comme en matrice)

            // 3. offset à 0 (début du tableau)
            msg.layout.data_offset = 0;
            msg.data[i] = static_cast<int8_t>(ActuatorState::DONE);
            pub_ctrl_actuators_->publish(msg);
        }
    }

    // RCLCPP_INFO(this->get_logger(), "Actuators requests %s", states_string.c_str());
}

void HardwareInterfaceNode::read_stm_state()
{
    read(mod_reg::reg_state);

    auto msg = champi_interfaces::msg::STMState();
    msg.e_stop_pressed = mod_reg::state->e_stop_pressed;
    msg.tirette_released = mod_reg::state->tirette_released;

    pub_stm_state->publish(msg);
}

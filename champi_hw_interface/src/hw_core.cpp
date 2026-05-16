#include "champi_hw_interface/hw_interface.h"
#include <champi_hw_interface/hw_actuators.h>

#include "tf2/impl/utils.h"

#include <cctype>
#include <unistd.h>


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
    subscriber_strategy = this->create_subscription<std_msgs::msg::String>("/chosen_strategy", 10, std::bind(
        &HardwareInterfaceNode::strategy_callback, this, std::placeholders::_1));

    pub_odom_otos_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_otos", 10);

    // ACTUATORS
    subscriber_ctrl_actuators_ = this->create_subscription<std_msgs::msg::Int8>("/ctrl/actuators", 10, std::bind(
        &HardwareInterfaceNode::actuators_control_callback, this, std::placeholders::_1));
    pub_ctrl_actuators_ = this->create_publisher<std_msgs::msg::Int8MultiArray>("/actuators_finished", 10);
    pub_stm_state = this->create_publisher<champi_interfaces::msg::STMState>("/STM_state", 10);

    // NUTBOXES DETECTION — subscribe to compute suction cup activation mask
    subscriber_nutboxes_detection_ = this->create_subscription<champi_interfaces::msg::NutBoxesDetection>(
        "/nutboxes_detection", 10,
        std::bind(&HardwareInterfaceNode::nutboxes_detection_callback, this, std::placeholders::_1));
}

void HardwareInterfaceNode::strategy_callback(const std_msgs::msg::String::SharedPtr msg)
{
    const auto hash_pos = msg->data.find('#');
    if (hash_pos == std::string::npos || hash_pos + 1 >= msg->data.size()) {
        RCLCPP_WARN(this->get_logger(), "Received malformed /chosen_strategy message: '%s'", msg->data.c_str());
        return;
    }

    std::string color = msg->data.substr(hash_pos + 1);
    for (char &c : color) {
        c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    }

    TeamColor parsed_color = TeamColor::UNKNOWN;
    if (color == "YELLOW") {
        parsed_color = TeamColor::YELLOW;
    } else if (color == "BLUE") {
        parsed_color = TeamColor::BLUE;
    }

    if (parsed_color == TeamColor::UNKNOWN) {
        RCLCPP_WARN(this->get_logger(), "Received unknown team color '%s' on /chosen_strategy", color.c_str());
        return;
    }

    if (mod_reg::requests->team_color != parsed_color) {
        mod_reg::requests->team_color = parsed_color;
        RCLCPP_INFO(this->get_logger(), "Set Team color to STM from /chosen_strategy: %s", color.c_str());
    }
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


void HardwareInterfaceNode::reconnect_modbus() {
    RCLCPP_ERROR(this->get_logger(), "🔌 Too many consecutive failures (%d), reconnecting modbus...", consecutive_failures_);
    consecutive_failures_ = 0;

    if (mb_) {
        modbus_close(mb_);
        modbus_free(mb_);
        mb_ = nullptr;
    }

    while (setup_modbus() != 0) {
        RCLCPP_ERROR(this->get_logger(), "Reconnect failed, retrying in 1s...");
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    RCLCPP_INFO(this->get_logger(), "🔌 Modbus reconnected! Re-configuring STM...");
    setup_stm();
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

    // Check if we need to reconnect
    RCLCPP_DEBUG(this->get_logger(), "Loop start, consecutive_failures_=%d", static_cast<int>(consecutive_failures_));
    if (consecutive_failures_ >= MAX_CONSECUTIVE_FAILURES) {
        RCLCPP_ERROR(this->get_logger(), "🔌 Consecutive failures reached %d, triggering reconnect...", static_cast<int>(consecutive_failures_));
        std::lock_guard<std::mutex> lock(modbus_mutex_);
        reconnect_modbus();
        return;
    }

    // Read
    std::lock_guard<std::mutex> lock(modbus_mutex_);
    if (!read(mod_reg::reg_state)) {
        return; // skip this iteration, will reconnect if failures pile up
    }

    if (mod_reg::state->safe_check_counter != latest_safe_check_counter_value) {
        latest_safe_check_counter_value = mod_reg::state->safe_check_counter;
        auto odom_otos = make_odom_otos(mod_reg::state->otos_pose, dt);
        pub_odom_otos_->publish(odom_otos);

        geometry_msgs::msg::TransformStamped transform_stamped;

        // Write
        mod_reg::cmd->is_read = false;
        mod_reg::cmd->cmd_vel.x = -latest_twist_.linear.x;
        mod_reg::cmd->cmd_vel.y = latest_twist_.linear.y;
        mod_reg::cmd->cmd_vel.theta = latest_twist_.angular.z;

        // RCLCPP_INFO(this->get_logger(), "New cmd_vel to send: x: %.2f, y: %.2f, theta: %.2f",
        //              mod_reg::cmd->cmd_vel.x,
        //              mod_reg::cmd->cmd_vel.y,
        //              mod_reg::cmd->cmd_vel.theta);

        write(mod_reg::reg_cmd);
    }
    else {
        RCLCPP_DEBUG(this->get_logger(), "Safe check counter is still the same: %d",
                     static_cast<int>(mod_reg::state->safe_check_counter));
    }

    // Throttle actuator state check to ~5Hz (every 10 loop iterations at 50Hz)
    static int actuator_check_counter = 0;
    if (++actuator_check_counter >= 10) {
        actuator_check_counter = 0;
        check_for_actuators_state();
    }

    read_stm_state(); // uses already-read reg_state, no extra modbus read
}

bool HardwareInterfaceNode::write( mod_reg::register_metadata &reg_meta) const {
    int result;
    int nb_attempts = 0;
    do {
        result = modbus_write_registers(this->mb_, reg_meta.address, reg_meta.size, reg_meta.ptr);
        nb_attempts++;
        if (result == -1 && nb_attempts < MODBUS_MAX_RETRIES) {
            RCLCPP_WARN(this->get_logger(), "Write retry %d/%d for reg addr=%d size=%d: errno=%d (%s)",
                        nb_attempts, MODBUS_MAX_RETRIES, reg_meta.address, reg_meta.size, errno, modbus_strerror(errno));
            modbus_flush(this->mb_);
            usleep(10000); // 10ms delay before retry
        }
    }
    while (result == -1 && nb_attempts < MODBUS_MAX_RETRIES);

    if (nb_attempts > 1 && result != -1) {
        RCLCPP_WARN(this->get_logger(), "Write succeeded after %d attempts (reg addr=%d)", nb_attempts, reg_meta.address);
    }

    if (result == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write data: reg addr=%d size=%d, errno=%d (%s)",
                     reg_meta.address, reg_meta.size, errno, modbus_strerror(errno));
        consecutive_failures_++;
        usleep(2000);
        return false;
    }

    consecutive_failures_ = 0;
    usleep(2000); // 2ms inter-transaction gap to let the bus settle
    return true;
}

bool HardwareInterfaceNode::read( mod_reg::register_metadata &reg_meta) const {

    int result;
    int nb_attempts = 0;
    do {
        result = modbus_read_registers(this->mb_, reg_meta.address, reg_meta.size, reg_meta.ptr);
        nb_attempts++;
        if (result == -1 && nb_attempts < MODBUS_MAX_RETRIES) {
            RCLCPP_WARN(this->get_logger(), "Read retry %d/%d for reg addr=%d size=%d: errno=%d (%s)",
                        nb_attempts, MODBUS_MAX_RETRIES, reg_meta.address, reg_meta.size, errno, modbus_strerror(errno));
            modbus_flush(this->mb_);
            usleep(10000); // 10ms delay before retry
        }
    }
    while (result == -1 && nb_attempts < MODBUS_MAX_RETRIES);

    if (nb_attempts > 1 && result != -1) {
        RCLCPP_WARN(this->get_logger(), "Read succeeded after %d attempts (reg addr=%d)", nb_attempts, reg_meta.address);
    }

    if (result == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to read data: reg addr=%d size=%d, errno=%d (%s)",
                     reg_meta.address, reg_meta.size, errno, modbus_strerror(errno));
        consecutive_failures_++;
        RCLCPP_ERROR(this->get_logger(), ">>> consecutive_failures_ = %d / %d", static_cast<int>(consecutive_failures_), MAX_CONSECUTIVE_FAILURES);
        usleep(2000);
        return false;
    }

    consecutive_failures_ = 0;
    usleep(2000); // 2ms inter-transaction gap to let the bus settle
    return true;
}


void HardwareInterfaceNode::check_for_actuators_state() const // TODO mettre a 5Hz
{
    std::string states_string;
    read(mod_reg::reg_actuators);
    for (size_t i=0; i < static_cast<size_t>(ActuatorCommand::ACTUATORS_COUNT); i++)
    {
        const ActuatorState state = static_cast<ActuatorState>(mod_reg::actuators->requests[i]);
        states_string += to_c_str(state);
        states_string += " ";

        if (state == ActuatorState::DONE)
        {
            // set state to NOTHING
            mod_reg::actuators->requests[i] = static_cast<uint8_t>(ActuatorState::NOTHING);
            this->write(mod_reg::reg_actuators);
            RCLCPP_INFO(this->get_logger(), "Actuator %s is done", to_c_str(static_cast<ActuatorCommand>(i)));

            // pub to topic
            auto msg = std_msgs::msg::Int8MultiArray();
            // 1. Initialiser les données
            msg.data.resize(static_cast<size_t>(ActuatorCommand::ACTUATORS_COUNT));

            // 2. Définir la structure (layout) du tableau
            msg.layout.dim.resize(1);             // 1 dimension (1D array)
            msg.layout.dim[0].label = "actuator_states";  // facultatif, pour info
            msg.layout.dim[0].size = static_cast<size_t>(ActuatorCommand::ACTUATORS_COUNT);     // taille totale du tableau
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
    // reg_state is already read in loop(), no need to read again

    auto msg = champi_interfaces::msg::STMState();
    msg.e_stop_pressed = mod_reg::state->e_stop_pressed;
    msg.tirette_released = mod_reg::state->tirette_released;

    pub_stm_state->publish(msg);
}

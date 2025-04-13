#include "champi_controllers/hw_interface.h"
#include <tf2/impl/utils.h>


HardwareInterfaceNode::HardwareInterfaceNode() : Node("modbus_sender_node")
{
    this->declare_parameter<std::string>("device_ser_no", "3952366C3233");
    this->declare_parameter<int>("baud_rate", 115200);
    this->declare_parameter<int>("slave_id", 1);

    this->device_ser_no_ = this->get_parameter("device_ser_no").as_string();
    this->baud_rate_ = this->get_parameter("baud_rate").as_int();
    this->slave_id_ = this->get_parameter("slave_id").as_int();

    stm_config_.is_set = false;

    stm_config_.holo_drive_config.wheel_radius = (float) this->declare_parameter<float>("base_config.wheel_radius");
    stm_config_.holo_drive_config.base_radius = (float) this->declare_parameter<float>("base_config.base_radius");
    stm_config_.holo_drive_config.max_speed_linear = this->declare_parameter<double>("base_config.max_speed_linear");
    stm_config_.holo_drive_config.max_accel_wheel = (float) this->declare_parameter<float>("base_config.max_accel_wheel");
    stm_config_.holo_drive_config.max_accel_linear = this->declare_parameter<double>("base_config.max_acceleration_linear");
    stm_config_.holo_drive_config.max_decel_linear = this->declare_parameter<double>("base_config.max_deceleration_linear");
    stm_config_.holo_drive_config.max_accel_angular = this->declare_parameter<double>("base_config.max_acceleration_angular");
    stm_config_.holo_drive_config.max_decel_angular = this->declare_parameter<double>("base_config.max_deceleration_angular");

    stm_config_.cmd_vel_timeout = (float) this->declare_parameter<float>("base_config.cmd_vel_timeout");

    cov_pose_odom_wheels_ = this->declare_parameter<std::vector<double>>("covariances.pose_wheels");
    cov_vel_odom_wheels_ = this->declare_parameter<std::vector<double>>("covariances.vel_wheels");
    cov_pose_odom_otos_ = this->declare_parameter<std::vector<double>>("covariances.pose_otos");
    cov_vel_odom_otos_ = this->declare_parameter<std::vector<double>>("covariances.vel_otos");
    assert(cov_pose_odom_wheels_.size() == 6);
    assert(cov_vel_odom_wheels_.size() == 6);
    assert(cov_pose_odom_otos_.size() == 6);
    assert(cov_vel_odom_otos_.size() == 6);

    mod_reg::setup_registers();

    while (setup_modbus() != 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to setup modbus, retrying...");
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    setup_stm();

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&HardwareInterfaceNode::loop, this));
    
    subscriber_twist_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel_joy", 10, std::bind(
        &HardwareInterfaceNode::twist_callback, this, std::placeholders::_1));

    latest_twist_ = std::make_shared<geometry_msgs::msg::Twist>();

    pub_odom_wheels_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_wheels", 10);
    pub_odom_otos_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_otos", 10);
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

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, pose.theta);
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();

    msg.twist.twist.linear.x = vel.x;
    msg.twist.twist.linear.y = vel.y;
    msg.twist.twist.angular.z = vel.theta;

    // Covariance
    for(int i = 0; i < 6; i++) {
        msg.pose.covariance[i * 6 + i] = cov_pose[i];
        msg.twist.covariance[i * 6 + i] = cov_vel[i];
    }
    return msg;
}


nav_msgs::msg::Odometry HardwareInterfaceNode::make_odom_wheels(const Vector3 &vel, const double dt) {
  const Vector3 pose = pose_integrator_odom_wheels_.compute(vel, dt);
    return make_odom(
        pose,
        vel,
        cov_pose_odom_wheels_,
        cov_vel_odom_wheels_,
        this->now());
}

nav_msgs::msg::Odometry HardwareInterfaceNode::make_odom_otos(const Vector3 &pose, const double dt) const {

    static Vector3 prev_pose = {0, 0, 0};
    static bool first_time = true;

    Vector3 vel = {0, 0, 0};
    if (first_time) {
        prev_pose = pose;
        first_time = false;
    }
    else {
        vel.x = (pose.x - prev_pose.x) / dt;
        vel.y = (pose.y - prev_pose.y) / dt;
        vel.theta = (pose.theta - prev_pose.theta) / dt;
    }

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
    if (errno == 5) { // Happens when we upload new firmware to the STM32
        RCLCPP_WARN(this->get_logger(), "Connection lost. Setting up modbus again...");
        reconnect();
    }

    pub_odom_wheels_->publish(make_odom_wheels(mod_reg::state->measured_vel, dt));
    pub_odom_otos_->publish(make_odom_otos(mod_reg::state->otos_pose, dt));

    // RCLCPP_INFO(this->get_logger(), "Measured velocity: x=%f, y=%f, z=%f",  mod_reg::measured_vel->x,  mod_reg::measured_vel->y,  mod_reg::measured_vel->theta);
    // RCLCPP_INFO(this->get_logger(), "Pose: x=%f, y=%f, z=%f",  mod_reg::otos_pose->x,  mod_reg::otos_pose->y,  mod_reg::otos_pose->theta);

    // Write
    mod_reg::cmd->is_read = false;
    mod_reg::cmd->cmd_vel.x = latest_twist_->linear.x;
    mod_reg::cmd->cmd_vel.y = latest_twist_->linear.y;
    mod_reg::cmd->cmd_vel.theta = latest_twist_->angular.z;

    write(mod_reg::reg_cmd);
    if (errno == 5) { // Happens when we upload new firmware to the STM32
        RCLCPP_WARN(this->get_logger(), "Connection lost. Setting up modbus again...");
        reconnect();
    }
}

int HardwareInterfaceNode::write( mod_reg::register_metadata &reg_meta) const {
    int rc = modbus_write_registers(this->mb_, reg_meta.address, reg_meta.size, reg_meta.ptr);
    if (rc == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write data: error num: %d, message: %s", errno, modbus_strerror(errno));
    }

    return rc;
}

int HardwareInterfaceNode::read( mod_reg::register_metadata &reg_meta) const {
    int rc = modbus_read_registers(this->mb_, reg_meta.address, reg_meta.size, reg_meta.ptr);
    if (rc == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to read data: %s", modbus_strerror(errno));
    }
    return rc;
}
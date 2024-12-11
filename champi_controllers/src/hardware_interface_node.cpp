#include <rclcpp/rclcpp.hpp>
#include <modbus/modbus.h>
#include <vector>
#include <chrono>

#include <champi_controllers/ModbusRegister.h>



class HardwareInterfaceNode : public rclcpp::Node
{
public:
    HardwareInterfaceNode() : Node("modbus_sender_node")
    {
        this->declare_parameter<std::string>("device", "/dev/ttyACM0");
        this->declare_parameter<int>("baud_rate", 115200);
        this->declare_parameter<int>("slave_id", 1);

        this->device_ = this->get_parameter("device").as_string();
        this->baud_rate_ = this->get_parameter("baud_rate").as_int();
        this->slave_id_ = this->get_parameter("slave_id").as_int();



        base_config_.is_set = false;
        base_config_.max_accel = (float) this->declare_parameter<float>("base_config.max_accel_wheels");
        base_config_.wheel_radius = (float) this->declare_parameter<float>("base_config.wheel_radius");
        base_config_.base_radius = (float) this->declare_parameter<float>("base_config.base_radius");
        base_config_.cmd_vel_timeout = (float) this->declare_parameter<float>("base_config.cmd_vel_timeout");

        mod_reg::setup_registers_master();

        setup_modbus();

        configure_stm();



        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&HardwareInterfaceNode::loop, this));

    }

    ~HardwareInterfaceNode()
    {
        if (mb_) {
            modbus_close(mb_);
            modbus_free(mb_);
        }
    }

private:
    void setup_modbus()
    {
        mb_ = modbus_new_rtu(device_.c_str(), baud_rate_, 'N', 8, 1);
        if (!mb_) {
            RCLCPP_FATAL(this->get_logger(), "Failed to create Modbus RTU context");
            rclcpp::shutdown();
            return;
        }

        if (modbus_connect(mb_) == -1) {
            RCLCPP_FATAL(this->get_logger(), "Connection to Modbus RTU device failed: %s", modbus_strerror(errno));
            modbus_free(mb_);
            rclcpp::shutdown();
            return;
        }

        if (modbus_set_slave(mb_, slave_id_) == -1) {
            RCLCPP_FATAL(this->get_logger(), "Failed to set Modbus slave ID: %s", modbus_strerror(errno));
            modbus_close(mb_);
            modbus_free(mb_);
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Modbus RTU connection established with device: %s", device_.c_str());

        // uint32_t old_response_to_sec;
        // uint32_t old_response_to_usec;

        /* Save original timeout */
        // modbus_get_response_timeout(mb_, &old_response_to_sec, &old_response_to_usec);

        // RCUTILS_LOG_INFO("Old response timeout: %d sec %d usec", old_response_to_sec, old_response_to_usec);
    }


    void write( mod_reg::register_metadata &reg_meta)
    {
        int rc = modbus_write_registers(this->mb_, reg_meta.address, reg_meta.size, reg_meta.ptr);
        if (rc == -1) {
            // TODO handle error
            RCUTILS_LOG_ERROR("Failed to write data: %s", modbus_strerror(errno));
        }
    }

    void read( mod_reg::register_metadata &reg_meta)
    {
        int rc = modbus_read_registers(this->mb_, reg_meta.address, reg_meta.size, reg_meta.ptr);
        if (rc == -1) {
            // TODO handle error
            RCUTILS_LOG_ERROR("Failed to read data: %s", modbus_strerror(errno));
        }
    }


    void write_config()
    {
        *mod_reg::base_config = base_config_;
        mod_reg::base_config->is_set = true;
        write( mod_reg::reg_base_config);
    }

    void read_config()
    {
        read( mod_reg::reg_base_config);
        base_config_ = *mod_reg::base_config;
    }

    void configure_stm()
    {
        while (!base_config_.is_set) {
            write_config();
            read_config();
            RCLCPP_WARN_SKIPFIRST(this->get_logger(), "Writing config to STM did not work, retrying...");
        }
        RCLCPP_INFO(this->get_logger(), "STM configured successfully");
    }


    void loop()
    {
         mod_reg::cmd_vel->x = 0.1;
         mod_reg::cmd_vel->y = 0.2;
         mod_reg::cmd_vel->theta = 0.3;


        write( mod_reg::reg_cmd_vel);

        read( mod_reg::reg_measured_vel);


        RCLCPP_INFO(this->get_logger(), "Measured velocity: x=%f, y=%f, z=%f",  mod_reg::measured_vel->x,  mod_reg::measured_vel->y,  mod_reg::measured_vel->theta);
    }

    std::string device_;
    int baud_rate_;
    int slave_id_;
    modbus_t *mb_{nullptr};
    rclcpp::TimerBase::SharedPtr timer_;

    BaseConfig base_config_{};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HardwareInterfaceNode>());
    rclcpp::shutdown();
    return 0;
}

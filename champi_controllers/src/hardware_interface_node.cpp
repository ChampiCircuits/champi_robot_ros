#include <rclcpp/rclcpp.hpp>
#include <modbus/modbus.h>
#include <vector>
#include <chrono>

#include <champi_controllers/ModbusRegister.h>

#include <libudev.h>
#include <iostream>


#include <libudev.h>
#include <iostream>
#include <string>

std::string findDeviceBySerial(const std::string& targetSerial) {
    struct udev *udev = udev_new();
    if (!udev) {
        std::cerr << "Can't create udev\n";
        return "";
    }

    struct udev_enumerate *enumerate = udev_enumerate_new(udev);
    udev_enumerate_add_match_subsystem(enumerate, "tty");
    udev_enumerate_scan_devices(enumerate);

    struct udev_list_entry *devices = udev_enumerate_get_list_entry(enumerate);
    struct udev_list_entry *dev_list_entry;

    udev_list_entry_foreach(dev_list_entry, devices) {
        const char *path = udev_list_entry_get_name(dev_list_entry);
        struct udev_device *dev = udev_device_new_from_syspath(udev, path);

        // Get the parent USB device (if it exists)
        struct udev_device *parent = udev_device_get_parent_with_subsystem_devtype(
            dev,
            "usb",
            "usb_device"
        );

        if (parent) {
            const char *serial = udev_device_get_sysattr_value(parent, "serial");
            const char *devNode = udev_device_get_devnode(dev);

            if (serial && devNode) {
                if (targetSerial == serial) {
                    std::string result(devNode);

                    // Cleanup
                    udev_device_unref(dev);
                    udev_enumerate_unref(enumerate);
                    udev_unref(udev);

                    return result;
                }
            }
        }

        udev_device_unref(dev);
    }

    udev_enumerate_unref(enumerate);
    udev_unref(udev);

    return "";
}



class HardwareInterfaceNode : public rclcpp::Node
{
public:
    HardwareInterfaceNode() : Node("modbus_sender_node")
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

        mod_reg::setup_registers();

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

        std::string device_path = findDeviceBySerial(device_ser_no_);

        mb_ = modbus_new_rtu(device_path.c_str(), baud_rate_, 'N', 8, 1);
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

        RCLCPP_INFO(this->get_logger(), "Modbus RTU connection established with device: %s", device_path.c_str());

        // uint32_t old_response_to_sec;
        // uint32_t old_response_to_usec;

        /* Save original timeout */
        // modbus_get_response_timeout(mb_, &old_response_to_sec, &old_response_to_usec);

        // RCUTILS_LOG_INFO("Old response timeout: %d sec %d usec", old_response_to_sec, old_response_to_usec);
    }


    int write( mod_reg::register_metadata &reg_meta)
    {
        int rc = modbus_write_registers(this->mb_, reg_meta.address, reg_meta.size, reg_meta.ptr);
        if (rc == -1) {

            RCUTILS_LOG_ERROR("Failed to write data: error num: %d, message: %s", errno, modbus_strerror(errno));
        }
        return rc;
    }

    int read( mod_reg::register_metadata &reg_meta)
    {
        int rc = modbus_read_registers(this->mb_, reg_meta.address, reg_meta.size, reg_meta.ptr);
        if (rc == -1) {
            RCUTILS_LOG_ERROR("Failed to read data: %s", modbus_strerror(errno));
        }
        return rc;

    }


    void write_config()
    {
        *mod_reg::stm_config = stm_config_;
        mod_reg::stm_config->is_set = true;
        int result = write(mod_reg::reg_stm_config);
        if (result == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to write STM config: %s", modbus_strerror(errno));
        }


    }

    void read_config()
    {
        int result = read( mod_reg::reg_stm_config);
        if (result == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to read STM config: %s", modbus_strerror(errno));
            return;
        }
        stm_config_ = *mod_reg::stm_config;
        // TODO check if the read config is the same as the written one
    }

    void configure_stm()
    {
        do {
            write_config();
            read_config();
            RCLCPP_WARN_SKIPFIRST(this->get_logger(), "Writing config to STM...");
            rclcpp::sleep_for(std::chrono::seconds(1));
        }
        while (!stm_config_.is_set);
        std::cout << stm_config_.is_set << std::endl;
        RCLCPP_INFO(this->get_logger(), "STM configured successfully");
    }


    void loop()
    {
         mod_reg::cmd_vel->x = 0.1;
         mod_reg::cmd_vel->y = 0.2;
         mod_reg::cmd_vel->theta = 0.3;


        write( mod_reg::reg_cmd_vel);

        read( mod_reg::reg_measured_vel);

        read( mod_reg::reg_otos_pose);


        RCLCPP_INFO(this->get_logger(), "Measured velocity: x=%f, y=%f, z=%f",  mod_reg::measured_vel->x,  mod_reg::measured_vel->y,  mod_reg::measured_vel->theta);
        RCLCPP_INFO(this->get_logger(), "Pose: x=%f, y=%f, z=%f",  mod_reg::otos_pose->x,  mod_reg::otos_pose->y,  mod_reg::otos_pose->theta);

    }

    std::string device_ser_no_;
    int baud_rate_;
    int slave_id_;
    modbus_t *mb_{nullptr};
    rclcpp::TimerBase::SharedPtr timer_;

    StmConfig stm_config_{};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HardwareInterfaceNode>());
    rclcpp::shutdown();
    return 0;
}

#include "champi_controllers/hw_interface.h"

#include <libudev.h>


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

void HardwareInterfaceNode::setup_modbus()
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

void HardwareInterfaceNode::write_config()
{
    *mod_reg::stm_config = stm_config_;
    mod_reg::stm_config->is_set = true;
    int result = write(mod_reg::reg_stm_config);
    if (result == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write STM config: %s", modbus_strerror(errno));
    }


}

void HardwareInterfaceNode::read_config()
{
    int result = read( mod_reg::reg_stm_config);
    if (result == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to read STM config: %s", modbus_strerror(errno));
        return;
    }
    stm_config_ = *mod_reg::stm_config;
    // TODO check if the read config is the same as the written one
}

void HardwareInterfaceNode::setup_stm()
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

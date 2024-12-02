#include <rclcpp/rclcpp.hpp>
#include <modbus/modbus.h>
#include <vector>
#include <chrono>

using namespace std;

class ModbusSenderNode : public rclcpp::Node
{
public:
    ModbusSenderNode() : Node("modbus_sender_node")
    {
        this->declare_parameter<std::string>("device", "/dev/ttyACM0");
        this->declare_parameter<int>("baud_rate", 115200);
        this->declare_parameter<int>("slave_id", 1);

        this->device_ = this->get_parameter("device").as_string();
        this->baud_rate_ = this->get_parameter("baud_rate").as_int();
        this->slave_id_ = this->get_parameter("slave_id").as_int();

        setup_modbus();

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&ModbusSenderNode::send_data, this));
    }

    ~ModbusSenderNode()
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

        uint32_t old_response_to_sec;
        uint32_t old_response_to_usec;

        /* Save original timeout */
        // modbus_get_response_timeout(mb_, &old_response_to_sec, &old_response_to_usec);

        // RCUTILS_LOG_INFO("Old response timeout: %d sec %d usec", old_response_to_sec, old_response_to_usec);


    }

    void send_data()
    {
        const uint16_t data[3] = {1, 2, 3};
        int rc = modbus_write_registers(mb_, 0, 3, data); // Writing to 3 consecutive registers starting at address 0
        if (rc == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to write data: %s", modbus_strerror(errno));
        } else {
            RCLCPP_INFO(this->get_logger(), "Data sent successfully: [%d, %d, %d]", data[0], data[1], data[2]);
        }
    }

    std::string device_;
    int baud_rate_;
    int slave_id_;
    modbus_t *mb_{nullptr};
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ModbusSenderNode>());
    rclcpp::shutdown();
    return 0;
}

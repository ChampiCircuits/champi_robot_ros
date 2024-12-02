#include <rclcpp/rclcpp.hpp>
#include <modbus/modbus.h>
#include <vector>
#include <chrono>

using namespace std;


struct Vector3
{
    float x;
    float y;
    float z;
};

uint16_t registers[100];

const int address_cmd_vel = 0;
const int data_len_cmd_vel = sizeof(Vector3) / sizeof(uint16_t);
Vector3* cmd_vel = reinterpret_cast<Vector3*>(registers);

const int address_measured_vel = address_cmd_vel + sizeof(Vector3) / sizeof(uint16_t);
const int data_len_measured_vel = sizeof(Vector3) / sizeof(uint16_t);
Vector3* measured_vel = reinterpret_cast<Vector3*>(registers + address_measured_vel);



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
            std::chrono::milliseconds(20),
            std::bind(&ModbusSenderNode::loop, this));
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

        // uint32_t old_response_to_sec;
        // uint32_t old_response_to_usec;

        /* Save original timeout */
        // modbus_get_response_timeout(mb_, &old_response_to_sec, &old_response_to_usec);

        // RCUTILS_LOG_INFO("Old response timeout: %d sec %d usec", old_response_to_sec, old_response_to_usec);


    }

    void loop()
    {

        cmd_vel->x = 1.1;
        cmd_vel->y = 2.2;
        cmd_vel->z = 3.3;


        int rc = modbus_write_registers(mb_, address_cmd_vel, data_len_cmd_vel, registers+address_cmd_vel);
        if (rc == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to write data: %s", modbus_strerror(errno));
        } else {
            RCLCPP_INFO(this->get_logger(), "Data sent successfully");
        }

        rc = modbus_read_registers(mb_, address_measured_vel, data_len_measured_vel, registers+address_measured_vel);
        if (rc == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to read data: %s", modbus_strerror(errno));
        } else {
            RCLCPP_INFO(this->get_logger(), "Data read successfully");
            RCLCPP_INFO(this->get_logger(), "Measured velocity: x=%f, y=%f, z=%f", measured_vel->x, measured_vel->y, measured_vel->z);
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

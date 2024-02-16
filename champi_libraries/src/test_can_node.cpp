#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <fstream>
#include <iostream>
#include <string>

#include "champi_can/msgs_can.pb.h"

using namespace std;

class CanInterface {
   public:
    CanInterface(string can_interface_name) :
        can_interface_name_(can_interface_name) {
    }

    ~CanInterface() {
        this->close_interface();
    }

    int open() {
        if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
            perror("Socket");
            return 1;
        } else {
            cout << "CAN Socket created" << endl;
        }

        strcpy(ifr.ifr_name, can_interface_name_.c_str());
        ioctl(s, SIOCGIFINDEX, &ifr);

        memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            perror("Bind");
            return 1;
        }
        return 0;
    }

    int send(canid_t id, string msg) {
        assert(msg.size() <= 8);
        struct can_frame frame;
        frame.can_id = id;
        for (long unsigned int i = 0; i < msg.size(); i++) { // todo optimize with memcpy
            frame.data[i] = msg[i];
        }
        frame.can_dlc = msg.size();

        if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
            perror("Write");
            return 1;
        }
        return 0;
    }


   private:
    int s;
    string can_interface_name_;
    struct sockaddr_can addr;
    struct ifreq ifr;

    int close_interface() {
        if (close(s) < 0) {
            perror("Close");
            return 1;
        } else {
            cout << "CAN Socket closed" << endl;
        }
        return 0;
    }
};

int main() {
    // Verify that the version of the library that we linked against is
    // compatible with the version of the headers we compiled against.
    // GOOGLE_PROTOBUF_VERIFY_VERSION;

    cout << "Hello, World!" << endl;

    // msgs_can::BaseVel base_vel_cmd;

    // base_vel_cmd.set_x(1.0);
    // base_vel_cmd.set_y(2.0);
    // base_vel_cmd.set_theta(3.0);

    // // serialize the message
    // string buffer;
    // base_vel_cmd.SerializeToString(&buffer);

    // cout << "buffer: " << buffer << endl;

    // // deserialize the message
    // msgs_can::BaseVel base_vel_cmd2;
    // base_vel_cmd2.ParseFromString(buffer);

    // cout << "x: " << base_vel_cmd2.x() << endl;
    // cout << "y: " << base_vel_cmd2.y() << endl;
    // cout << "theta: " << base_vel_cmd2.theta() << endl;

    // Optional:  Delete all global objects allocated by libprotobuf.
    // google::protobuf::ShutdownProtobufLibrary();

    // Test send CAN

    CanInterface can_interface("vcan0");

    can_interface.open();

    canid_t id = 0x123;
    string msg = "Hello";

    can_interface.send(id, msg);

    return 0;
}

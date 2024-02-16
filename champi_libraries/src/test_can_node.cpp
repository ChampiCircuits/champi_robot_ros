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
        
        unsigned char* ptr = (unsigned char*) &msg[0];
        return send(id, ptr, msg.size());
    }

    int send(canid_t id, unsigned char* msg, int msg_size) {
        assert(msg_size <= 8);
        struct can_frame frame;
        frame.can_id = id;
 
        memcpy(frame.data, msg, msg_size);
        frame.can_dlc = msg_size;

        if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
            perror("Write");
            return 1;
        }
        
    }

    int receive(canid_t& id, string& msg) {
        struct can_frame frame;
        int nbytes = read(s, &frame, sizeof(struct can_frame));
        if (nbytes < 0) {
            perror("Read");
            return 1;
        }
        id = frame.can_id;
        msg = string((char *)frame.data, frame.can_dlc);
        return 0;
    }

    int close_interface() {
        if (close(s) < 0) {
            perror("Close");
            return 1;
        } else {
            cout << "CAN Socket closed" << endl;
        }
        return 0;
    }


   private:
    int s;
    string can_interface_name_;
    struct sockaddr_can addr;
    struct ifreq ifr;


};

class ChampiCanInterface {
    /*
    Implements our champiprotocol !!
    */

    public:
    ChampiCanInterface(string can_interface_name) :
        can_interface_(can_interface_name) {
    }

    ~ChampiCanInterface() {

    }

    int open() {
        return can_interface_.open();
    }

    int send_simple(canid_t id, string msg) {
        
        unsigned long int msg_size = msg.size();

        unsigned char* ptr = (unsigned char*) &msg[0];

        while(msg_size > 8) {
            can_interface_.send(id, ptr, 8);
            msg_size-=8;
            ptr += 8;
        }
        if(msg_size > 0) {
            can_interface_.send(id, ptr, msg_size);
        }
    }







    private:
    CanInterface can_interface_;
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

    // CanInterface can_interface("vcan0");

    // can_interface.open();

    // canid_t id = 0x123;
    // string msg = "Hello";

    // can_interface.send(id, msg);

    // canid_t id_rec;
    // string msg_rec;
    
    // can_interface.receive(id_rec, msg_rec);

    ChampiCanInterface champi_can_interface("vcan0");

    champi_can_interface.open();

    canid_t id = 0x123;

    string msg = "HelloPOISSON";

    champi_can_interface.send_simple(id, msg);

    return 0;
}

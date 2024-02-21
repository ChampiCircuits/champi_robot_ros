#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <string>

#include <fstream>
#include <iostream>
#include <cstring>
#include <cassert>

#include <boost/asio.hpp>

class CanInterface {
   public:
    CanInterface(std::string can_interface_name) :
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
            std::cout << "CAN Socket created" << std::endl;
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

    int send(canid_t id, std::string msg) {
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
        return 0;
    }

    int receive(struct can_frame &frame) {
        int nbytes = read(s, &frame, sizeof(struct can_frame));
        if (nbytes < 0) {
            perror("Read");
            return 1;
        }
        return 0;
    }

    int close_interface() {
        if (close(s) < 0) {
            perror("Close");
            return 1;
        } else {
            std::cout << "CAN Socket closed" << std::endl;
        }
        return 0;
    }


   private:
    int s;
    std::string can_interface_name_;
    struct sockaddr_can addr;
    struct ifreq ifr;

};
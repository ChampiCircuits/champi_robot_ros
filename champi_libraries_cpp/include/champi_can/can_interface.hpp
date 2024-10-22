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
    /**
     * @brief Class to handle the CAN interface: open, send, receive and close the interface.
     * 
     */
   public:

    /**
     * @brief Construct a new Can Interface object
     * 
     * @param can_interface_name Name of the CAN interface to use. Example: "can0"
     */
    CanInterface(std::string can_interface_name) :
        can_interface_name_(can_interface_name) {
    }

    /**
     * @brief Default constructor. Do not use directly.
     *
     */
    CanInterface() = default;

    /**
     * @brief Destructor. Closes the CAN socket.
     * 
     */
    ~CanInterface() {
        this->close_interface();
    }

    /**
     * @brief Configure and open the CAN socket. Must be called before sending or receiving frames.
     * 
     * @return int 0 if the interface was opened successfully, 1 otherwise.
     */
    int open() {
        if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
            perror("Socket");
            return 1;
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

    /**
     * @brief Send a CAN frame
     * 
     * @param id ID of the frame to send.
     * @param msg Message to send.
     * @return int 0 if the frame was sent successfully, 1 otherwise.
     */
    int send(canid_t id, std::string msg) {        
        return send(id, (unsigned char*) &msg[0], msg.size());
    }

    /**
     * @brief Send a CAN frame.
     * 
     * @param id ID of the frame to send.
     * @param msg Message to send. 8 bytes maximum.
     * @param msg_size Size of the message to send. Must be less or equal to 8.
     * @return int 0 if the frame was sent successfully, 1 otherwise.
     */
    int send(canid_t id, unsigned char* msg, int msg_size) {
        assert(msg_size <= 8); // I left it so we can't miss a problem in the code (TODO remove ?)
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

    /**
     * @brief Receive a CAN frame. This function is blocking until a frame is received.
     * When a frame is received, it is stored in the frame parameter.
     * 
     * @param frame (out) The frame to store the received frame.
     * @return int 0 if the frame was read successfully, 1 otherwise.
     */
    int receive(struct can_frame &frame) {
        int nbytes = read(s, &frame, sizeof(struct can_frame));
        if (nbytes < 0) {
            perror("Read");
            return 1;
        }
        return 0;
    }

    /**
     * @brief Close the CAN socket. Must be called before the program ends for a clean exit.
     * 
     * @return int 0 if the socket was closed successfully, 1 otherwise.
     */
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
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <fstream>
#include <iostream>
#include <string>
#include <mutex>
#include <thread>
#include <map>
#include <vector>

#include <assert.h>



#include "champi_can/msgs_can.pb.h"

using namespace std;

/*
Test frames:
cansend vcan0 123#C1.00.73.64.71.73.44.51
cansend vcan0 123#C0.00.31.32.33.73.64.71
cansend vcan0 123#C2.00.34.35.36

cansend vcan0 123#40.20.66.73.64

cansend vcan0 123#
*/

class MessageRecomposer {

public:
    MessageRecomposer(int id_of_interest) :
        id_of_interest_(id_of_interest),
        msg_number_(-1),
        n_frames_(-1),
        full_msg_received_(false) {
    }

    // default constructor, do not use directly
    MessageRecomposer() :
        id_of_interest_(-1),
        msg_number_(-1),
        n_frames_(-1),
        full_msg_received_(false) {
    }

    void add_new_frame(can_frame frame) {

        int msg_number;
        int msg_size;
        int frame_index;

        decode_descriptor(frame, msg_number, msg_size, frame_index);

        // TODO check if the number follows the expected order (0 -> 1 -> 2 -> 3 -> 0 -> ...)
        // But it's not that easy bc what if we miss an entire message ? we would loose the 3 messages that follow it
        if(msg_number_ != msg_number) {
            // new message
            cout << "New message" << endl;
            msg_number_ = msg_number;
            n_frames_ = msg_size;
            for(int i=0; i<n_frames_; i++) {
                frames_received_[i] = false;
            }
        }

        frames_received_[frame_index] = true;
        msg_parts[frame_index] = string((char*)frame.data+2, frame.can_dlc-2);

        for(int i=0; i<n_frames_; i++) {
            cout << "Frame " << i << " received: " << frames_received_[i] << endl;
        }

        if(all_frames_received()) {
            cout << "All frames received" << endl;
            string full_msg;
            for(int i=0; i<n_frames_; i++) {
                full_msg += msg_parts[i];
            }
            full_msg_ = full_msg;
            full_msg_received_ = true;
            for(int i=0; i<n_frames_; i++) {
                frames_received_[i] = false;
            }
        }
        
    }

    bool all_frames_received() {
        for(int i=0; i<n_frames_; i++) {
            if(!frames_received_[i]) {
                return false;
            }
        }
        return true;
    }

    void decode_descriptor(can_frame frame, int &msg_number, int &msg_size, int &frame_index) {

        uint16_t msg_descriptor = frame.data[1] << 8 | frame.data[0];

        // todo mask the unused bits for safety (eg if error of transmission)

        msg_number = (msg_descriptor >> 12);
        msg_size = (msg_descriptor >> 6) & 0x3F;
        frame_index = msg_descriptor & 0x3F;
    }

    bool check_if_new_full_msg() {
        return full_msg_received_;
    }

    string get_full_msg() {
        full_msg_received_ = false;
        return full_msg_;
    }

private:
    int id_of_interest_;
    int msg_number_;
    int n_frames_; // nb of frames needed to get the full message
    bool frames_received_[64];
    string msg_parts[64];

    string full_msg_;
    bool full_msg_received_;

};

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



class ChampiCan {
    /*
    Implements our champiprotocol !!
    */

    public:
    ChampiCan(string can_interface_name, vector<int> ids_of_interest) :
        can_interface_(can_interface_name) {

        for(int id : ids_of_interest) {
            message_recomposers_[id] = MessageRecomposer(id);
        }
    }

    ~ChampiCan() {
        receive_thread_.join();
        cout << "Receive thread joined" << endl;
        // TODO understand why the receive thread is not joining
    }

    int start() {
        int ret = can_interface_.open();
        if(ret != 0) {
            return ret;
        }

        receive_thread_ = thread(&ChampiCan::receive_thread_ftc, this);

        return 0;
    }

    int send(canid_t id, string msg) {
        
        static unsigned char msg_number = 0;

        unsigned long int msg_size = msg.size();
        assert(msg_size <= 512);
        uint16_t nb_frames = (uint16_t) msg_size / 6 + (msg_size % 6 > 0 ? 1 : 0);

        for(uint16_t i=0; i<nb_frames; i++) {
            
            uint16_t msg_descriptor = msg_number << 12 | (nb_frames << 6) | i;

            string msg_to_send = msg.substr(i*6, 6);
            msg_to_send = string((char*)&msg_descriptor, 2) + msg_to_send;

            can_interface_.send(id, msg_to_send);
        }

        msg_number = (msg_number + 1) % 4; 

        return 0;
    }

    void receive_thread_ftc() {
        canid_t id;
        string msg;
        while(true) {
            struct can_frame frame;
            can_interface_.receive(frame);
            id = frame.can_id;

            mtx_.lock();

            if(message_recomposers_.find(id) != message_recomposers_.end()) {

                // check for empty frame
                if(frame.can_dlc == 0) {
                    cout << "WARNING! Empty frame received for a frame with data expected. Discarding frame" << endl;
                    continue;
                }

                message_recomposers_[id].add_new_frame(frame);
            }
            mtx_.unlock();
        }
    }

    /**
     * @brief Warning: Never, NEVER ask for an ID that is not in the list of ids_of_interest.
     * 
     * @param id 
     * @return true 
     * @return false 
     */
    bool check_if_new_full_msg(canid_t id) {
        lock_guard<mutex> lock(mtx_);
        return message_recomposers_[id].check_if_new_full_msg();
    }

    /**
     * @brief Warning: Always check if the message is ready (check_if_new_full_msg) before calling this function.
     * Will return the newest message received.
     * 
     * @param id 
     * @return string 
     */
    string get_full_msg(canid_t id) {
        lock_guard<mutex> lock(mtx_);
        return message_recomposers_[id].get_full_msg();
    }



    private:
    CanInterface can_interface_;
    map<int, MessageRecomposer> message_recomposers_;

    thread receive_thread_;
    mutex mtx_;
};

int main() {
    // Verify that the version of the library that we linked against is
    // compatible with the version of the headers we compiled against.
    // GOOGLE_PROTOBUF_VERIFY_VERSION;

    cout << "Hello, Wsqdqsorld!" << endl;

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

    ChampiCan champi_can_interface("vcan0", {0x123, 0x456});

    champi_can_interface.start();

    canid_t id = 0x123;

    string msg = "123456789";

    while(true) {
        if(champi_can_interface.check_if_new_full_msg(id)) {
            cout << "New message received: " << champi_can_interface.get_full_msg(id) << endl;
        }
        champi_can_interface.send(id, msg);

        this_thread::sleep_for(chrono::milliseconds(1000));
    }


    return 0;
}

#include <string>
#include <mutex>
#include <thread>
#include <map>
#include <vector>
#include <cassert>
#include <iostream>

#include "champi_can/message_recomposer.hpp"
#include "champi_can/can_interface.hpp"


class ChampiCan {
    /*
    Implements our champiprotocol !!
    */

    public:
    ChampiCan(std::string can_interface_name, std::vector<int> ids_of_interest) :
        can_interface_(can_interface_name) {

        for(int id : ids_of_interest) {
            message_recomposers_[id] = MessageRecomposer(id);
        }
    }

    ~ChampiCan() {
        std::terminate(); // quick fix. TODO use future instead of thread
        std::cout << "Receive thread joined" << std::endl;
        // TODO understand why the receive thread is not joining
    }

    int start() {
        int ret = can_interface_.open();
        if(ret != 0) {
            return ret;
        }

        receive_thread_ = std::thread(&ChampiCan::receive_thread_ftc, this);

        return 0;
    }

    int send(canid_t id, std::string msg) {
        
        static unsigned char msg_number = 0; // TODO how does this works ?? It should be uint16_t...

        unsigned long int msg_size = msg.size();
        assert(msg_size <= 512);
        uint16_t nb_frames = (uint16_t) msg_size / 6 + (msg_size % 6 > 0 ? 1 : 0);

        for(uint16_t i=0; i<nb_frames; i++) {
            
            uint16_t msg_descriptor = msg_number << 12 | (nb_frames << 6) | i;

            std::string msg_to_send = msg.substr(i*6, 6);
            msg_to_send = std::string((char*)&msg_descriptor, 2) + msg_to_send;

            can_interface_.send(id, msg_to_send);
        }

        msg_number = (msg_number + 1) % 4; 

        return 0;
    }

    void receive_thread_ftc() {
        canid_t id;
        std::string msg;
        while(true) {
            struct can_frame frame;
            can_interface_.receive(frame);
            id = frame.can_id;

            mtx_.lock();

            if(message_recomposers_.find(id) != message_recomposers_.end()) {

                // check for empty frame
                if(frame.can_dlc == 0) {
                    std::cout << "WARNING! Empty frame received for a frame with data expected. Discarding frame" << std::endl;
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
        std::lock_guard<std::mutex> lock(mtx_);
        return message_recomposers_[id].check_if_new_full_msg();
    }

    /**
     * @brief Warning: Always check if the message is ready (check_if_new_full_msg) before calling this function.
     * Will return the newest message received.
     * 
     * @param id 
     * @return string 
     */
    std::string get_full_msg(canid_t id) {
        std::lock_guard<std::mutex> lock(mtx_);
        return message_recomposers_[id].get_full_msg();
    }



    private:
    CanInterface can_interface_;
    std::map<int, MessageRecomposer> message_recomposers_;

    std::thread receive_thread_;
    std::mutex mtx_;
};
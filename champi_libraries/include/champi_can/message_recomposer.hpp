#include <linux/can.h>
#include <iostream>

class MessageRecomposer {

public:
    explicit MessageRecomposer(int id_of_interest, bool verbose=false) :
        id_of_interest_(id_of_interest),
        msg_number_(-1),
        n_frames_(-1),
        full_msg_received_(false),
        verbose_(verbose){
    }

    // default constructor, do not use directly
    MessageRecomposer() :
        id_of_interest_(-1),
        msg_number_(-1),
        n_frames_(-1),
        full_msg_received_(false),
        verbose_(false){
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
            if (verbose_) {
                std::cout << "CHAMPI_CAN: New message advertised for ID: " << std::hex << id_of_interest_ << ". Msg number: " << msg_number << ". Msg size: " << msg_size << std::endl;
            }
            msg_number_ = msg_number;
            n_frames_ = msg_size;
            for(int i=0; i<n_frames_; i++) {
                frames_received_[i] = false;
            }
        }

        frames_received_[frame_index] = true;
        msg_parts[frame_index] = std::string((char*)frame.data+2, frame.can_dlc-2);

        if(verbose_) {
            for(int i=0; i<n_frames_; i++) {
                std::cout << "CHAMPI_CAN: Frame " << i << " received: " << frames_received_[i] << std::endl;
            }
        }

        if(all_frames_received()) {
            std::string full_msg;
            for(int i=0; i<n_frames_; i++) {
                full_msg += msg_parts[i];
            }
            full_msg_ = full_msg;
            full_msg_received_ = true;
            for(int i=0; i<n_frames_; i++) {
                frames_received_[i] = false;
            }
            if (verbose_) {
                std::cout << "CHAMPI_CAN: Full message received for ID: " << std::hex << id_of_interest_ << ". Message: " << full_msg_ << std::endl;
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

    std::string get_full_msg() {
        full_msg_received_ = false;
        return full_msg_;
    }

private:
    int id_of_interest_;
    int msg_number_;
    int n_frames_; // nb of frames needed to get the full message
    bool frames_received_[64]{};
    std::string msg_parts[64]; // TODO RAM usage could be optimized at the cost of CPU usage

    std::string full_msg_;
    bool full_msg_received_;

    bool verbose_;

};
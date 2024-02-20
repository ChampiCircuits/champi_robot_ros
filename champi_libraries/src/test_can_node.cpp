

#include <fstream>
#include <iostream>
#include <string>
#include <mutex>
#include <thread>
#include <map>
#include <vector>

#include <assert.h>

#include "champi_can/msgs_can.pb.h"

#include "champi_can/champi_can.hpp"


using namespace std;

/*
Test frames:
cansend vcan0 123#C1.00.73.64.71.73.44.51
cansend vcan0 123#C0.00.31.32.33.73.64.71
cansend vcan0 123#C2.00.34.35.36

cansend vcan0 123#40.20.66.73.64

cansend vcan0 123#
*/





int main() {
    // Verify that the version of the library that we linked against is
    // compatible with the version of the headers we compiled against.
    GOOGLE_PROTOBUF_VERIFY_VERSION;

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

    string msg = "LesChampignonsSontGentils";

    while(true) {
        if(champi_can_interface.check_if_new_full_msg(id)) {
            cout << "New message received: " << champi_can_interface.get_full_msg(id) << endl;
        }
        champi_can_interface.send(id, msg);

        this_thread::sleep_for(chrono::milliseconds(1000));
    }


    return 0;
}

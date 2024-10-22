

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
#include "champi_can/can_ids.hpp"


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

    ChampiCan champi_can_interface("vcan0", {can_ids::RET_BASE_TEST}, true);


    int ret = champi_can_interface.start();
    if(ret == 0) {
        cout << "CAN interface started successfully" << endl;
    }
    else {
        cout << "Error starting CAN interface" << endl;
        return -1;
    }

    string msg = "123456789123456789123456789";

    ret = champi_can_interface.send(can_ids::BASE_TEST, msg);
    if(ret == 0) {
        cout << "Message sent successfully" << endl;
    }
    else {
        cout << "Error sending message" << endl;
    }

    while(true) {
        if(champi_can_interface.check_if_new_full_msg(can_ids::RET_BASE_TEST)) {
            cout << "New message received: " << champi_can_interface.get_full_msg(can_ids::RET_BASE_TEST) << endl;
        }
        // champi_can_interface.send(can_ids::BASE_TEST, msg);

        this_thread::sleep_for(chrono::milliseconds(1000));
    }
}

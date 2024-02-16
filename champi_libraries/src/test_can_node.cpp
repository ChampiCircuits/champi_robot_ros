#include <iostream>
#include <fstream>
#include <string>
#include "champi_can/msgs_can.pb.h"

using namespace std;

int main() {

    // Verify that the version of the library that we linked against is
    // compatible with the version of the headers we compiled against.
    // GOOGLE_PROTOBUF_VERIFY_VERSION;


    cout << "Hello, World!" << endl;

    msgs_can::BaseVel base_vel_cmd;

    base_vel_cmd.set_x(1.0);
    base_vel_cmd.set_y(2.0);
    base_vel_cmd.set_theta(3.0);

    // serialize the message
    string buffer;
    base_vel_cmd.SerializeToString(&buffer);

    cout << "buffer: " << buffer << endl;

    // deserialize the message
    msgs_can::BaseVel base_vel_cmd2;
    base_vel_cmd2.ParseFromString(buffer);

    cout << "x: " << base_vel_cmd2.x() << endl;
    cout << "y: " << base_vel_cmd2.y() << endl;
    cout << "theta: " << base_vel_cmd2.theta() << endl;

    // Optional:  Delete all global objects allocated by libprotobuf.
    // google::protobuf::ShutdownProtobufLibrary();


    

    return 0;
}

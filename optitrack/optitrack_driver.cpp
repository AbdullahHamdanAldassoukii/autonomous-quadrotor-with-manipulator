#include <optitrack/optitrack.hpp>
#include <optitrack/optitrack_channels.h>
#include <optitrack/common/getopt.h>
#include <optitrack/common/timestamp.h>
#include <lcmtypes/pose_xyzrpy_t.hpp>
#include <lcmtypes/pose_list_t.hpp>
#include <lcm/lcm-cpp.hpp>
#include <netinet/in.h>
#include <sys/socket.h>
#include <iostream>
#include <cmath>


int main(int argc, char** argv)
{
    const char* kInterfaceArg = "interface";
    const char* kQRigidBodyArg = "quadrotor";
    const char* kPURigidBodyArg = "pickup";
    const char* kDORigidBodyArg = "dropoff";
    
    getopt_t* gopt = getopt_create();
    getopt_add_bool(gopt, 'h', "help", 0, "Display this help message.\n");
    getopt_add_string(gopt, 'i', kInterfaceArg, "192.168.3.0", "Local network interface for connecting to Optitrack network");
    getopt_add_int(gopt, 'q', kQRigidBodyArg, "1", "Id of Quadrotor rigid body to publish pose for.");
    getopt_add_int(gopt, 'p', kPURigidBodyArg, "2", "Id of Pick Up rigid body to publish pose for.");
    getopt_add_int(gopt, 'd', kDORigidBodyArg, "3", "Id of Drop Off rigid body to publish pose for.");
    
    
    if (!getopt_parse(gopt, argc, argv, 1) || getopt_get_bool(gopt, "help")) {
        printf("Usage: %s [options]", argv[0]);
        getopt_do_usage(gopt);
        return 1;
    }
    
    std::string interface = getopt_get_string(gopt, kInterfaceArg);
    int QrigidBodyId = getopt_get_int(gopt, kQRigidBodyArg);
    int PUrigidBodyId = getopt_get_int(gopt, kPURigidBodyArg);
    int DOrigidBodyId = getopt_get_int(gopt, kDORigidBodyArg);


    
    // If there's no interface specified, then we'll need to guess
    if (interface.length() == 0)
    {
        interface = guess_optitrack_network_interface();
    }
    // If there's still no interface, we have a problem
    if (interface.length() == 0)
    {
        printf("[optitrack_driver] error could not determine network interface for receiving multicast packets.\n");
        return -1;
    }
    
    SOCKET dataSocket = create_optitrack_data_socket(interface, PORT_DATA);
    
    if (dataSocket == -1) {
        printf("[optitrack_driver] error failed to create socket for interface %s:%d\n", interface.c_str(), PORT_DATA);
        return -1;
    } else {
        printf("[optitrack_driver] successfully created socket for interface %s:%d\n", interface.c_str(), PORT_DATA);
    }
    
    // Code from DataListenThread function in PacketClient.cpp
    char packet[20000];
    socklen_t addrLen = sizeof(sockaddr);
    sockaddr_in incomingAddress;
    std::vector<optitrack_message_t> incomingMessages;
    
    lcm::LCM lcmInstance;
    // MAIN THREAD LOOP
    while (1) {
        // Block until we receive a datagram from the network
        recvfrom(dataSocket, packet, sizeof(packet), 0, (sockaddr*)&incomingAddress, &addrLen);
        incomingMessages = parse_optitrack_packet_into_messages(packet, sizeof(packet));
        pose_list_t PoseList;
        for(auto& msg : incomingMessages) {            
            // Converting from Optitrack with z forward, x left, y up to Maebot with x forward y left, z up gives the
            // following:
            //  opti.z -> quadrotor y
            //  opti.x -> quadrotor x
            //  -opti.y -> quadrotor z
            pose_xyzrpy_t Pose;
            Pose.utime = utime_now();
            Pose.x = msg.x;
            Pose.y = msg.z;
            Pose.z = -msg.y;
            double roll;
            double pitch;
            double yaw;
            toEulerAngle(msg, roll, pitch, yaw);
            Pose.roll = roll;
            Pose.pitch = yaw;
            Pose.yaw = -pitch;
            if(msg.id == QrigidBodyId) {
                PoseList.Q_pose = Pose;

            }
            else if(msg.id == PUrigidBodyId) {
                PoseList.PU_pose = Pose;
            }
            else if(msg.id == DOrigidBodyId) {
                PoseList.DO_pose = Pose;

            }
        }
        PoseList.utime = utime_now();
        lcmInstance.publish("QUADROTOR_POSE_CHANNEL", &PoseList);
    }
    
    // Cleanup options now that we've parsed everything we need
    getopt_destroy(gopt);

    return 0;
}

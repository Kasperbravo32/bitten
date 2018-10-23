/* -----------------------------------------------------------------------
 * Filename: can_driver_node.cpp
 * Author: Kasper Jørgensen
 * Purpose: Create the 'can_driver' node in the Leica Robot system
 * Date of dev.: September 2018 - January 2019
 * 
 * -----------------------------------------------------------------------
 *                      -------  Libraries   -------
 * ----------------------------------------------------------------------- */
#include <linux/can.h>
#include <linux/can/raw.h>

#include <endian.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>

#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "ros/ros.h"
#include <bitten/canopen_msg.h>
#include <global_node_definitions.h>
#include <can_driver_node.h>

#define PROGNAME  "socketcan"
#define VERSION  "0.1.0"

namespace
{

std::sig_atomic_t signalValue;

void onSignal(int value)
{
    signalValue = static_cast<decltype(signalValue)>(value);
}

void processFrame(const struct canfd_frame& frame)
{
    can_msg.can_id = frame.can_id;
    can_msg.lenth = frame.len; //payload is 8 bytes in length
    for(int i = 0; i < (int)frame.len; i++)
    {
        if(i > 7)
        {
            break;
        }
        can_msg.data[i] = frame.data[i];
    }
}

} // namespace
 
 /* ----------------------------------------------------------------------
 *                          -------  Main   -------
 * ----------------------------------------------------------------------- */  
int main(int argc, char** argv)
{   
    using namespace std::chrono_literals;

    // Options
    const char* interface = "can0";

    // Service variables
    struct sigaction sa;
    int rc;

    // CAN connection variables
    struct sockaddr_can addr;
    struct ifreq ifr;
    int sockfd;

    // Register signal handlers
    sa.sa_handler = onSignal;
    ::sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    ::sigaction(SIGINT, &sa, nullptr);
    ::sigaction(SIGTERM, &sa, nullptr);
    ::sigaction(SIGQUIT, &sa, nullptr);
    ::sigaction(SIGHUP, &sa, nullptr);

    // Initialize the signal value to zero
    signalValue = 0;

    // Open the CAN network interface
    sockfd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sockfd == -1)
    {
        ROS_ERROR("socket: %s", strerror(errno));
        return errno;
    }

    // Set a receive filter so we only receive select CAN IDs
    {
        struct can_filter filter[4];
        filter[0].can_id   = 0xCFDD633;
        filter[0].can_mask = CAN_SFF_MASK;
        filter[1].can_id   = 0xCFDD634;
        filter[1].can_mask = CAN_SFF_MASK;
        filter[2].can_id   = 0xCFDD733;
        filter[2].can_mask = CAN_SFF_MASK;
        filter[3].can_id   = 0xCFDD734;
        filter[3].can_mask = CAN_SFF_MASK;

        rc = ::setsockopt(
            sockfd,
            SOL_CAN_RAW,
            CAN_RAW_FILTER,
            &filter,
            sizeof(filter)
        );
        if (rc == -1)
        {
            ROS_ERROR("setsockopt filter: %s", strerror(errno));
            ::close(sockfd);
            return errno;
        }
    }

    // Enable reception of CAN FD frames
    {
        int enable = 1;

        rc = ::setsockopt(
            sockfd,
            SOL_CAN_RAW,
            CAN_RAW_FD_FRAMES,
            &enable,
            sizeof(enable)
        );
        if (rc == -1)
        {
            ROS_ERROR("setsockopt CAN FD: %s", strerror(errno));
            ::close(sockfd);
            return errno;
        }
    }

    // Get the index of the network interface
    std::strncpy(ifr.ifr_name, interface, IFNAMSIZ);
    if (::ioctl(sockfd, SIOCGIFINDEX, &ifr) == -1)
    {
        ROS_ERROR("ioctl: %s", strerror(errno));
        ::close(sockfd);
        return errno;
    }
    
    // Bind the socket to the network interface
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    rc = ::bind(
        sockfd,
        reinterpret_cast<struct sockaddr*>(&addr),
        sizeof(addr)
    );
    if (rc == -1)
    {
        ROS_ERROR("bind: %s", strerror(errno));
        ::close(sockfd);
        return errno;
    }

    ROS_INFO("Initiating node");
    ros::init(argc, argv, "can_driver_node");
    ros::NodeHandle n;

    ROS_INFO("Publishing to can_topic");
    ros::Publisher canPub = n.advertise<bitten::canopen_msg>("can_topic", 1000);

    ros::Rate loop_rate(LOOP_RATE_INT);
    sleep(1);

    ROS_INFO("CAN read started");
    
    /* -------------------------------------------------
    *     SUPERLOOP
    * ------------------------------------------------- */
    while ( ( 0 == signalValue ) && ros::ok() )
    {
        struct canfd_frame frame;

        // Read in a CAN frame
        auto numBytes = ::read(sockfd, &frame, CANFD_MTU);
        switch (numBytes)
        {
            case CAN_MTU:
            {
                processFrame(frame);
            }
                break;
            case CANFD_MTU:
                break;
            case -1:
                // Check the signal value on interrupt
                if (EINTR == errno)
                    continue;

                // Delay before continuing
                ROS_ERROR("read: %s", strerror(errno));
                std::this_thread::sleep_for(100ms);
                return errno;
            default:
                continue;
        }
        canPub.publish(can_msg);
    }

    // Cleanup
    if (::close(sockfd) == -1)
    {
        ROS_ERROR("close: %s", strerror(errno));
        return errno;
    }

    return EXIT_SUCCESS;
}

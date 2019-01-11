/* -----------------------------------------------------------------------
 * Filename: can_driver_node.cpp
 * Author: Frederik Snedevind & Kasper Banke Jørgensen
 * Purpose: Create the 'can_driver' node in the Leica Robot system
 * Date of dev.: September 2018 - January 2019
 * -----------------------------------------------------------------------
 *                      -------  Libraries   -------
 * ----------------------------------------------------------------------- */
#include "ros/ros.h"
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>

#include <can_driver_node.h>
#include <bitten/can_msg.h>
#include <global_node_definitions.h>

 /* ----------------------------------------------------------------------
 *                      -------  Initializing   -------
 * ----------------------------------------------------------------------- */
void processFrame(const struct can_frame& frame);

 /* ----------------------------------------------------------------------
 *                       -------  Global variables   -------
 * ----------------------------------------------------------------------- */
bitten::can_msg can_msg;

 /* ----------------------------------------------------------------------
 *                          -------  Main   -------
 * ----------------------------------------------------------------------- */  
int main(int argc, char** argv)
{
    // CAN connection variables
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;
    const char* interface = "can0";
    int sock;
    int rc;

    // Open the CAN network interface
    sock = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock == -1)
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
            sock,
            SOL_CAN_RAW,
            CAN_RAW_FILTER,
            &filter,
            sizeof(filter)
        );
        if (rc == -1)
        {
            ROS_ERROR("setsockopt filter: %s", strerror(errno));
            ::close(sock);
            return errno;
        }
    }

    // Get the index of the network interface
    std::strncpy(ifr.ifr_name, interface, IFNAMSIZ);
    if (::ioctl(sock, SIOCGIFINDEX, &ifr) == -1)
    {
        ROS_ERROR("ioctl: %s", strerror(errno));
        ::close(sock);
        return errno;
    }
    
    // Bind the socket to the network interface
    addr.can_family = PF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    rc = ::bind(
        sock,
        reinterpret_cast<struct sockaddr*>(&addr),
        sizeof(addr)
    );
    if (rc == -1)
    {
        ROS_ERROR("bind: %s", strerror(errno));
        ::close(sock);
        return errno;
    }

    ros::init(argc, argv, "can_driver_node");
    ros::NodeHandle n;

    ros::Publisher canPub = n.advertise<bitten::can_msg>("can_topic", LOOP_RATE_INT);

    ros::Rate loop_rate(LOOP_RATE_INT);

    sleep(1);
    if (canPub)
        ROS_INFO("Initiated CAN_node");
    else
        ROS_INFO("Didn't initiate CAN_node");
    
    /* -------------------------------------------------
    *     SUPERLOOP
    * ------------------------------------------------- */
    while ( ros::ok() )
    {
        // Read in a CAN frame
        auto numBytes = ::read(sock, &frame, CAN_MTU);
        switch (numBytes)
        {
            case CAN_MTU:
            {
                can_msg.can_id = frame.can_id;
                can_msg.lenth = frame.can_dlc; //payload is 8 bytes in length
                for(int i = 0; i < 8; i++)
                    can_msg.data[i] = frame.data[i];
        
                canPub.publish(can_msg);
                break;
            }
            case -1:
                ROS_ERROR("read: %s", strerror(errno));
                return errno;
            default:
                continue;
        }
    }

    // Cleanup
    if (::close(sock) == -1)
    {
        ROS_ERROR("close: %s", strerror(errno));
        return errno;
    }
    return 0;
}

void processFrame(const struct can_frame& frame)
{
    can_msg.can_id = frame.can_id;
    can_msg.lenth = frame.can_dlc; //payload is 8 bytes in length
    for(int i = 0; i < 8; i++)
        can_msg.data[i] = frame.data[i];
}

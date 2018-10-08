/* -----------------------------------------------------------------------
 * Filename: manual_node.cpp
 * Author: Kasper Jørgensen
 * Purpose: Create the 'manual' node in the Leica Robot system
 * Date of dev.: September 2018 - January 2019
 * 
 * -----------------------------------------------------------------------
 *                      -------  Libraries   -------
 * ----------------------------------------------------------------------- */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Joy.h>
#include <bitten/control_msg.h>

 /* ----------------------------------------------------------------------
 *                      -------  Initializing   -------
 * ----------------------------------------------------------------------- */
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

/* ----------------------------------------------------------------------
 *                       -------  Constants   -------
 * ----------------------------------------------------------------------- */
const int loop_rate_int    = 50;

/* ----------------------------------------------------------------------
 *                       -------  Global variables   -------
 * ----------------------------------------------------------------------- */
struct msg_type
{
    std::string nodeName;
    std::string programName;
    uint32_t flags;
    uint8_t id;
    float jointPosition[6];
    float jointVelocity[6];
    uint8_t jointToMove[6];
}controlMsg;

// int8_t msgReceived = 0;
float jointVel[6];

 /* ----------------------------------------------------------------------
 *                          -------  Main   -------
 * ----------------------------------------------------------------------- */        
int main(int argc, char **argv)
{
    ROS_INFO("Initiating system...");
    ros::init(argc, argv, "manual_node");
    ros::NodeHandle n;

    ROS_INFO("Subscribing to \"joy_topic\"");
    ros::Subscriber joySub = n.subscribe<sensor_msgs::Joy>("joy", 1000, &joyCallback);
    
    ROS_INFO("Publishing to \"manual_topic\"");
    ros::Publisher manualPub = n.advertise<bitten::control_msg>("manual_topic", 1000);

    controlMsg.nodeName = "manual_node";

    ros::Rate loop_rate(loop_rate_int);

    ros::spinOnce();

    bitten::control_msg msg;
    msg.nodeName = "manual node";

/* -------------------------------------------------
*     SUPERLOOP
* ------------------------------------------------- */
    while (ros::ok())
    {    
        for(int i = 0; i < 6; i++)
            msg.jointsVelocity[i] = controlMsg.jointVelocity[i];
        
        manualPub.publish(msg);
 
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    // msgReceived = 1;

    //joint 1
    controlMsg.jointVelocity[0] = joy->axes[0];
    //joint 2
    controlMsg.jointVelocity[1] = joy->axes[4];
    //joint 3
    controlMsg.jointVelocity[2] = joy->axes[1];
    //joint 4
    if(joy->buttons[15])
        controlMsg.jointVelocity[3] = -1;
    else if(joy->buttons[16])
        controlMsg.jointVelocity[3] = 1;
    else
        controlMsg.jointVelocity[3] = 0;
    //joint 5
    controlMsg.jointVelocity[4] = joy->axes[3];
    //joint 6
    if(joy->buttons[1])
        controlMsg.jointVelocity[5] = 1;
    else if(joy->buttons[3])
        controlMsg.jointVelocity[5] = -1;
    else
        controlMsg.jointVelocity[5] = 0;

    std::cout << "joint1: " << controlMsg.jointVelocity[0] << std::endl
            << "joint2: " << controlMsg.jointVelocity[1] << std::endl
            << "joint3: " << controlMsg.jointVelocity[2] << std::endl
            << "joint4: " << controlMsg.jointVelocity[3] << std::endl
            << "joint5: " << controlMsg.jointVelocity[4] << std::endl
            << "joint6: " << controlMsg.jointVelocity[5] << std::endl << std::endl;
}
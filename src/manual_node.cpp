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
#include <bitten/control_msg.h>
#include <manual_node.h>
#include <global_node_definitions.h>

 /* ----------------------------------------------------------------------
 *                          -------  Main   -------
 * ----------------------------------------------------------------------- */        
int main(int argc, char **argv)
{
    ROS_INFO("Initiating system...");
    ros::init(argc, argv, "manual_node");
    ros::NodeHandle n;

    ROS_INFO("Subscribing to \"%s\"", topicNames[JOY_TOPIC].c_str());
    ros::Subscriber joySub = n.subscribe<sensor_msgs::Joy>("joy", 1000, &joyCallback);
    
    ROS_INFO("Publishing to \"%s\"", topicNames[MANUAL_TOPIC].c_str());
    ros::Publisher manualPub = n.advertise<bitten::control_msg>("manual_topic", 1000);

    manual_msg.nodeName = "manual node";

    ros::Rate loop_rate(LOOP_RATE_INT);
    /* -------------------------------------------------
    *     SUPERLOOP
    * ------------------------------------------------- */
    while (ros::ok())
    {
        if((manual_msg.jointVelocity[0] == 1) || (manual_msg.jointVelocity[0] == -1))
        {
            if((manual_msg.jointVelocity[2] >= -0.3) && (manual_msg.jointVelocity[2] <= 0.3))
                manual_msg.jointVelocity[2] = 0;
        }
        if((manual_msg.jointVelocity[2] == 1) || (manual_msg.jointVelocity[2] == -1))
        {
            if((manual_msg.jointVelocity[0] >= -0.3) && (manual_msg.jointVelocity[0] <= 0.3))
                manual_msg.jointVelocity[0] = 0;
        }
        if((manual_msg.jointVelocity[1] == 1) || (manual_msg.jointVelocity[1] == -1))
        {
            if((manual_msg.jointVelocity[4] >= -0.3) && (manual_msg.jointVelocity[4] <= 0.3))
                manual_msg.jointVelocity[4] = 0;
        }
        if((manual_msg.jointVelocity[4] == 1) || (manual_msg.jointVelocity[4] == -1))
        {
            if((manual_msg.jointVelocity[1] >= -0.3) && (manual_msg.jointVelocity[1] <= 0.3))
                manual_msg.jointVelocity[1] = 0;
        }
        
        manualPub.publish(manual_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    //joint 1
    manual_msg.jointVelocity[0] = joy->axes[0];
    //joint 2
    manual_msg.jointVelocity[1] = joy->axes[4];
    //joint 3
    manual_msg.jointVelocity[2] = joy->axes[1];
    //joint 4
    if(joy->buttons[15])
        manual_msg.jointVelocity[3] = -1;
    else if(joy->buttons[16])
        manual_msg.jointVelocity[3] = 1;
    else
        manual_msg.jointVelocity[3] = 0;
    //joint 5
    manual_msg.jointVelocity[4] = joy->axes[3];
    //joint 6
    if(joy->buttons[1])
        manual_msg.jointVelocity[5] = 1;
    else if(joy->buttons[3])
        manual_msg.jointVelocity[5] = -1;
    else
        manual_msg.jointVelocity[5] = 0;
    
    std::cout << "Joint 1: " << manual_msg.jointVelocity[0] << std::endl
            << "Jøoint 2: " << manual_msg.jointVelocity[1] << std::endl
            << "Joint 3: " << manual_msg.jointVelocity[2] << std::endl
            << "Joint 4: " << manual_msg.jointVelocity[3] << std::endl
            << "Joint 5: " << manual_msg.jointVelocity[4] << std::endl
            << "Joint 6: " << manual_msg.jointVelocity[5] << std::endl << std::endl;
}
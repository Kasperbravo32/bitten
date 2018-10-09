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
#include <global_node_definitions.h>

 /* ----------------------------------------------------------------------
 *                      -------  Initializing   -------
 * ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------
 *                       -------  Global variables   -------
 * ----------------------------------------------------------------------- */
bitten::control_msg wp_msg;

 /* ----------------------------------------------------------------------
 *                          -------  Main   -------
 * ----------------------------------------------------------------------- */        
int main(int argc, char **argv)
{
    ros::init(argc,argv,nodeNames[WP_NODE].c_str());
    ros::NodeHandle n;

    ROS_INFO("Started %s",nodeNames[WP_NODE].c_str());

    ros::Publisher wpPub = n.advertise<bitten::control_msg>(topicNames[WP_TOPIC].c_str(),3*LOOP_RATE_INT);
    if (wpPub)
        ROS_INFO("Publishing on %s",topicNames[WP_TOPIC].c_str());
    else
        ROS_INFO("Couldn't publish on %s",topicNames[WP_TOPIC].c_str());

    
    //ros::Subscriber 


    wp_msg.nodeName = "Waypoint Node";

    ros::Rate loop_rate(LOOP_RATE_INT);
    while(ros::ok())
    {


        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}





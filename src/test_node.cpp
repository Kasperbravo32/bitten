/* -----------------------------------------------------------------------
 * Filename: test_node.cpp
 * Author: Kasper Jørgensen
 * Purpose: Create the 'test' node in the Leica Robot system
 * Date of dev.: September 2018 - January 2019
 * 
 * -----------------------------------------------------------------------
 *                      -------  Libraries   -------
 * ----------------------------------------------------------------------- */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <bitten/control_msg.h>
#include <commander_node.h>
#include <global_node_definitions.h>

struct msg_type
{
    std::string node_name;
    std::string program_name;
    uint32_t flags;
    uint8_t id;
    float joint_position[6];
    float joint_velocity[6];
    uint8_t joint_to_move[6];
}fb_control_msg;

float joint_vel[6];

/* ----------------------------------------------------------------------
 *                          -------  Main   -------
 * ----------------------------------------------------------------------- */
int main(int argc, char **argv)
{
    /*
    ROS_INFO("Initiating system...");
    ros::init(argc, argv, "test_node");
    ros::NodeHandle n;

    ROS_INFO("Publishing to \"test_topic\"");
    ros::Publisher test_pub = n.advertise<bitten::control_msg>("test_topic", 1000);

    ros::Rate loop_rate(LOOP_RATE_INT);

    ROS_INFO("Initiating system...");
    Robot_s TX90;
    // Staubli_TX90_initializer(TX90);

    
    for (int i = 0; i < 6; i++)
        Waypoint_0.joint_positions[i] = 0;

    //std::string joint_names[6] = {"joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"};
 

    /*          -------------------------------------           */
    /* --------------- Define Waypoints below ----------------- */
    /*          -------------------------------------           */
/*
    int NumberofWaypoints = 5;                                  /* Number of waypoints excluding waypoint_0.                                                    */
    //int RemainingWaypoints = NumberofWaypoints;
/*
    Waypoint_s WaypointBank[NumberofWaypoints];

    ROS_INFO("Loading Waypoints...");
    /* Define joints for WP#1 */
    /*WaypointBank[0].joint_positions[0] = (95.0/180.0)*3.14;
    WaypointBank[0].joint_positions[1] = -(45.0/180.0)*3.14;
    WaypointBank[0].joint_positions[2] = -(83.0/180.0)*3.14;
    WaypointBank[0].joint_positions[3] = (3.0/180.0)*3.14;
    WaypointBank[0].joint_positions[4] = -(55.0/180.0)*3.14;
    WaypointBank[0].joint_positions[5] = (0.0/180.0)*3.14;

    /* Define joints for WP#2 */
    /**WaypointBank[1].joint_positions[0] = (128.0/180.0)*3.14;
    WaypointBank[1].joint_positions[1] = -(100.0/180.0)*3.14;
    WaypointBank[1].joint_positions[2] = -(33.0/180)*3.14;
    WaypointBank[1].joint_positions[3] = (6.0/180)*3.14;
    WaypointBank[1].joint_positions[4] = -(50.0/180)*3.14;
    WaypointBank[1].joint_positions[5] = (0.0/180)*3.14;

    /* Define joints for WP#3 */
    /*WaypointBank[2].joint_positions[0] = (95.0/180)*3.14;;
    WaypointBank[2].joint_positions[1] = -(45.0/180)*3.14;
    WaypointBank[2].joint_positions[2] = -(83.0/180)*3.14;
    WaypointBank[2].joint_positions[3] = (3.0/180)*3.14;
    WaypointBank[2].joint_positions[4] = -(55.0/180)*3.14;
    WaypointBank[2].joint_positions[5] = (0.0/180)*3.14;

    /* Define joints for WP#4 */
    // WaypointBank[3].joint_positions[0] = (63.0/180)*3.14;
    // WaypointBank[3].joint_positions[1] = -(97.0/180)*3.14;
    // WaypointBank[3].joint_positions[2] = -(40.0/180)*3.14;
    // WaypointBank[3].joint_positions[3] = (0.0/180)*3.14;
    // WaypointBank[3].joint_positions[4] = -(48.0/180)*3.14;
    // WaypointBank[3].joint_positions[5] = (0.0/180)*3.14;

    // /* Define joints for WP#5 */
    // WaypointBank[4].joint_positions[0] = (95.0/180)*3.14;;
    // WaypointBank[4].joint_positions[1] = -(45.0/180)*3.14;   
    // WaypointBank[4].joint_positions[2] = -(83.0/180)*3.14;
    // WaypointBank[4].joint_positions[3] = (3.0/180)*3.14;
    // WaypointBank[4].joint_positions[4] = -(55.0/180)*3.14;
    // WaypointBank[4].joint_positions[5] = (0.0/180)*3.14;

    // ROS_INFO("Loaded: %d Waypoints.", NumberofWaypoints);

    // bitten::control_msg test_msg;

    while (ros::ok())
    {
        
        // for(int i = 0; i < NumberofWaypoints + 1; i++)
        // {
            
        //     test_pub.publish(test_msg);

        // }

        // ros::spinOnce();
        // loop_rate.sleep();
    }

    return 0;
}
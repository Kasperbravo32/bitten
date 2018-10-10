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
#include <bitten/feedback_msg.h>
#include <global_node_definitions.h>
#include "wp_node.h"


 /* ----------------------------------------------------------------------
 *                      -------  Initializing   -------
 * ----------------------------------------------------------------------- */
void fbCallback(const bitten::feedback_msg::ConstPtr& feedback);

/* ----------------------------------------------------------------------
 *                       -------  Global variables   -------
 * ----------------------------------------------------------------------- */
bitten::control_msg wp_msg;
bitten::feedback_msg fb_msg;

bool readyForNextWp = false;
bool transmitWpReady = false;
bool newConnection = true;
bool connectionEstablished = false;


int NumberofWaypoints = 5;                                  /* Number of waypoints excluding waypoint_0.                                                    */
int RemainingWaypoints = NumberofWaypoints;

Waypoint_s Waypoint_0;
Waypoint_s WaypointBank[5];


 /* ----------------------------------------------------------------------
 *                          -------  Main   -------
 * ----------------------------------------------------------------------- */        
int main(int argc, char **argv)
{
    ros::init(argc,argv,nodeNames[WP_NODE]);
    ros::NodeHandle n;

    wp_msg.nodeName = nodeNames[WP_TOPIC].c_str();
    wp_msg.id       = WP_ID;

    ROS_INFO("Started %s",nodeNames[WP_NODE].c_str());

    ros::Publisher wpPub = n.advertise<bitten::control_msg>(topicNames[WP_TOPIC],3*LOOP_RATE_INT);
    if (wpPub)
        ROS_INFO("Publishing on %s",topicNames[WP_TOPIC].c_str());
    else
        ROS_INFO("Couldn't publish on %s",topicNames[WP_TOPIC].c_str());

    
    ROS_INFO("Subscribing to %s",topicNames[FEEDBACK_TOPIC].c_str());
    ros::Subscriber feedbackSub = n.subscribe<bitten::feedback_msg>(topicNames[FEEDBACK_TOPIC], 3*LOOP_RATE_INT, &fbCallback);
    if (feedbackSub)
        ROS_INFO("Subscribed to \"%s\"!", topicNames[FEEDBACK_TOPIC].c_str());
    else
        ROS_INFO("Couldn't subscribe to \"%s\".",topicNames[FEEDBACK_TOPIC].c_str());

    wp_msg.nodeName = "Waypoint Node";
    ros::Rate loop_rate(LOOP_RATE_INT);

    ROS_INFO("Loading Waypoints...");    

    Waypoint_0.jointPosition[0] = 0;
    Waypoint_0.jointPosition[1] = 0;
    Waypoint_0.jointPosition[2] = 0;
    Waypoint_0.jointPosition[3] = 0;
    Waypoint_0.jointPosition[4] = 0;
    Waypoint_0.jointPosition[5] = 0;
    Waypoint_0.waypointName = "Waypoint_0";

    /* Define joints for WP#1 */
    WaypointBank[0].jointPosition[0] = (95.0/180.0)*3.14;
    WaypointBank[0].jointPosition[1] = -(45.0/180.0)*3.14;
    WaypointBank[0].jointPosition[2] = -(83.0/180.0)*3.14;
    WaypointBank[0].jointPosition[3] = (3.0/180.0)*3.14;
    WaypointBank[0].jointPosition[4] = -(55.0/180.0)*3.14;
    WaypointBank[0].jointPosition[5] = (0.0/180.0)*3.14;
    WaypointBank[0].waypointName = "Waypoint 1";

    /* Define joints for WP#2 */
    WaypointBank[1].jointPosition[0] = (128.0/180.0)*3.14;
    WaypointBank[1].jointPosition[1] = -(100.0/180.0)*3.14;
    WaypointBank[1].jointPosition[2] = -(33.0/180)*3.14;
    WaypointBank[1].jointPosition[3] = (6.0/180)*3.14;
    WaypointBank[1].jointPosition[4] = -(50.0/180)*3.14;
    WaypointBank[1].jointPosition[5] = (0.0/180)*3.14;
    WaypointBank[1].waypointName = "Waypoint 2";

    /* Define joints for WP#3 */
    WaypointBank[2].jointPosition[0] = (95.0/180)*3.14;;
    WaypointBank[2].jointPosition[1] = -(45.0/180)*3.14;
    WaypointBank[2].jointPosition[2] = -(83.0/180)*3.14;
    WaypointBank[2].jointPosition[3] = (3.0/180)*3.14;
    WaypointBank[2].jointPosition[4] = -(55.0/180)*3.14;
    WaypointBank[2].jointPosition[5] = (0.0/180)*3.14;
    WaypointBank[2].waypointName = "Waypoint 3";

    /* Define joints for WP#4 */
    WaypointBank[3].jointPosition[0] = (63.0/180)*3.14;
    WaypointBank[3].jointPosition[1] = -(97.0/180)*3.14;
    WaypointBank[3].jointPosition[2] = -(40.0/180)*3.14;
    WaypointBank[3].jointPosition[3] = (0.0/180)*3.14;
    WaypointBank[3].jointPosition[4] = -(48.0/180)*3.14;
    WaypointBank[3].jointPosition[5] = (0.0/180)*3.14;
    WaypointBank[3].waypointName = "Waypoint 4";

    /* Define joints for WP#5 */
    WaypointBank[4].jointPosition[0] = (95.0/180)*3.14;;
    WaypointBank[4].jointPosition[1] = -(45.0/180)*3.14;   
    WaypointBank[4].jointPosition[2] = -(83.0/180)*3.14;
    WaypointBank[4].jointPosition[3] = (3.0/180)*3.14;
    WaypointBank[4].jointPosition[4] = -(55.0/180)*3.14;
    WaypointBank[4].jointPosition[5] = (0.0/180)*3.14;
    WaypointBank[4].waypointName = "Waypoint 5";

    static int connectionTimer = 10*LOOP_RATE_INT;
    while(ros::ok())
    {
        if (connectionEstablished)
        {
            if (newConnection)
            {
                ROS_INFO("Connected!");
                newConnection = false;
                readyForNextWp = true;
            }

            if (readyForNextWp == true)
            {
                ROS_INFO("Sending wp: %s", WaypointBank[NumberofWaypoints - RemainingWaypoints].waypointName.c_str());
                wp_msg.flags = NEW_WAYPOINT;
                wp_msg.programName = WaypointBank[NumberofWaypoints - RemainingWaypoints].waypointName;

                for (int i = 0; i < 6; i++)
                {
                    wp_msg.jointPosition[i] = WaypointBank[NumberofWaypoints - RemainingWaypoints].jointPosition[i];
                }
                transmitWpReady = true;
                readyForNextWp = false;
            }
        }

        else
        {
            static int timer = LOOP_RATE_INT;
            if (! --timer)
            {
                ROS_INFO("Trying to establish connection...");
                wp_msg.flags = ESTABLISH_CONNECTION;
                transmitWpReady = true;
                timer = LOOP_RATE_INT;
            }
        }
        if (transmitWpReady)
        {
            wpPub.publish(wp_msg);
            transmitWpReady = false;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}


/* ----------------------------------------------------------------------
 *                -------  Feedback Callback function   -------
 * ----------------------------------------------------------------------- */ 
void fbCallback(const bitten::feedback_msg::ConstPtr& feedback)
{
    if (feedback->recID == WP_ID)
    {
        if (feedback->flags == PING)
        {
            wp_msg.flags = PONG;
            transmitWpReady = true;
        }

        if (connectionEstablished == false && feedback->flags == ACK)
            connectionEstablished = true;

        else if (connectionEstablished == false && feedback->flags == DENIED)
            ROS_INFO("Connection denied");

        if (feedback->flags & WAYPOINT_REACHED)
        {
            ROS_INFO("Received WAYPOINT_REACHED");
            if (--RemainingWaypoints > 0)
            {
                readyForNextWp = true;
            }
            else
            {
                ROS_INFO("Finished all waypoints. terminating connection.");
                wp_msg.flags = TERMINATE_CONNECTION;
                transmitWpReady = true;
            }
        }
    }
}

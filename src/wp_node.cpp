/* -----------------------------------------------------------------------
 * Filename: wp_node.cpp
 * Author: Frederik Snedevind & Kasper Banke Jørgensen
 * Purpose: Create the 'waypoint' node in the Leica Robot system
 * Date of dev.: September 2018 - January 2019
 * 
 * -----------------------------------------------------------------------
 *                      -------  Libraries   -------
 * ----------------------------------------------------------------------- */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include <iostream>
#include <fstream>
#include <stdlib.h>     /* atof */
#include <sensor_msgs/Joy.h>
#include <bitten/control_msg.h>
#include <bitten/feedback_msg.h>
#include <global_node_definitions.h>
#include "wp_node.h"

using namespace std;

int NumberofWaypoints;                  /* Number of waypoints excluding waypoint_0.                    */
int RemainingWaypoints;                 /* Number of remaining waypoints to perform.                    */
Waypoint_s WaypointBank[16];            /* Create an empty bank of waypoints, to contain future tests   */

 /* ----------------------------------------------------------------------
 *                          -------  Main   -------
 * ----------------------------------------------------------------------- */        
int main(int argc, char **argv)
{
    ros::init(argc,argv,nodeNames[WP_NODE]);
    ros::NodeHandle n;

    wp_msg.nodeName = nodeNames[WP_TOPIC].c_str();
    wp_msg.id       = WP_ID;

    ros::Publisher wpPub = n.advertise<bitten::control_msg>(topicNames[WP_TOPIC],3*LOOP_RATE_INT);
    ros::Subscriber feedbackSub = n.subscribe<bitten::feedback_msg>(topicNames[FEEDBACK_TOPIC], 3*LOOP_RATE_INT, &fbCallback);

    // wp_msg.nodeName = "Waypoint Node";
    ros::Rate loop_rate(LOOP_RATE_INT);

    if (wpPub && feedbackSub)
        ROS_INFO("Initiated %s", nodeNames[WP_NODE].c_str());
    else
        ROS_INFO("Failed to initiate %s",nodeNames[WP_NODE].c_str());

    // ROS_INFO("Loading Waypoints...");    



    Waypoint_0.jointPosition[0] = (0.1/180)*3.14;
    Waypoint_0.jointPosition[1] = (0.1/180)*3.14;
    Waypoint_0.jointPosition[2] = (0.1/180)*3.14;
    Waypoint_0.jointPosition[3] = (0.1/180)*3.14;
    Waypoint_0.jointPosition[4] = (0.1/180)*3.14;
    Waypoint_0.jointPosition[5] = (0.1/180)*3.14;
    Waypoint_0.waypointName = "Waypoint_0"; 
    

/* --------------------------------------------------------
    Demo Waypoints
    Set NumberofWaypoints = 7
   -------------------------------------------------------- */
    // Waypoint_s WaypointBank[NumberofWaypoints];

    // /* Define joints for WP#1 */
    // WaypointBank[0].jointPosition[0] = (2/180.0)*3.14;
    // WaypointBank[0].jointPosition[1] = (0.1/180.0)*3.14;
    // WaypointBank[0].jointPosition[2] = (0.1/180.0)*3.14;
    // WaypointBank[0].jointPosition[3] = (0.1/180.0)*3.14;
    // WaypointBank[0].jointPosition[4] = (0.1/180.0)*3.14;
    // WaypointBank[0].jointPosition[5] = (0.1/180.0)*3.14;
    // WaypointBank[0].waypointName = "Waypoint 0";

    // WaypointBank[1].jointPosition[0] = (95.0/180.0)*3.14;
    // WaypointBank[1].jointPosition[1] = -(45.0/180.0)*3.14;
    // WaypointBank[1].jointPosition[2] = -(83.0/180.0)*3.14;
    // WaypointBank[1].jointPosition[3] = (3.0/180.0)*3.14;
    // WaypointBank[1].jointPosition[4] = -(55.0/180.0)*3.14;
    // WaypointBank[1].jointPosition[5] = (10.0/180.0)*3.14;
    // WaypointBank[1].waypointName = "Waypoint 1";

    // /* Define joints for WP#2 */
    // WaypointBank[2].jointPosition[0] = (128.0/180.0)*3.14;
    // WaypointBank[2].jointPosition[1] = -(90.0/180.0)*3.14;
    // WaypointBank[2].jointPosition[2] = -(33.0/180)*3.14;
    // WaypointBank[2].jointPosition[3] = (6.0/180)*3.14;
    // WaypointBank[2].jointPosition[4] = -(50.0/180)*3.14;
    // WaypointBank[2].jointPosition[5] = (1.0/180)*3.14;
    // WaypointBank[2].waypointName = "Waypoint 2";

    // /* Define joints for WP#3 */
    // WaypointBank[3].jointPosition[0] = (95.0/180)*3.14;;
    // WaypointBank[3].jointPosition[1] = -(45.0/180)*3.14;
    // WaypointBank[3].jointPosition[2] = -(83.0/180)*3.14;
    // WaypointBank[3].jointPosition[3] = (3.0/180)*3.14;
    // WaypointBank[3].jointPosition[4] = -(55.0/180)*3.14;
    // WaypointBank[3].jointPosition[5] = (1.0/180)*3.14;
    // WaypointBank[3].waypointName = "Waypoint 3";

    // /* Define joints for WP#4 */
    // WaypointBank[4].jointPosition[0] = (63.0/180)*3.14;
    // WaypointBank[4].jointPosition[1] = -(87.0/180)*3.14;
    // WaypointBank[4].jointPosition[2] = -(40.0/180)*3.14;
    // WaypointBank[4].jointPosition[3] = (1.0/180)*3.14;
    // WaypointBank[4].jointPosition[4] = -(48.0/180)*3.14;
    // WaypointBank[4].jointPosition[5] = (1.0/180)*3.14;
    // WaypointBank[4].waypointName = "Waypoint 4";

    // /* Define joints for WP#5 */
    // WaypointBank[5].jointPosition[0] = (95.0/180)*3.14;;
    // WaypointBank[5].jointPosition[1] = -(45.0/180)*3.14;   
    // WaypointBank[5].jointPosition[2] = -(83.0/180)*3.14;
    // WaypointBank[5].jointPosition[3] = (3.0/180)*3.14;
    // WaypointBank[5].jointPosition[4] = -(55.0/180)*3.14;
    // WaypointBank[5].jointPosition[5] = (1.0/180)*3.14;
    // WaypointBank[5].waypointName = "Waypoint 5";

    // WaypointBank[6].jointPosition[0] = (0.1/180.0)*3.14;
    // WaypointBank[6].jointPosition[1] = (0.1/180.0)*3.14;   
    // WaypointBank[6].jointPosition[2] = (0.1/180.0)*3.14;
    // WaypointBank[6].jointPosition[3] = (0.1/180.0)*3.14;
    // WaypointBank[6].jointPosition[4] = (0.1/180.0)*3.14;
    // WaypointBank[6].jointPosition[5] = (0.1/180.0)*3.14;
    // WaypointBank[6].waypointName = "Waypoint 6";


/* --------------------------------------------------------
    Beer Waypoints
    Set NumberofWaypoints = 4
   -------------------------------------------------------- */

    static int connectionTimer = 10*LOOP_RATE_INT;
    bool newConnection = true;

    while(ros::ok())
    {
        if (connectionEstablished)
        {
            if (newConnection)
            {
                newConnection = false;
            }

            if (readyForNextWp == true)
            {


                ROS_INFO("Sending wp: %s", WaypointBank[NumberofWaypoints - RemainingWaypoints].waypointName.c_str());
                wp_msg.flags = NEW_WAYPOINT;
                wp_msg.programName = WaypointBank[NumberofWaypoints - RemainingWaypoints].waypointName;

                for (int i = 0; i < 6; i++)
                    wp_msg.jointPosition[i] = WaypointBank[NumberofWaypoints - RemainingWaypoints].jointPosition[i];

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



        if (feedback->flags & GOAL_REACHED)
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

        if (feedback->flags & START_TEST)
        {
            string inputString;
            string inputFilePath = "/home/frederik/catkin_ws/src/bitten/tests/";
            inputFilePath += feedback->programName;
            ifstream inputFile(inputFilePath);

            if (inputFile.is_open())
            {

                bool FirstRead = true;
                int waypointCounter = 1;
                int jointCounter;
                string::size_type sz;
                string testName;

                inputFile >> testName;

                for (int i = 0; inputFile >> WaypointBank[i].waypointName; i++, waypointCounter++)
                {
                    cout << endl << WaypointBank[i].waypointName << ": ";
                    for (int n = 0; n < 6; n++)
                    {
                        inputFile >> WaypointBank[i].jointPosition[n];
                        cout << WaypointBank[i].jointPosition[n] << "\t";
                    }
                        
                    NumberofWaypoints = waypointCounter;
                    RemainingWaypoints = NumberofWaypoints;
                }

                cout << "\nTest name: " << testName << endl;
                cout << "Total number of waypoints: " << NumberofWaypoints << endl;
                
                inputFile.close();

                readyForNextWp = true;
            }
            else
            {
                cout << "Couldn't open: " << inputFilePath << endl;
            }
        }

    }
}

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
#include <pwd.h>
#include <sensor_msgs/Joy.h>
#include <bitten/control_msg.h>
#include <bitten/feedback_msg.h>
#include <global_node_definitions.h>
#include "wp_node.h"

using namespace std;

int NumberofWaypoints;                  /* Number of waypoints excluding waypoint_0.                    */
int RemainingWaypoints;                 /* Number of remaining waypoints to perform.                    */
Waypoint_s WaypointBank[16];            /* Create an empty bank of waypoints, to contain future tests   */

passwd* pw = getpwuid(getuid());
string path(pw->pw_dir);

string testsPath        = path + "/catkin_ws/src/bitten/tests/";
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

    ros::Rate loop_rate(LOOP_RATE_INT);

    sleep(1);
    
    if (wpPub && feedbackSub)
        ROS_INFO("Initiated %s", nodeNames[WP_NODE].c_str());
    else
        ROS_INFO("Failed to initiate %s",nodeNames[WP_NODE].c_str());

    bool newConnection = true;

    while(ros::ok())
    {
        if (readyForNextWp == true)
        {
            wp_msg.flags = NEW_WAYPOINT;
            wp_msg.programName = WaypointBank[NumberofWaypoints - RemainingWaypoints].waypointName;
            ROS_INFO("Moving robot to: %s", wp_msg.programName.c_str());

            for (int i = 0; i < 6; i++)
                wp_msg.jointPosition[i] = WaypointBank[NumberofWaypoints - RemainingWaypoints].jointPosition[i];

            transmitWpReady = true;
            readyForNextWp = false;
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
    if (feedback->recID == WP_ID || feedback->recID == ALL_ID)
    {
        if (feedback->flags == PING)
        {
            wp_msg.flags = PONG;
            transmitWpReady = true;
        }

        if (feedback->flags & GOAL_REACHED)
        {            
            ROS_INFO(" OK!");

            if (--RemainingWaypoints > 0)
                readyForNextWp = true;

            else
            {
                wp_msg.flags = TEST_DONE_FLAG;
                transmitWpReady = true;
            }
        }

        if (feedback->flags & START_TEST)
        {
            string inputFilePath = testsPath + feedback->programName;
            ifstream inputFile(inputFilePath);

            if (inputFile.is_open())
            {
                int waypointCounter = 1;
                string testName;
                inputFile >> testName;

                for (int i = 0; inputFile >> WaypointBank[i].waypointName; i++, waypointCounter++)
                {
                    for (int n = 0; n < 6; n++)
                        inputFile >> WaypointBank[i].jointPosition[n];

                    NumberofWaypoints = waypointCounter;
                    RemainingWaypoints = NumberofWaypoints;
                }

                cout << "Running test: " << testName << endl;
                cout << "Total number of waypoints: " << NumberofWaypoints << endl;
                
                inputFile.close();

                readyForNextWp = true;
            }
            else
                cout << "Couldn't open: " << inputFilePath << endl;
        }
    }
}

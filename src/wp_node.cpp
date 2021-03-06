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
#include <string>
#include <fstream>
#include <pwd.h>
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
using namespace std;

bitten::control_msg wp_msg;
bitten::feedback_msg fb_msg;

int NumberofWaypoints;                                              /* Number of waypoints excluding waypoint_0.                                        */
int RemainingWaypoints;                                             /* Number of remaining waypoints to perform.                                        */

int TestLoopCounter = 0;

bool readyForNextWp     = false;                                    /* Bool to determine if the node is ready to Tx a new WP                            */
bool transmitWpReady    = false;                                    /* Bool to determine if the waypoint_node is ready to transmit on waypoint_topic    */

Waypoint_s WaypointBank[256];                                        /* Create an empty bank of waypoints, to contain future tests                       */

passwd* pw = getpwuid(getuid());                                    /* Get a password reference, to help fetch the home path of the current PC          */
string path(pw->pw_dir);                                            /* Get the system home filepath                                                     */
string testsPath        = path + "/catkin_ws/src/bitten/tests/";    /* Save the filepath to the tests folder, containing the files for various tests    */

 /* ----------------------------------------------------------------------
 *                          -------  Main   -------
 * ----------------------------------------------------------------------- */        
int main(int argc, char **argv)
{
    ros::init(argc,argv,nodeNames[WP_NODE]);
    ros::NodeHandle n;

    wp_msg.nodeName = nodeNames[WP_NODE].c_str();
    wp_msg.id       = WP_ID;

    ros::Publisher wpPub        = n.advertise<bitten::control_msg>  (topicNames[WP_TOPIC],5);
    ros::Subscriber feedbackSub = n.subscribe<bitten::feedback_msg> (topicNames[FEEDBACK_TOPIC], 5, &fbCallback);

    ros::Rate loop_rate(LOOP_RATE_INT);

    sleep(1);
    
    if (wpPub && feedbackSub)
        ROS_INFO("Initiated %s", nodeNames[WP_NODE].c_str());
    else
        ROS_INFO("Failed to initiate %s",nodeNames[WP_NODE].c_str());

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
            wp_msg.flags = 0;
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
                readyForNextWp = true;
                inputFile.close();
            }
            
            else
                cout << "Couldn't open: " << inputFilePath << endl;
        }

        if (feedback->flags & START_TEST_LOOP)
        {
            TestLoopCounter = feedback->val;
            int waypointCounter = 1;
            string testName;
            for (int n = 0; n < TestLoopCounter; n++)
            {
                string inputFilePath = testsPath + feedback->programName;
                ifstream inputFile(inputFilePath);

                if (inputFile.is_open())
                {
                    
                    inputFile >> testName;

                    for (int i = 0; inputFile >> WaypointBank[waypointCounter-1].waypointName; i++, waypointCounter++)
                    {
                        for (int n = 0; n < 6; n++)
                            inputFile >> WaypointBank[waypointCounter-1].jointPosition[n];

                        NumberofWaypoints = waypointCounter;
                        RemainingWaypoints = NumberofWaypoints;
                    }
                    
                    readyForNextWp = true;
                    inputFile.close();
                }
                else
                    cout << "Couldn't open: " << inputFilePath << endl;
            }   
            cout << "Running test: " << testName << endl;
            cout << "Total number of waypoints: " << NumberofWaypoints << endl;
        }
    }
}
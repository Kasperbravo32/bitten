/* -----------------------------------------------------------------------
 * Filename: dataCollectionModule.cpp
 * Author: Frederik Snedevind & Kasper Banke Jørgensen
 * Purpose: Create the 'DataCollector' node in the Leica Robot system
 * Date of dev.: September 2018 - January 2019
 * 
 * -----------------------------------------------------------------------
 *                      -------  Libraries   -------
 * ----------------------------------------------------------------------- */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <global_node_definitions.h>
#include "control_msgs/FollowJointTrajectoryFeedback.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "bitten/feedback_msg.h"
#include "bitten/control_msg.h"
#include "movement_node.h"
#include <cmath>
#include <iostream>
#include <fstream>

#include <pwd.h>

using namespace std;
/* -----------------------------------------------------------------------
 *                     -------  Initializers  -------
 * ----------------------------------------------------------------------- */
void feedbackCallback  (const control_msgs::FollowJointTrajectoryFeedback::ConstPtr& feedbackMsg);
void jointPathCallback (const bitten::control_msg::ConstPtr& jointPathCallback);

/* ----------------------------------------------------------------------
 *                    -------  Global Variables   -------
 * ----------------------------------------------------------------------- */
passwd* pw = getpwuid(getuid());
string path(pw->pw_dir);
string dataPath = path + "/catkin_ws/src/bitten/data_analysis/logfile.txt";
string currentRecordFile;

double goalPositions[6];
double currPositions[6];

bool goalReady  = false;
bool justOpened = true;
bool waypoints_started = false;

int lineNumber = 0;
/* ----------------------------------------------------------------------
 *                          -------  Main    -------
 * ----------------------------------------------------------------------- */
int main (int argc , char **argv) 
{    
    ros::init(argc , argv , "dataCollectionModule");
    ros::NodeHandle n;

    ros::Subscriber DataJointPathCommSub = n.subscribe<bitten::control_msg>                         ("movement_topic", 2 , jointPathCallback);
    ros::Subscriber DataFeedbackSub      = n.subscribe<control_msgs::FollowJointTrajectoryFeedback> ("feedback_states"   , 2 , feedbackCallback);

    ros::Rate loop_rate(LOOP_RATE_INT);

    sleep(1);

    if (DataJointPathCommSub && DataFeedbackSub)
        ROS_INFO("Initiated %s",nodeNames[DATACOLLECTOR_NODE].c_str());
    else
        ROS_INFO("Failed to to initiate %s",nodeNames[DATACOLLECTOR_NODE].c_str());

    std::ofstream logfile(dataPath , std::ios_base::out | std::ios_base::trunc);
/*  -------------------------------------------------
         SUPERLOOP
    ------------------------------------------------- */
    while(ros::ok())
    {
        if (logfile.is_open())
        {
            if (justOpened == true)
            {
                justOpened = false;
            }
            
            if (waypoints_started == true)
            {
                if (goalReady == true)
                {
                    
                    logfile << "GOAL";
                    for (int i = 0; i < 6; i++)
                        logfile << " " << goalPositions[i];

                        
                    logfile << "\n";
                    lineNumber++;
                    cout << "Current line number: " << lineNumber << endl;
                    goalReady = false;
                }

                logfile << "ACTUAL";
                for (int i = 0; i < 6; i++)
                    logfile << " " << currPositions[i];
                logfile << "\n";
                lineNumber++;
            }
        }

        else if (goalReady == true)
        {
            goalReady = false;

            if (logfile.is_open())
            {
                logfile << "GOAL";
                for (int i = 0; i < 6; i++)
                    logfile << " " << goalPositions[i];
                logfile << "\n";
            }
        }
            
        ros::spinOnce();
        loop_rate.sleep();
    }
    logfile.close();
    return 0;
}



/* ----------------------------------------------------------------------
 *                    -------  feedbackCallback   -------
 * ----------------------------------------------------------------------- */
void feedbackCallback  (const control_msgs::FollowJointTrajectoryFeedback::ConstPtr& feedbackMsg)
{
    for (int i = 0; i < 6; i++)
        currPositions[i] = feedbackMsg->actual.positions[i];
}

/* ----------------------------------------------------------------------
 *                -------  Joint Path Command Callback   -------
 * ----------------------------------------------------------------------- */
void jointPathCallback (const bitten::control_msg::ConstPtr& jointPathCallback)
{
    if (jointPathCallback->nodeName == nodeNames[WP_NODE])
    {
        if (waypoints_started == false)
            waypoints_started = true;
        goalReady = true;
        for (int i = 0; i < 6; i++) 
            goalPositions[i] = jointPathCallback->jointPosition[i];
    }
}
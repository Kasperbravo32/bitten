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
void jointPathCallback (const trajectory_msgs::JointTrajectory::ConstPtr& jointPathCallback);

/* ----------------------------------------------------------------------
 *                    -------  Global Variables   -------
 * ----------------------------------------------------------------------- */
passwd* pw = getpwuid(getuid());
string path(pw->pw_dir);
string dataPath = path + "/catkin_ws/src/bitten/data_analysis/logfile1.txt";
string currentRecordFile;

double goalPositions[6];
double currPositions[6];

bool goalReady  = false;
bool justOpened = true;
bool waypoints_started = false;

int lineNumber = 0;

trajectory_msgs::JointTrajectory        jointPathMsg;               /* Message used to transmit wanted position to motion_streaming_interface           */
trajectory_msgs::JointTrajectoryPoint   jointPathPointMsg;      

/* ----------------------------------------------------------------------
 *                          -------  Main    -------
 * ----------------------------------------------------------------------- */
int main (int argc , char **argv) 
{    
    ros::init(argc , argv , "dataCollectionModule");
    ros::NodeHandle n;

    ros::Subscriber DataJointPathCommSub = n.subscribe<trajectory_msgs::JointTrajectory>            ("joint_path_command", 2 , jointPathCallback);
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
                    cout << "Current line number : " << lineNumber << " [lines]" << endl;
                    cout << "Current runtime     : " << lineNumber/50.0 << " [s]" << endl;
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
void jointPathCallback (const trajectory_msgs::JointTrajectory::ConstPtr& jointPathCallback)
{

    if (waypoints_started == false)
        waypoints_started = true;
    goalReady = true;
    for (int i = 0; i < 6; i++)
    {
        jointPathPointMsg.positions.push_back(jointPathCallback->points)
        goalPositions[i] = jointPathCallback->positions[i];
    }

    for (int i = 0; i < 6; i++)
            {
                if ((commander->jointVelocity[i] > 0.2) && 
                    (((i == 0 || i == 2 || i == 3) && commander->buttons[8] == 1) || 
                    ((i == 1 || i == 4 || i == 5) &&  commander->buttons[2] == 1)))
                {
                    if(justMovedArr[i] == false)
                    {
                        TX90.setGoalPos(i, TX90.getMaxRotation(i));
                        justMovedArr[i] = true;
                        jointTransmitReady = true;
                    }
                }
                
                else if ((commander->jointVelocity[i] < -0.2) && 
                        (((i == 0 || i == 2 || i == 3) && commander->buttons[8] == 1) || 
                        ((i == 1 || i == 4 || i == 5) && commander->buttons[2] == 1)))
                {   
                    if (justMovedArr[i] == false)
                    {
                        TX90.setGoalPos(i, TX90.getMinRotation(i));
                        justMovedArr[i] = true;
                        jointTransmitReady = true;
                    }
                }
                else
                {
                    if (justMovedArr[i] == true)
                    {
                        TX90.setGoalPos(i, TX90.getCurrPos(i));
                        TX90.setCurrVelocity(1);

                        stopMotion = true;
                        justMovedArr[i] = false;
                        jointTransmitReady = true;
                    }
                    else
                    {

                    }
                }
            }
}
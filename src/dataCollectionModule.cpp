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
string dataPath = path + "/catkin_ws/src/bitten/data_analysis/STUPIDF1LE.txt";
string currentRecordFile;

double goalPositions[6];
double currPositions[6];

bool goalReady = false;

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

    std::ofstream outputFile(dataPath , std::ios_base::out);
/*  -------------------------------------------------
         SUPERLOOP
    ------------------------------------------------- */
    while(ros::ok())
    {
        if (outputFile.is_open())
        {
            if (goalReady == true)
            {
                outputFile << "GOAL";
                for (int i = 0; i < 6; i++)
                    outputFile << " " << goalPositions[i];
                outputFile << "\n";

            }

            outputFile << "ACTUAL";
            for (int i = 0; i < 6; i++)
                outputFile << " " << currPositions[i];
            outputFile << "\n";

        }
        else if (goalReady == true)
        {
            goalReady = false;

            if (outputFile.is_open())
            {
                cout << "Data Collector opened loggingfile" << endl;

                outputFile << "GOAL";
                for (int i = 0; i < 6; i++)
                    outputFile << " " << goalPositions[i];
                outputFile << "\n";
            }
            else
                cout << "Couldn't open data logging file." << endl;
        }
            
        ros::spinOnce();
        loop_rate.sleep();
    }
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
    goalReady = true;

    for (int i = 0; i < 6; i++)
        goalPositions[i] = (jointPathCallback->points[0].positions[i]);
}
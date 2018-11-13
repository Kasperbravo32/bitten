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
#include "bitten/feedback_msg.h"
#include "bitten/control_msg.h"
#include "movement_node.h"
#include <cmath>
#include <iostream>

using namespace std;
/* -----------------------------------------------------------------------
 *                     -------  Initializers  -------
 * ----------------------------------------------------------------------- */
// void robotStateCallback (const control_msgs::FollowJointTrajectoryFeedback::ConstPtr&   RobotState);
// void commanderCallback  (const bitten::control_msg::ConstPtr&                           commander);

void feedbackCallback  (const control_msgs::FollowJointTrajectoryFeedback::ConstPtr& feedbackMsg);
void jointPathCallback (const trajectory_msgs::JointTrajectory::ConstPtr& jointPathCallback);
 /* ----------------------------------------------------------------------
 *                    -------  Message objects   -------
 * ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------
 *                    -------  Global Variables   -------
 * ----------------------------------------------------------------------- */
iostream outputFile;

passwd* pw = getpwuid(getuid());
std::string path(pw->pw_dir);
std::string testsPath        = path + "/catkin_ws/src/bitten/tests/";
std::string currentRecordFile;


/* ----------------------------------------------------------------------
 *                          -------  Main    -------
 * ----------------------------------------------------------------------- */
int main (int argc , char **argv) 
{    
    ros::init(argc , argv , "DataCollector");
    ros::NodeHandle n;

    ros::Subscriber DataJointPathCommSub = n.subscribe<trajectory_msgs::JointTrajectory>    ("joint_path_command", 2 , );
    ros::Subscriber DataFeedbackSub      = n.subscribe<control_msgs::FollowJointTrajectoryFeedback> ("feedback_states"   , 2 , robotStateCallback);

    ros::Rate loop_rate(LOOP_RATE_INT);
    sleep(1);

    if (DataJointPathCommSub && DataFeedbackSub)
        ROS_INFO("Initiated %s",nodeNames[DATACOLLECTOR_NODE].c_str());
    else
        ROS_INFO("Failed to to initiate %s",nodeNames[DATACOLLECTOR_NODE].c_str());

/*  -------------------------------------------------
         SUPERLOOP
    ------------------------------------------------- */
    while(ros::ok())
    {
        if (outputputFile.is_open())
        {
            outputFile
        }
        else
            outputFile.open("/home/")
        
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

}

/* ----------------------------------------------------------------------
 *                -------  Joint Path Command Callback   -------
 * ----------------------------------------------------------------------- */
void jointPathCallback (const trajectory_msgs::JointTrajectory::ConstPtr& jointPathCallback)
{

}
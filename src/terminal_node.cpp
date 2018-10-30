/* -----------------------------------------------------------------------
 * Filename: movement_node.cpp
 * Author: Frederik Snedevind
 * Purpose: Create the 'movement' node in the Leica Robot system
 * Date of dev.: September 2018 - January 2019
 * 
 * -----------------------------------------------------------------------
 *                      -------  Libraries   -------
 * ----------------------------------------------------------------------- */
#include "ros/ros.h"
#include "std_msgs/String.h"
// #include <sensor_msgs/Joy.h>
#include <bitten/control_msg.h>
#include <bitten/feedback_msg.h>
#include <global_node_definitions.h>

#include "control_msgs/FollowJointTrajectoryFeedback.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "bitten/feedback_msg.h"
#include "bitten/control_msg.h"
#include "movement_node.h"

/* -----------------------------------------------------------------------
 *                     -------  Initializers  -------
 * ----------------------------------------------------------------------- */
void robotStateCallback (const control_msgs::FollowJointTrajectoryFeedback::ConstPtr&   RobotState);
void commanderCallback  (const bitten::control_msg::ConstPtr&                           commander);
void InitRobot();
 /* ----------------------------------------------------------------------
 *                    -------  Message objects   -------
 * ----------------------------------------------------------------------- */
trajectory_msgs::JointTrajectory        jointPathMsg;               /* Message used to transmit wanted position to motion_streaming_interface           */
trajectory_msgs::JointTrajectoryPoint   jointPathPointMsg;          /* Message used to contain specific points of wanted position, part of jointPathMsg */

// trajectory_msgs::JointTrajectory        empty_traj_msg;
// trajectory_msgs::JointTrajectoryPoint   empty_traj_msg_point;

bitten::feedback_msg                    movementFeedbackMsg;        /* Message used to report back when wanted position is reached, e.g. waypoint-mode  */

/* ----------------------------------------------------------------------
 *                    -------  Global Variables   -------
 * ----------------------------------------------------------------------- */
bool robotInitialized       = false;
bool goalExists             = false;
bool jointTransmitReady     = true;
bool feedbackTransmitReady  = false;
bool DirtyBit               = false;

int OPERATING_MODE = 0;
int ManPubTimer = LOOP_RATE_INT / 5;


int temp_looper = 2;
int TEMP_LOOPER = 3;

double TIME_FROM_START_TIMER = 0;
 /* ----------------------------------------------------------------------
 *                         -------  Main   -------
 * ----------------------------------------------------------------------- */        
int main (int argc , char **argv) 
{
    ROS_INFO("Initiating %s", nodeNames[TERMINAL_NODE].c_str());

    ros::init(argc , argv , "terminal_node");
    ros::NodeHandle n;

    ros::Subscriber movement_sub = n.subscribe<bitten::control_msg>                         ("movement_topic"       , 2 , commanderCallback);
    ros::Subscriber feedback_sub = n.subscribe<control_msgs::FollowJointTrajectoryFeedback> ("feedback_states"      , 2 , robotStateCallback);
    
    ros::Publisher  movement_pub = n.advertise<trajectory_msgs::JointTrajectory>            ("joint_path_command"   , 2);
    ros::Publisher  feedback_pub = n.advertise<bitten::feedback_msg>                        ("movement_feedback"    , LOOP_RATE_INT);

    ros::Rate loop_rate(LOOP_RATE_INT);
    sleep(1);

    if (movement_sub && feedback_sub && movement_pub && feedback_pub)
        ROS_INFO("Initiated %s",nodeNames[MOVEMENT_NODE].c_str());
    else
        ROS_INFO("Failed to to initiate %s",nodeNames[MOVEMENT_NODE].c_str());

/*  -------------------------------------------------
         SUPERLOOP
    ------------------------------------------------- */
    while(ros::ok())
    {
    }
}
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
#include <sensor_msgs/Joy.h>
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

 /* ----------------------------------------------------------------------
 *                    -------  Message objects   -------
 * ----------------------------------------------------------------------- */
trajectory_msgs::JointTrajectory        jointPathMsg;               /* Message used to transmit wanted position to motion_streaming_interface           */
trajectory_msgs::JointTrajectoryPoint   jointPathPointMsg;          /* Message used to contain specific points of wanted position, part of jointPathMsg */
bitten::feedback_msg                    movementFeedbackMsg;        /* Message used to report back when wanted position is reached, e.g. waypoint-mode  */

/* ----------------------------------------------------------------------
 *                    -------  Global Variables   -------
 * ----------------------------------------------------------------------- */
bool robotInitialized = false;
void InitRobot();

bool goalExists = false;
bool goalReached = false;

double goalPosition[6];
 /* ----------------------------------------------------------------------
 *                         -------  Main   -------
 * ----------------------------------------------------------------------- */        
int main (int argc , char **argv) 
{
    ROS_INFO("Initiating system...");
    
    movementFeedbackMsg.senderID = MOVEMENT_ID;
    InitRobot();

    ros::init(argc , argv , "movement_node");
    ros::NodeHandle n;

    ros::Subscriber movement_sub = n.subscribe<bitten::control_msg>("movement_topic" , 3*LOOP_RATE_INT , commanderCallback);
    ros::Publisher movement_pub = n.advertise<trajectory_msgs::JointTrajectory> ("joint_path_command" , 3*LOOP_RATE_INT);

    

    ros::Rate loop_rate(LOOP_RATE_INT);
    sleep(1);

    if (movement_sub)
    {
        if (movement_pub)
            ROS_INFO("Initiated %s",nodeNames[MOVEMENT_NODE].c_str());
        else
            ROS_INFO("Failed to initialize movement_node publisher");
    }
    else
        ROS_INFO("Failed to initialize movement_node subscriber");

    

/*  -------------------------------------------------
         SUPERLOOP
    ------------------------------------------------- */
    while(ros::ok())
    {
        if (robotInitialized == true)
        {


        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

 /* ----------------------------------------------------------------------
 *                 -------  Initialize TX90 Robot   -------
 * ----------------------------------------------------------------------- */       
void InitRobot()
{
    TX90.minLink    = 1;
    TX90.maxLink    = 6;
    TX90.resetStatePosition = {0 , 0 , 0 , 0 , 0 , 0};

    TX90.currVelocity   = 0.05;
    
    TX90.maxRotation = {    3.14,               /* joint_1  */
                            2.57,               /* joint_2  */
                            2.53,               /* joint_3  */
                            4.71,               /* joint_4  */
                            2.44,               /* joint_5  */
                            4.71};              /* joint_6  */

    TX90.minRotation = {    -3.14,              /* joint_1  */
                            -2.27,              /* joint_2  */
                            -2.53,              /* joint_3  */
                            -4.71,              /* joint_4  */
                            -2.01,              /* joint_5  */
                            -4.71};             /* joint_6  */ 

    TX90.maxVelocity = {    (400/180)*3.14,     /* joint_1  */
                            (400/180)*3.14,     /* joint_2  */
                            (430/180)*3.14,     /* joint_3  */
                            (540/180)*3.14,     /* joint_4  */
                            (475/180)*3.14,     /* joint_5  */
                            (760/180)*3.14};    /* joint_6  */                   

    TX90.maxEffort  =   {   318,                /* joint_1  */
                            166,                /* joint_2  */
                            76,                 /* joint_3  */
                            34,                 /* joint_4  */
                            29,                 /* joint_5  */
                            11};                /* joint_6  */
                            
    TX90.currPos    = { 0 , 0 , 0 , 0 , 0 , 0};

    TX90.jointNames = { "joint_1", 
                        "joint_2", 
                        "joint_3", 
                        "joint_4", 
                        "joint_5", 
                        "joint_6"};

    TX90.tool = false;
}
/* ----------------------------------------------------------------------
 *             -------  commander_node Callback function   -------
 * ----------------------------------------------------------------------- */       
void commanderCallback  (const bitten::control_msg::ConstPtr&                           commander)
{
    if (goalExists == false)
    {
        switch(commander->id)
        {
            case MANUAL_ID:




            break;

            case WP_ID:
                // for (int i = 0; i < 6; i++)
                //     goalPosition[i] = <double>commander->jointPosition;

            break;

            default:
            break;

        }




        
        goalExists = true;
        goalReached = false;
    }
    else if (goalExists == true && goalReached == true)
    {

    }
    else
    {

    }


}
/* ----------------------------------------------------------------------
 *              -------  robot_state Callback function   -------
 * ----------------------------------------------------------------------- */       
void robotStateCallback (const control_msgs::FollowJointTrajectoryFeedback::ConstPtr&   RobotState)
{
    if (robotInitialized == false)
    {
        for (int i = 0; i < 6; i++)
            TX90.currPos[i] = RobotState->actual.positions[i];
        
        robotInitialized = true;
    }
    else
    {
        for (int i = 0; i < 6; i++)
            TX90.currPos[i] = RobotState->actual.positions[i];

    }

}

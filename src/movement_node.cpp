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
void InitRobot();
 /* ----------------------------------------------------------------------
 *                    -------  Message objects   -------
 * ----------------------------------------------------------------------- */
trajectory_msgs::JointTrajectory        jointPathMsg;               /* Message used to transmit wanted position to motion_streaming_interface           */
trajectory_msgs::JointTrajectoryPoint   jointPathPointMsg;          /* Message used to contain specific points of wanted position, part of jointPathMsg */
bitten::feedback_msg                    movementFeedbackMsg;        /* Message used to report back when wanted position is reached, e.g. waypoint-mode  */

/* ----------------------------------------------------------------------
 *                    -------  Global Variables   -------
 * ----------------------------------------------------------------------- */
bool robotInitialized       = false;
bool goalExists             = false;
bool goalReached            = false;
bool jointTransmitReady     = false;
 /* ----------------------------------------------------------------------
 *                         -------  Main   -------
 * ----------------------------------------------------------------------- */        
int main (int argc , char **argv) 
{
    ROS_INFO("Initiating %s", nodeNames[MOVEMENT_NODE].c_str());
    
    movementFeedbackMsg.senderID = MOVEMENT_ID;
    InitRobot();

    ros::init(argc , argv , "movement_node");
    ros::NodeHandle n;

    ros::Subscriber movement_sub = n.subscribe<bitten::control_msg>                         ("movement_topic" , 3*LOOP_RATE_INT , commanderCallback);
    ros::Subscriber feedback_sub = n.subscribe<control_msgs::FollowJointTrajectoryFeedback> ("feedback_states" , 3*LOOP_RATE_INT, robotStateCallback);
    ros::Publisher  movement_pub = n.advertise<trajectory_msgs::JointTrajectory>            ("joint_path_command" , 3*LOOP_RATE_INT);
    ros::Publisher  feedback_pub = n.advertise<bitten::feedback_msg>                        ("movement_feedback" , 3*LOOP_RATE_INT);


    jointPathPointMsg.positions.resize(6);
    jointPathPointMsg.velocities.resize(6);
    jointPathPointMsg.accelerations.resize(6);
    jointPathPointMsg.effort.resize(1);

    ros::Rate loop_rate(LOOP_RATE_INT);
    sleep(1);

    if (movement_sub)
    {
        if (movement_pub)
        {
            if (feedback_sub)
                ROS_INFO("Initiated %s",nodeNames[MOVEMENT_NODE].c_str());
            else
                ROS_INFO("Failed to subscribe to \"feedback_states\"");
        }
        else
            ROS_INFO("Failed to publish to \"joint_path_command\"");
    }
    else
        ROS_INFO("Failed to subscribe to \"movement_topic\"");


    

/*  -------------------------------------------------
         SUPERLOOP
    ------------------------------------------------- */
    while(ros::ok())
    {
        if (robotInitialized == true)
        {
            if (goalExists == true && goalReached == false)
            {
                for (int i = 0; i < 6; i++)
                {

                    if (TX90.currPos[i] >= TX90.goalPosition[i]*0.999 && TX90.currPos[i] <= TX90.goalPosition[i] * 1.001)
                        TX90.jointsAtGoal |= (1 < i);

                    else
                    {
                        jointPathMsg.joint_names.push_back(TX90.jointNames[i]);
                        jointPathPointMsg.positions.push_back(TX90.goalPosition[i]);
                        jointPathPointMsg.velocities.push_back(TX90.currVelocity*TX90.maxVelocity[i]);
                    }

                    if (TX90.jointsAtGoal & 0b111111)
                    {

                        feedback_msg.flags |= GOAL_REACHED;
                        goalReached = true;
                        goalExists  = false;

                    }
                }
                jointPathMsg.points.push_back(jointPathPointMsg);
            }
        }

        if (jointTransmitReady)
        {
            movement_pub.publish(jointPathMsg);
            jointTransmitReady = false;
        }

        if (feedbackTransmitReady)
        {
            feedback_pub.publish(feedback_msg);
            
            feedback_msg.flags = 0;
            feedbackTransmitReady = false;
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
                for (int i = 0; i < 6; i++)
                {
                    if (commander->jointVelocity[i] > 0)        /* If goalposition is in positive rotational direction */
                    {
                        if (TX90.currPos[i] +  (1/LOOP_RATE_INT * commander->jointVelocity[i] * TX90.maxVelocity[i] * TX90.currVelocity) < TX90.maxRotation[i])
                            TX90.goalPosition[i] += (1/LOOP_RATE_INT * commander->jointVelocity[i] * TX90.maxVelocity[i] * TX90.currVelocity);
                    }
                    
                    else if (commander->jointVelocity[i] < 0)   /* if goalposition is in negative rotational direction */
                    {
                        if (TX90.currPos[i] -  (1/LOOP_RATE_INT * commander->jointVelocity[i] * TX90.maxVelocity[i] * TX90.currVelocity) > TX90.minRotation[i])
                            TX90.goalPosition[i] += (1/LOOP_RATE_INT * commander->jointVelocity[i] * TX90.maxVelocity[i] * TX90.currVelocity);
                    }   
                }

                TX90.jointsAtGoal = 0;
                goalExists = true;
                goalReached = false;
            break;

            case WP_ID:
                for (int i = 0; i < 6; i++)
                    TX90.goalPosition[i] = commander->jointPosition[i];

                TX90.jointsAtGoal = 0;
                goalExists = true;
                goalReached = false;
            break;

            default:
            break;
        }
    }

    else if (goalExists == true && goalReached == true)
    {
        switch(commander->id)
        {
            case MANUAL_ID:
                for (int i = 0; i < 6; i++)
                {
                    if (commander->jointVelocity[i] > 0)        /* If goalposition is in positive rotational direction */
                    {
                        if (TX90.currPos[i] +  (1/LOOP_RATE_INT * commander->jointVelocity[i] * TX90.maxVelocity[i] * TX90.currVelocity) < TX90.maxRotation[i])
                            TX90.goalPosition[i] += (1/LOOP_RATE_INT * commander->jointVelocity[i] * TX90.maxVelocity[i] * TX90.currVelocity);
                    }
                    
                    else if (commander->jointVelocity[i] < 0)   /* if goalposition is in negative rotational direction */
                    {
                        if (TX90.currPos[i] -  (1/LOOP_RATE_INT * commander->jointVelocity[i] * TX90.maxVelocity[i] * TX90.currVelocity) > TX90.minRotation[i])
                            TX90.goalPosition[i] += (1/LOOP_RATE_INT * commander->jointVelocity[i] * TX90.maxVelocity[i] * TX90.currVelocity);
                    }   
                }
                goalExists = true;
                goalReached = false;
            break;

            case WP_ID:
                for (int i = 0; i < 6; i++)
                    TX90.goalPosition[i] = commander->jointPosition[i];
                goalExists = true;
                goalReached = false;
            break;

            default:
                /* Didn't recognize the ID  */
            break;
        }
    }

    else
    {
        /* Received data, but is currently occupied */
    }

    if (commander->buttons[0] == 1 && TX90.currVelocity >=0.05)     /* Check for Reduce speed input     */
        TX90.currVelocity -= 0.05;

    if (commander->buttons[1] == 1 && TX90.currVelocity <= 0.95)    /* Check for increase speed input   */
        TX90.currVelocity += 0.05;
}
/* ----------------------------------------------------------------------
 *              -------  robot_state Callback function   -------
 * ----------------------------------------------------------------------- */       
void robotStateCallback (const control_msgs::FollowJointTrajectoryFeedback::ConstPtr&   RobotState)
{
    /* Update currPos from the feedback, measuring on the sensors   */
    for (int i = 0; i < 6; i++)
        TX90.currPos[i] = RobotState->actual.positions[i];
    
    /* Set an 'initialized' flag    */
    if (!robotInitialized)
        robotInitialized = true;
}

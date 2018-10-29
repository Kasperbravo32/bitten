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
bitten::feedback_msg                    movementFeedbackMsg;        /* Message used to report back when wanted position is reached, e.g. waypoint-mode  */

/* ----------------------------------------------------------------------
 *                    -------  Global Variables   -------
 * ----------------------------------------------------------------------- */
bool robotInitialized       = false;
bool goalExists             = false;
bool jointTransmitReady     = true;
bool feedbackTransmitReady  = false;

int OPERATING_MODE = 0;
int ManPubTimer = LOOP_RATE_INT / 5;
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

    ros::Subscriber movement_sub = n.subscribe<bitten::control_msg>                         ("movement_topic"       , 3 , commanderCallback);
    ros::Subscriber feedback_sub = n.subscribe<control_msgs::FollowJointTrajectoryFeedback> ("feedback_states"      , 5 , robotStateCallback);
    
    ros::Publisher  movement_pub = n.advertise<trajectory_msgs::JointTrajectory>            ("joint_path_command"   , LOOP_RATE_INT);
    ros::Publisher  feedback_pub = n.advertise<bitten::feedback_msg>                        ("movement_feedback"    , LOOP_RATE_INT);


    jointPathPointMsg.positions.resize(6);
    jointPathPointMsg.velocities.resize(6);
    jointPathPointMsg.accelerations.resize(6);
    jointPathPointMsg.effort.resize(1);

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
        switch(OPERATING_MODE)
        {
            case WP_ID:
                if (goalExists == true)
                {
                    // ROS_INFO("Goal true in superloop");
                    jointPathMsg.joint_names.clear();
                    jointPathMsg.points.clear();
                    jointPathPointMsg.positions.clear();
                    jointPathPointMsg.velocities.clear();

                    for (int i = 0; i < 6; i++)
                    {
                        jointPathMsg.joint_names.push_back(TX90.jointNames[i]);
                        jointPathPointMsg.positions.push_back(TX90.goalPosition[i]);
                        jointPathPointMsg.velocities.push_back(TX90.currVelocity*TX90.maxVelocity[i]);
                    }

                    jointPathMsg.points.push_back(jointPathPointMsg);
                    // movement_pub.publish(jointPathMsg);

                }   

                if (jointTransmitReady)
                {
                    ROS_INFO("Publishing to bot");
                    movement_pub.publish(jointPathMsg);
                    jointTransmitReady = false;
                }
            

                if (feedbackTransmitReady)
                {
                    ROS_INFO("Publishing to feedback");;
                    feedback_pub.publish(movementFeedbackMsg);
                    
                    movementFeedbackMsg.flags = 0;
                    feedbackTransmitReady = false;
                }
                break;

            case MANUAL_ID:

                // ROS_INFO("Goal true in superloop");
                jointPathMsg.joint_names.clear();
                jointPathMsg.points.clear();
                jointPathPointMsg.positions.clear();
                // jointPathPointMsg.velocities.clear();

                for (int i = 0; i < 6; i++)
                {
                    jointPathMsg.joint_names.push_back(TX90.jointNames[i]);
                    jointPathPointMsg.positions.push_back(TX90.goalPosition[i]);
                    // jointPathPointMsg.velocities.push_back(TX90.currVelocity*TX90.maxVelocity[i]);
                }

                jointPathMsg.points.push_back(jointPathPointMsg);
                jointTransmitReady = true;

                // if (! --ManPubTimer)
                // {
                //     // ROS_INFO("Publishing to bot");
                //     movement_pub.publish(jointPathMsg);
                //     ManPubTimer = LOOP_RATE_INT / 6;
                // }

                if (jointTransmitReady)
                {
                    
                    movement_pub.publish(jointPathMsg);
                    jointTransmitReady = false;
                }
        

                if (feedbackTransmitReady)
                {
                    ROS_INFO("Publishing to feedback");;
                    feedback_pub.publish(movementFeedbackMsg);
                    
                    movementFeedbackMsg.flags = 0;
                    feedbackTransmitReady = false;
                }
            break;
        }
        
        for (int i = 0; i < 3; i++)
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

    TX90.currVelocity   = 1;

    TX90.maxRotation = {    3.14,                   /* joint_1  */
                            2.57,                   /* joint_2  */
                            2.53,                   /* joint_3  */
                            4.71,                   /* joint_4  */
                            2.44,                   /* joint_5  */
                            4.71};                  /* joint_6  */

    TX90.minRotation = {    -3.14,                  /* joint_1  */
                            -2.27,                  /* joint_2  */
                            -2.53,                  /* joint_3  */
                            -4.71,                  /* joint_4  */
                            -2.01,                  /* joint_5  */
                            -4.71};                 /* joint_6  */ 

    TX90.maxVelocity = {    (400.0/180.0)*3.14,     /* joint_1  */
                            (400.0/180.0)*3.14,     /* joint_2  */
                            (430.0/180.0)*3.14,     /* joint_3  */
                            (540.0/180.0)*3.14,     /* joint_4  */
                            (475.0/180.0)*3.14,     /* joint_5  */
                            (760.0/180.0)*3.14};    /* joint_6  */                   

    TX90.maxEffort   =  {   318,                    /* joint_1  */
                            166,                    /* joint_2  */
                            76,                     /* joint_3  */
                            34,                     /* joint_4  */
                            29,                     /* joint_5  */
                            11};                    /* joint_6  */
                            
    TX90.currPos     =  {   0.0 , 
                            0.0 , 
                            0.0 , 
                            0.0 , 
                            0.0 , 
                            0.0 };

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
                if (OPERATING_MODE != MANUAL_ID)
                {
                    OPERATING_MODE = MANUAL_ID;
                    ROS_INFO("Set OpMode to ManMode");
                }

                for (int i = 0; i < 6; i++)
                {
                    if (commander->jointVelocity[i] == 0)
                    {
                        TX90.goalPosition[i] = TX90.currPos[i];
                    }
                    else if (commander->jointVelocity[i] > 0)
                    {
                        if (TX90.currPos[i] +  (2/LOOP_RATE_INT * commander->jointVelocity[i] * TX90.maxVelocity[i] * TX90.currVelocity) < TX90.maxRotation[i])
                            TX90.goalPosition[i] = TX90.currPos[i] + (2/LOOP_RATE_INT * commander->jointVelocity[i] * TX90.maxVelocity[i] * TX90.currVelocity);
                    }
                    else if (commander->jointVelocity[i] < 0)
                    {
                        if (TX90.currPos[i] +  (2/LOOP_RATE_INT * commander->jointVelocity[i] * TX90.maxVelocity[i] * TX90.currVelocity) > TX90.minRotation[i])
                            TX90.goalPosition[i] = TX90.currPos[i] + (2/LOOP_RATE_INT * commander->jointVelocity[i] * TX90.maxVelocity[i] * TX90.currVelocity);
                    }





                    // if (commander->jointVelocity[i] == 0)
                    // {

                    // }

                    // else if (commander->jointVelocity[i] > 0)        /* If goalposition is in positive rotational direction */
                    // {
                    //     if (TX90.currPos[i] +  (1/LOOP_RATE_INT * commander->jointVelocity[i] * TX90.maxVelocity[i] * TX90.currVelocity) < TX90.maxRotation[i])
                    //     {
                    //         TX90.goalPosition[i] += (1/LOOP_RATE_INT * commander->jointVelocity[i] * TX90.maxVelocity[i] * TX90.currVelocity);
                    //     }
                    // }
                    
                    // else if (commander->jointVelocity[i] < 0)   /* if goalposition is in negative rotational direction */
                    // {
                    //     if (TX90.currPos[i] -  (1/LOOP_RATE_INT * commander->jointVelocity[i] * TX90.maxVelocity[i] * TX90.currVelocity) > TX90.minRotation[i])
                    //     {
                    //         TX90.goalPosition[i] += (1/LOOP_RATE_INT * commander->jointVelocity[i] * TX90.maxVelocity[i] * TX90.currVelocity);
                    //     }
                    // }   
                }

            break;

            case WP_ID:
                if (OPERATING_MODE != WP_ID)
                {
                    OPERATING_MODE = WP_ID;
                    ROS_INFO("Set OpMode to WPMode");
                }

                for (int i = 0; i < 6; i++)
                    TX90.goalPosition[i] = commander->jointPosition[i];
                    
                goalExists = true;
                jointTransmitReady = true;
            break;

            default:
                ROS_INFO("Entered Missing ID callback");
            break;
        }
    }
    else
    {
        /* Received data, but is currently occupied */
    }

    // if (commander->buttons[0] == 1 && TX90.currVelocity >=0.05)     /* Check for Reduce speed input     */
    //     TX90.currVelocity -= 0.05;

    // if (commander->buttons[1] == 1 && TX90.currVelocity <= 0.95)    /* Check for increase speed input   */
    //     TX90.currVelocity += 0.05;
}
/* ----------------------------------------------------------------------
 *              -------  robot_state Callback function   -------
 * ----------------------------------------------------------------------- */       
void robotStateCallback (const control_msgs::FollowJointTrajectoryFeedback::ConstPtr&   RobotState)
{
    /* Update currPos from the feedback, measuring on the sensors   */
    int jointAtGoalCounter = 0;

    if (!robotInitialized)
    {
        robotInitialized = true;
        for (int i = 0; i < 6; i++)
            TX90.currPos[i] = RobotState->actual.positions[i];
    }

    for (int i = 0; i < 6; i++)
    {
        if (TX90.goalPosition[i] >= 0)
        {
            if (((TX90.currPos[i]) >= 0.999 * TX90.goalPosition[i]) && ((TX90.currPos[i]) <= 1.001 * TX90.goalPosition[i]) && goalExists == true && robotInitialized == true) 
                jointAtGoalCounter++;

            TX90.currPos[i] = RobotState->actual.positions[i];
        }

        else if (TX90.goalPosition[i] < 0)
        {
            if (((TX90.currPos[i]) <= 0.999 * TX90.goalPosition[i]) && ((TX90.currPos[i]) >= 1.001 * TX90.goalPosition[i]) && goalExists == true && robotInitialized == true)
                jointAtGoalCounter++;

            TX90.currPos[i] = RobotState->actual.positions[i];  
        }
    }

    if (jointAtGoalCounter == 6)
    {
        ManPubTimer = 0;
        movementFeedbackMsg.flags |= GOAL_REACHED;
        goalExists  = false;
        feedbackTransmitReady = true;
    }
}

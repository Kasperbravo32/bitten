/* -----------------------------------------------------------------------
 * Filename: movement_node.cpp
 * Author: Frederik Snedevind & Kasper Banke Jørgensen
 * Purpose: Create the 'movement' node in the Leica Robot system
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

trajectory_msgs::JointTrajectory        jointPathMsgTest;               /* Message used to transmit wanted position to motion_streaming_interface       */

bitten::feedback_msg                    movementFeedbackMsg;        /* Message used to report back when wanted position is reached, e.g. waypoint-mode  */

/* ----------------------------------------------------------------------
 *                    -------  Global Variables   -------
 * ----------------------------------------------------------------------- */
bool robotInitialized       = false;
bool goalExists             = false;
bool jointTransmitReady     = true;
bool feedbackTransmitReady  = false;
bool stopMotion             = false;

bool justMovedArr[6]        ={false,
                              false,
                              false,
                              false,
                              false,
                              false};

int OPERATING_MODE = 0;
int buttonDebouncers[12];

TX90_c TX90;

 /* ----------------------------------------------------------------------
 *                         -------  Main   -------
 * ----------------------------------------------------------------------- */        
int main (int argc , char **argv) 
{    
    movementFeedbackMsg.senderID = MOVEMENT_ID;

    ros::init(argc , argv , "movement_node");
    ros::NodeHandle n;

    ros::Subscriber movement_sub = n.subscribe<bitten::control_msg>                         ("movement_topic"       , 2 , commanderCallback);
    ros::Subscriber feedback_sub = n.subscribe<control_msgs::FollowJointTrajectoryFeedback> ("feedback_states"      , 2 , robotStateCallback);
    
    ros::Publisher  movement_pub = n.advertise<trajectory_msgs::JointTrajectory>            ("joint_path_command"   , 5, true);
    ros::Publisher  movement_clearPub = n.advertise<trajectory_msgs::JointTrajectory>       ("joint_path_command"   , 5, true);
    ros::Publisher  feedback_pub = n.advertise<bitten::feedback_msg>                        ("movement_feedback"    , LOOP_RATE_INT);

    jointPathPointMsg.positions.resize(6);
    jointPathPointMsg.velocities.resize(6);
    jointPathPointMsg.accelerations.resize(6);
    jointPathPointMsg.effort.resize(1);

    jointPathMsgTest.joint_names.clear();
    jointPathMsgTest.points.clear();

    for (int i = 0; i < 6; i++)
        jointPathMsg.joint_names.push_back(TX90.getJointName(i));

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
                    jointPathMsg.points.clear();
                    jointPathPointMsg.positions.clear();
                    jointPathPointMsg.velocities.clear();

                    for (int i = 0; i < 6; i++)
                    {
                        jointPathPointMsg.positions.push_back(TX90.getGoalPos(i));
                        jointPathPointMsg.velocities.push_back(TX90.getCurrVelocity());
                    }
                    
                    jointPathMsg.points.push_back(jointPathPointMsg);
                }

                if (jointTransmitReady)
                {
                    movement_pub.publish(jointPathMsg);
                    jointTransmitReady = false;
                }

                if (feedbackTransmitReady)
                {
                    feedback_pub.publish(movementFeedbackMsg);   
                    movementFeedbackMsg.flags = 0;
                    feedbackTransmitReady = false;
                }
                break;

            case MANUAL_ID:
            {
                if (jointTransmitReady)
                {
                    jointPathMsg.points.clear();
                    jointPathPointMsg.positions.clear();
                    jointPathPointMsg.velocities.clear();

                    for (int i = 0; i < 6; i++)
                    {
                        jointPathPointMsg.positions.push_back(TX90.getGoalPos(i));
                        jointPathPointMsg.velocities.push_back(TX90.getCurrVelocity());
                    }

                    jointPathMsg.points.push_back(jointPathPointMsg);
                    movement_pub.publish(jointPathMsgTest);
                    movement_pub.publish(jointPathMsg);
                    jointTransmitReady = false;
                }

                if (feedbackTransmitReady)
                {
                    movementFeedbackMsg.flags |= SENT_CURR_POS;
                    for(int i = 0; i < 6; i++)
                        movementFeedbackMsg.positions[i] = TX90.getCurrPos(i);
                        
                    feedback_pub.publish(movementFeedbackMsg);
                    movementFeedbackMsg.flags = 0;
                    feedbackTransmitReady = false;
                }

                if (stopMotion == true)
                {
                    movement_pub.publish(jointPathMsgTest);
                    stopMotion = false;
                }

                break;
            }
            default:
                break;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

/* ----------------------------------------------------------------------
 *             -------  commander_node Callback function   -------
 * ----------------------------------------------------------------------- */       
void commanderCallback(const bitten::control_msg::ConstPtr& commander)
{
    switch(commander->id)
    {
        case MANUAL_ID:
            if (OPERATING_MODE != MANUAL_ID)
                OPERATING_MODE = MANUAL_ID;

            if (commander->flags & GET_CURR_POS) /* sent current position to commander */
            {
                for (int i = 0; i < 6; i++)
                    movementFeedbackMsg.positions[i] = TX90.getCurrPos(i);

                movementFeedbackMsg.flags = SENT_CURR_POS;
                feedbackTransmitReady = true;
            }

            if (commander->buttons[1] == 1 && commander->buttons[2] == 1) /* Return to home pressed, while deadman is pressed */
            {
                if (++buttonDebouncers[0] == 5)
                {
                    for (int i = 0; i < 6; i++)
                    {
                        TX90.setGoalPos(i, TX90.getResetStatePos(i));
                        jointTransmitReady = true;
                    }
                }
            }
            else
                buttonDebouncers[0] = 0;

            if (commander->buttons[6] == 1)
            {
                if (++buttonDebouncers[6] == 6)
                {
                    if (TX90.getCurrVelocity() > 0)
                    {
                        TX90.setCurrVelocity(TX90.getCurrVelocity() - 0.05);
                        ROS_INFO("Current Velocity: %f",TX90.getCurrVelocity());
                    }
                }
                else if (buttonDebouncers[6] == 60)
                {
                    TX90.setCurrVelocity(0);
                    ROS_INFO("Current Velocity: %f",TX90.getCurrVelocity());
                }
            }
            else
                buttonDebouncers[6] = 0;

            if (commander->buttons[7] == 1) /* Increment Current Velocity */
            {
                if (++buttonDebouncers[7] == 6)
                {
                    if (TX90.getCurrVelocity() < 1)
                    {
                        TX90.setCurrVelocity(TX90.getCurrVelocity() + 0.05);
                        ROS_INFO("Current Velocity: %f",TX90.getCurrVelocity());
                    }
                }
                else if (buttonDebouncers[7] == 60)
                {
                    TX90.setCurrVelocity(1);
                    ROS_INFO("Current Velocity: %f",TX90.getCurrVelocity());
                }
            }
            else
                buttonDebouncers[7] = 0;

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
        break;

        case WP_ID:
            if (goalExists == false)
            {
                if (OPERATING_MODE != WP_ID)
                    OPERATING_MODE = WP_ID;

                std::cout << "OK!" << std::endl << "Moving robot to: " << commander->programName << "...";
                for (int i = 0; i < 6; i++)
                    TX90.setGoalPos(i, commander->jointPosition[i]);

                goalExists = true;
                jointTransmitReady = true;
            }
        break;

        default:
        break;
    }
}

/* ----------------------------------------------------------------------
 *              -------  robot_state Callback function   -------
 * ----------------------------------------------------------------------- */       
void robotStateCallback (const control_msgs::FollowJointTrajectoryFeedback::ConstPtr&   RobotState)
{
    /* Update currPos from the feedback, measuring on the sensors   */
    if (!robotInitialized)
    {
        robotInitialized = true;
        for (int i = 0; i < 6; i++)
        {
            TX90.setCurrPos(i , RobotState->actual.positions[i]);
            TX90.setGoalPos(i , TX90.getCurrPos(i));
        }
    }

    for (int i = 0; i < 6; i++)
        TX90.setCurrPos(i, RobotState->actual.positions[i]);

    if(goalExists == true)
    {
        int jointAtGoalCounter = 0;
        double posBoundry;
        for (int i = 0; i < 6; i++)
        {
            posBoundry = 0.02 * TX90.getMaxRotation(i);
            if((TX90.getCurrPos(i) >= (TX90.getGoalPos(i) - posBoundry)) && (TX90.getCurrPos(i) <= (TX90.getGoalPos(i) + posBoundry)))
                jointAtGoalCounter++;
        }

        if (jointAtGoalCounter == 6)
        {
            movementFeedbackMsg.flags |= GOAL_REACHED;
            goalExists = false;
            feedbackTransmitReady = true;
        }
    }
}

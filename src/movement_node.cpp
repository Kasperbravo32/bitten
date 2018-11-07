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
int jointsNearlyAtGoal(void);
int jointsNearlyAtGoal2(void);
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
bool sendCommand = true;

int OPERATING_MODE = 0;

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
    
    ros::Publisher  movement_pub = n.advertise<trajectory_msgs::JointTrajectory>            ("joint_path_command"   , 2);
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

    static int PubTimer = LOOP_RATE_INT / 5;

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
                    jointPathMsg.joint_names.clear();
                    jointPathMsg.points.clear();
                    jointPathPointMsg.positions.clear();
                    jointPathPointMsg.velocities.clear();
                    
                    for (int i = 0; i < 6; i++)
                    {
                        jointPathMsg.joint_names.push_back(TX90.getJointName(i));
                        jointPathPointMsg.positions.push_back(TX90.getGoalPos(i));
                        jointPathPointMsg.velocities.push_back(TX90.getCurrVelocity()*TX90.getMaxVelocity(i));
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
                // jointsNearlyAtGoal();

/*                if(numberOfNearlyGoals == 6)
                {
                    sendCommand = true;
                }
                else */

                if(sendCommand == true)
                {
                    // ros::spinOnce();
                    jointPathMsg.joint_names.clear();
                    jointPathMsg.points.clear();
                    jointPathPointMsg.positions.clear();
                    jointPathPointMsg.velocities.clear();

                    for (int i = 0; i < 6; i++)
                    {                  
                        jointPathMsg.joint_names.push_back(TX90.getJointName(i));
                        jointPathPointMsg.positions.push_back(TX90.getGoalPos(i));
                    }

                    jointPathMsg.points.push_back(jointPathPointMsg);
                    jointTransmitReady = true;
                    sendCommand = false;
                    PubTimer = LOOP_RATE_INT / 3;
                }
                else if (! --PubTimer)
                {
                    sendCommand = true;
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
    static bool justMoved = false;
    static int ButtonCounter = 0;

    switch(commander->id)
    {
        case MANUAL_ID:
        {
            if (OPERATING_MODE != MANUAL_ID)
                OPERATING_MODE = MANUAL_ID;

            if (commander->flags & GET_CURR_POS) //sent current position to commander
            {
                for (int i = 0; i < 6; i++)
                    movementFeedbackMsg.positions[i] = TX90.getCurrPos(i);

                movementFeedbackMsg.flags = SENT_CURR_POS;
                feedbackTransmitReady = true;
            }

            if (commander->buttons[1] == 1 && commander->buttons[2] == 1) //reset and deadmans button
            {
                for (int i = 0; i < 6; i++)
                {
                    sendCommand = true;
                    TX90.setGoalPos(i, TX90.getResetStatePos(i));
                }
            }

            if (commander->buttons[7] == 1) //increment current velocity
            {
                ButtonCounter++;
                if (ButtonCounter == 6)
                {
                    if (TX90.getCurrVelocity() < 1)
                    {
                        TX90.setCurrVelocity(TX90.getCurrVelocity() + 0.05);
                        ROS_INFO("Current Velocity: %f",TX90.getCurrVelocity());
                    }
                }
                else if (ButtonCounter == 60)
                {
                    TX90.setCurrVelocity(1);
                    ROS_INFO("Current Velocity: %f",TX90.getCurrVelocity());
                }
            }
            else if (commander->buttons[6] == 1) //decrease current velocity
            {
                ButtonCounter++;
                if (ButtonCounter == 6)
                {
                    if (TX90.getCurrVelocity() > 0)
                    {
                        TX90.setCurrVelocity(TX90.getCurrVelocity() - 0.05);
                        ROS_INFO("Current Velocity: %f",TX90.getCurrVelocity());
                    }
                }

                if (ButtonCounter == 60)
                {
                    TX90.setCurrVelocity(0);
                    ROS_INFO("Current Velocity: %f",TX90.getCurrVelocity());
                }
            }
            else
                ButtonCounter = 0;

            // int jointAtGoalCounter = 0;
            // for (int i = 0; i < 6; i++)
            // {
            //     static double posBoundry = 0.02 * TX90.getMaxRotation(i);
            //     if((TX90.getCurrPos(i) >= (TX90.getGoalPos(i) - posBoundry)) && (TX90.getCurrPos(i) <= (TX90.getGoalPos(i) + posBoundry)))
            //         jointAtGoalCounter++;
            // }
            if (jointsNearlyAtGoal2() == 6)
            {
                for (int i = 0; i < 6; i++)
                {
                    static double intendedGoal;
                    if (commander->jointVelocity[i] > 0)
                    {
                        if(((i == 0 || i == 2 || i == 3) && commander->buttons[8] == 1) || ((i == 1 || i == 4 || i == 5) && commander->buttons[2] == 1))
                        {
                            intendedGoal = TX90.getGoalPos(i) + ((1/LOOP_RATE_INT) * commander->jointVelocity[i] * TX90.getMaxVelocity(i) * TX90.getCurrVelocity());
                            if (intendedGoal < TX90.getMaxRotation(i))
                            {
                                TX90.setGoalPos(i, intendedGoal);
                                justMoved = true;
                                sendCommand = true;
                            }
                        }
                    }
                    else if (commander->jointVelocity[i] < 0)
                    {
                        if(((i == 0 || i == 2 || i == 3) && commander->buttons[8] == 1) || ((i == 1 || i == 4 || i == 5) && commander->buttons[2] == 1))
                        {
                            intendedGoal = TX90.getGoalPos(i) + ((1/LOOP_RATE_INT) * commander->jointVelocity[i] * TX90.getMaxVelocity(i) * TX90.getCurrVelocity());
                            if (intendedGoal > TX90.getMinRotation(i))
                            {
                                TX90.setGoalPos(i, intendedGoal);
                                justMoved = true;
                                sendCommand = true;
                            }
                        }
                    }
                    else
                    {
                        if (justMoved == true)
                        {
                            TX90.setGoalPos(i, TX90.getCurrPos(i));
                            justMoved = false;
                            sendCommand = true;
                        }
                    }
                }
            }
            break;
        }
        case WP_ID:
        {
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
        }
        default:
            ROS_INFO("Entered Missing ID callback");
            break;
    }
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
        {
            TX90.setCurrPos(i , RobotState->actual.positions[i]);
            TX90.setGoalPos(i , TX90.getCurrPos(i));
        }
    }

    for (int i = 0; i < 6; i++)
    {
        TX90.setCurrPos(i, RobotState->actual.positions[i]);

        // if (goalExists == true)
        // {
        //     static double posBoundary = 0.02 * TX90.getMaxRotation(i);
        //     if((TX90.getCurrPos(i) >= (TX90.getGoalPos(i) - posBoundary)) && (TX90.getCurrPos(i) <= (TX90.getGoalPos(i) + posBoundary)))
        //         jointAtGoalCounter++;
        // }
    }
    int goals;
    if(goalExists == true)
        goals = jointsNearlyAtGoal2();
    if (/*jointAtGoalCounter*/goals == 6)
    {
        movementFeedbackMsg.flags |= GOAL_REACHED;
        goalExists = false;
        feedbackTransmitReady = true;
    }
}

int jointsNearlyAtGoal(void)
{
    static double currDistance;
    static double totalDistance;
    static double doneDistance;
    int numberOfNearlyGoals = 0;
    for(int i = 0; i < 6; i++)
    {
        if(TX90.getCurrPos(i) != TX90.getGoalPos(i) && TX90.getGoalPos(i) != TX90.getLastGoalPos(i))
        {
            currDistance = std::abs(TX90.getGoalPos(i) - TX90.getCurrPos(i));
            totalDistance = std::abs(TX90.getGoalPos(i) - TX90.getLastGoalPos(i));
            doneDistance = totalDistance - currDistance;
            if(doneDistance < 0.00001)
                doneDistance = 0.00001;
            if(totalDistance < 0.00001)
                totalDistance = 0.00001;
        }
        else
        {
            doneDistance = 1;
            totalDistance = 1;
        }
        if((doneDistance / totalDistance) > 0.6)
            numberOfNearlyGoals++;
    }
    return numberOfNearlyGoals;
}

int jointsNearlyAtGoal2(void)
{
    int jointAtGoalCounter = 0;
    for (int i = 0; i < 6; i++)
    {
        static double posBoundry = 0.02 * TX90.getMaxRotation(i);
        if((TX90.getCurrPos(i) >= (TX90.getGoalPos(i) - posBoundry)) && (TX90.getCurrPos(i) <= (TX90.getGoalPos(i) + posBoundry)))
            jointAtGoalCounter++;
    }
    return jointAtGoalCounter;
}
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


trajectory_msgs::JointTrajectory        jointPathMsgTest;               /* Message used to transmit wanted position to motion_streaming_interface           */

trajectory_msgs::JointTrajectory        jointPathMsgTest2;
trajectory_msgs::JointTrajectoryPoint   jointPathPointMsgTest2;
// trajectory_msgs::JointTrajectoryPoint   jointPathPointMsgTest;          /* Message used to contain specific points of wanted position, part of jointPathMsg */


/* ----------------------------------------------------------------------
 *                    -------  Global Variables   -------
 * ----------------------------------------------------------------------- */
bool robotInitialized       = false;
bool goalExists             = false;
bool jointTransmitReady     = true;
bool feedbackTransmitReady  = false;
bool clearTransmitReady     = false;
bool twistJ0Ready           = false;
bool sendCommand = true;

bool justMovedArr[6]        ={false,
                              false,
                              false,
                              false,
                              false,
                              false};

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
    {                  
        jointPathPointMsgTest2.positions.push_back(1);
        jointPathPointMsgTest2.velocities.push_back(1);
    }
    jointPathMsgTest2.points.push_back(jointPathPointMsgTest2);

    ros::Rate loop_rate(LOOP_RATE_INT);
    sleep(1);

    if (movement_sub && feedback_sub && movement_pub && feedback_pub)
        ROS_INFO("Initiated %s",nodeNames[MOVEMENT_NODE].c_str());
    else
        ROS_INFO("Failed to to initiate %s",nodeNames[MOVEMENT_NODE].c_str());

    int PubTimer = LOOP_RATE_INT / 5;

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
                        jointPathPointMsg.velocities.push_back(TX90.getCurrVelocity()/**TX90.getMaxVelocity(i)*/);
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
                // if(sendCommand == true)
                // {
                //     jointPathMsg.joint_names.clear();
                //     jointPathMsg.points.clear();
                //     jointPathPointMsg.positions.clear();
                //     jointPathPointMsg.velocities.clear();

                //     for (int i = 0; i < 6; i++)
                //     {                  
                //         jointPathMsg.joint_names.push_back(TX90.getJointName(i));
                //         jointPathPointMsg.positions.push_back(TX90.getGoalPos(i));
                //         jointPathPointMsg.velocities.push_back(TX90.getCurrVelocity());
                //     }

                //     jointPathMsg.points.push_back(jointPathPointMsg);
                //     jointTransmitReady = true;
                //     sendCommand = false;
                //     PubTimer = LOOP_RATE_INT/2;
                // }
                // else if(! PubTimer--)
                // {
                //     sendCommand = true;
                // }

                if (jointTransmitReady)
                {
                    jointPathMsg.joint_names.clear();
                    jointPathMsg.points.clear();
                    jointPathPointMsg.positions.clear();
                    jointPathPointMsg.velocities.clear();

                    for (int i = 0; i < 6; i++)
                    {                  
                        jointPathMsg.joint_names.push_back(TX90.getJointName(i));
                        jointPathPointMsg.positions.push_back(TX90.getGoalPos(i));
                        jointPathPointMsg.velocities.push_back(TX90.getCurrVelocity());
                    }

                    jointPathMsg.points.push_back(jointPathPointMsg);

                    movement_clearPub.publish(jointPathMsgTest);
                    movement_pub.publish(jointPathMsg);
                    jointTransmitReady = false;
                }

                if (clearTransmitReady)
                {
                    movement_pub.publish(jointPathMsgTest);
                    clearTransmitReady = false;
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

            static int b1c = 0;
            if (commander->buttons[1] == 1 && commander->buttons[2] == 1) //reset and deadmans button
            {
                if (++b1c == 5)
                {
                    for (int i = 0; i < 6; i++)
                    {
                        // sendCommand = true;
                        TX90.setGoalPos(i, TX90.getResetStatePos(i));
                        jointTransmitReady = true;
                    }
                }
            }
            else
                b1c = 0;

            static int b4c = 0;

            if (commander->buttons[4] == 1)
            {
                if (++b4c == 5)
                {
                    ROS_INFO("Clicked clearTrajButton");
                    clearTransmitReady = true;
                }
            }
            else
                b4c = 0;

            static int b5c = 0;
            if (commander->buttons[5] == 1)
            {
                if (++b5c == 5)
                {
                    ROS_INFO("Clicked Twist button");
                    twistJ0Ready = true;
                }
            }
            else
                b5c = 0;

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

            // if (jointsNearlyAtGoal2() == 6)
            // {
                double intendedGoal;
                for (int i = 0; i < 6; i++)
                {
                    if ((commander->jointVelocity[i] > 0.2) && (((i == 0 || i == 2 || i == 3) && commander->buttons[8] == 1) || ((i == 1 || i == 4 || i == 5) && commander->buttons[2] == 1))/*0*/)
                    {
                        // if()
                        // {
                            // intendedGoal = TX90.getCurrPos(i) + ((1/LOOP_RATE_INT) * commander->jointVelocity[i] * TX90.getMaxVelocity(i) * TX90.getCurrVelocity() * 3);
                            // if (intendedGoal < TX90.getMaxRotation(i))
                            // {
                                // if(commander->jointVelocity[i] > 0.2)
                                // {
                                    if(justMovedArr[i] == false)
                                    {
                                        TX90.setGoalPos(i, TX90.getMaxRotation(i) /*intendedGoal*/);
                                        TX90.setCurrVelocity(/*commander->jointVelocity[i]*/ 1);
                                        justMovedArr[i] = true;
                                        jointTransmitReady = true;
                                    }
                                    // justMoved = true;
                                // }
                                // sendCommand = true;
                            // }
                        // }
                    }
                    else if((commander->jointVelocity[i] < -0.2 /*0*/) && (((i == 0 || i == 2 || i == 3) && commander->buttons[8] == 1) || ((i == 1 || i == 4 || i == 5) && commander->buttons[2] == 1)))
                    {
                        // if()
                        // {
                            // intendedGoal = TX90.getCurrPos(i) + ((1/LOOP_RATE_INT) * commander->jointVelocity[i] * TX90.getMaxVelocity(i) * TX90.getCurrVelocity() * 3);
                            // if (intendedGoal > TX90.getMinRotation(i))
                            // {
                                if(justMovedArr[i] == false)
                                {
                                   TX90.setGoalPos(i, TX90.getMinRotation(i) /*intendedGoal*/);
                                    // if(commander->jointVelocity[i] < -0.2)
                                    TX90.setCurrVelocity(/*commander->jointVelocity[i] * -1*/ 1);
                                    // justMoved = true;
                                    justMovedArr[i] = true;
                                    jointTransmitReady = true;
                                }
                                // sendCommand = true;
                            // }
                        // }
                    }
                    else
                    {
                        if (justMovedArr[i] == true)
                        {
                            ROS_INFO("moved back to 0");
                            TX90.setGoalPos(i, TX90.getCurrPos(i));
                            TX90.setCurrVelocity(/*0.2*/ 1);
                            // justMoved = false;
                            justMovedArr[i] = false;
                            jointTransmitReady = true;
                            // sendCommand = true;
                        }
                    }
                }
            // }
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
        TX90.setCurrPos(i, RobotState->actual.positions[i]);

    if(goalExists == true)
        jointAtGoalCounter = jointsNearlyAtGoal2();

    if (jointAtGoalCounter == 6)
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
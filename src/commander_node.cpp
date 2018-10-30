/* -----------------------------------------------------------------------
 * Filename: commander_node.cpp
 * Author: Frederik Snedevind & Kasper Banke Jørgensen
 * Purpose: Create the 'Commander' node in the Leica Robot system
 * Date of dev.: September 2018 - January 2019
 * 
 * -----------------------------------------------------------------------
 *                      -------  Libraries   -------
 * ----------------------------------------------------------------------- */
// #include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <array>
#include <sstream>
#include <iostream>
#include <stdio.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "commander_node.h"
#include "global_node_definitions.h"

#include "bitten/feedback_msg.h"
#include "bitten/control_msg.h"

 /* ----------------------------------------------------------------------
 *                      -------  Initializing   -------
 * ----------------------------------------------------------------------- */
// void InitRobot();

void manualCallback             (const bitten::control_msg::ConstPtr& manual);
void wpCallback                 (const bitten::control_msg::ConstPtr& wp);
// void testCallback               (const bitten::control_msg::ConstPtr& test);
void movementFeedbackCallback   (const bitten::feedback_msg::ConstPtr& moveFeedback);

 /* ----------------------------------------------------------------------
 *                    -------  Message objects   -------
 * ----------------------------------------------------------------------- */
// MsgType_s manualInputMsg;               /* Used to collect data from:       manual node                                         */
// MsgType_s wpInputMsg;                   /* Used to collect data from:       waypoint node                                       */
// MsgType_s testInputMsg;                 /* Used to collect data from:       test node                                           */

// feedbackMsg_s fbMsg;

//sensor_msgs::JointState msg;
// trajectory_msgs::JointTrajectory msg;
// trajectory_msgs::JointTrajectoryPoint point_msg;

bitten::feedback_msg commanderFeedbackMsg;     /* Used to send feedback to various nodes */
bitten::control_msg passOnMsg;

 /* ----------------------------------------------------------------------
 *                    -------  Global Variables   -------
 * ----------------------------------------------------------------------- */
bool goalExists = false;
bool waiting = false;
bool ping = true;
bool fbTransmitReady = false;
bool jointStatesTransmitReady = false;
bool not_run = true;
bool robotOccupied = false;

const   int PING_RATE = LOOP_RATE_INT / 2;
        int poll_timer = LOOP_RATE_INT*2;
        int ping_timer = PING_RATE;
        int ping_timeout = PING_RATE;
        int pub_counter = LOOP_RATE_INT / 10;

double goalArray[6];
uint8_t jointsdone = 0;

 /* ----------------------------------------------------------------------
 *                          -------  Main   -------
 * ----------------------------------------------------------------------- */        
int main(int argc , char **argv)
{
    ROS_INFO("Initiating %s",nodeNames[COMMANDER_NODE].c_str());
    
    ros::init(argc , argv , "commander_node");
    ros::NodeHandle n;
    commanderFeedbackMsg.senderID = COMMANDER_ID;   

    ros::Subscriber manual_sub        = n.subscribe<bitten::control_msg> ("manual_topic"     , 5 , &manualCallback);
    ros::Subscriber wp_sub            = n.subscribe<bitten::control_msg> ("wp_topic"         , 10, &wpCallback);
    ros::Subscriber movement_feedback = n.subscribe<bitten::feedback_msg>("movement_feedback", 10, &movementFeedbackCallback);
    
    ros::Publisher commander_pub      = n.advertise<bitten::control_msg> ("movement_topic", LOOP_RATE_INT);
    ros::Publisher commander_fb_pub   = n.advertise<bitten::feedback_msg>("feedback_topic", LOOP_RATE_INT);

    ros::Rate loop_rate(LOOP_RATE_INT);
    INPUT_MODE = POLL_MODE;

    passOnMsg.nodeName = nodeNames[COMMANDER_NODE];

    sleep(1);
    if (manual_sub && wp_sub && commander_pub && commander_fb_pub && movement_feedback)
        ROS_INFO("Initiated %s",nodeNames[COMMANDER_NODE].c_str());
    else
        ROS_INFO("Didn't initiate %s",nodeNames[COMMANDER_NODE].c_str());

    // int ManPubTimer = LOOP_RATE_INT / 5;

/*  -------------------------------------------------
         SUPERLOOP
    ------------------------------------------------- */
    while(ros::ok())
    {
        switch(INPUT_MODE)
        {
            case MANUAL_MODE:

            break;

            case WP_MODE:
                // msg.joint_names.clear();
                // point_msg.positions.clear();

            break;

            // case TEST_MODE:
            // if (not_run)
            // {
            //     for(int i = 0; i < 6; i++)
            //     {
            //         // msg.joint_names.push_back(TX90.jointNames[i]);
            //         if (i != 0)
            //             point_msg.positions.push_back(0);
            //         else
            //             point_msg.positions.push_back(0);
                    
            //         point_msg.velocities.push_back(1);
            //         point_msg.accelerations.push_back(0);
            //     }

            //     point_msg.effort.push_back(0);
            //     point_msg.time_from_start = ros::Duration(0.5);
            //     msg.points.push_back(point_msg);
            //     jointStatesTransmitReady = true;
            //     not_run = false;
            // }
            // break;

            case POLL_MODE:

                if (! --poll_timer)
                {
                    ROS_INFO("waiting in poll_mode");
                    poll_timer = 2*LOOP_RATE_INT;
                }
            break;
            default:

            break;
        }

        if (INPUT_MODE != POLL_MODE)
        {
            if (! --ping_timer)
            {
                commanderFeedbackMsg.flags = PING;
                fbTransmitReady = true;

                if (ping == true)
                {
                    ping = false;
                    ping_timer = PING_RATE;
                }
                else
                {
                    ROS_INFO("Ping timeout, returning to poll_mode");
                    ping_timer = PING_RATE;
                    INPUT_MODE = POLL_MODE;
                    ping = true;
                }
            }
        }
        
        if (jointStatesTransmitReady)
        {
            commander_pub.publish(passOnMsg);
            jointStatesTransmitReady = false;
        }
        
        if (fbTransmitReady)
        {
            commander_fb_pub.publish(commanderFeedbackMsg);
            fbTransmitReady = false;
            commanderFeedbackMsg.flags = 0;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}

 /* ----------------------------------------------------------------------
 *                 -------  manual Callback function   -------
 * ----------------------------------------------------------------------- */       
void manualCallback (const bitten::control_msg::ConstPtr& manual)
{
    if (manual->flags & PONG)
        ping = true;

    if (manual->flags & ESTABLISH_CONNECTION)
    {
        if (INPUT_MODE == POLL_MODE)
        {
            INPUT_MODE = MANUAL_MODE;
            ROS_INFO("Established connection to %s",nodeNames[MANUAL_NODE].c_str());
            commanderFeedbackMsg.flags = ACK;
            commanderFeedbackMsg.recID = MANUAL_ID;

            fbTransmitReady = true;
        }
    }

    if (INPUT_MODE == MANUAL_MODE)
    {
        passOnMsg.buttons = manual->buttons;
        passOnMsg.jointVelocity = manual->jointVelocity;
        passOnMsg.id = MANUAL_ID;
        jointStatesTransmitReady = true; 
    }
}

 /* ----------------------------------------------------------------------
 *                -------  Waypoint Callback function   -------
 * ----------------------------------------------------------------------- */ 
void wpCallback (const bitten::control_msg::ConstPtr& wp)
{
    switch(wp->flags)
    {
        case ESTABLISH_CONNECTION:
            if (INPUT_MODE == POLL_MODE)
            {
                INPUT_MODE = WP_MODE;
                ROS_INFO("Established connection to %s",nodeNames[WP_NODE].c_str());
                commanderFeedbackMsg.flags = ACK;
                commanderFeedbackMsg.recID = WP_ID;
                fbTransmitReady = true;
            }
        break;

        case TERMINATE_CONNECTION:
            if (INPUT_MODE == WP_MODE)
            {
                INPUT_MODE = POLL_MODE;
                commanderFeedbackMsg.flags = ACK;
                commanderFeedbackMsg.recID = WP_ID;
                fbTransmitReady = true;
                ROS_INFO("Terminated connection to %s", nodeNames[WP_NODE].c_str());
            }
        break;

        case NEW_WAYPOINT:
            if (INPUT_MODE == WP_MODE)
            {
                if (robotOccupied == false)
                {
                    passOnMsg.programName = wp->programName;
                    ROS_INFO("Setting new goal to: %s",wp->programName.c_str());
                    robotOccupied = true;

                    passOnMsg.jointPosition = wp->jointPosition;
                    passOnMsg.id = WP_ID;
                    jointStatesTransmitReady = true;       
                }   
            }
        break;

        case PONG:
            ping = true;
        break;
    }
}

 /* ----------------------------------------------------------------------
 *                  -------  test Callback function   -------
 * ----------------------------------------------------------------------- */ 
// void testCallback (const bitten::control_msg::ConstPtr& test)
// {
//     /*
//      * MANDAG: Snak om hvordan TEST beskeden ser ud. én TEST besked med flere waypoints?
//      * */

// }

 /* ----------------------------------------------------------------------
 *                  -------  MovementFeedback Callback function   -------
 * ----------------------------------------------------------------------- */
void movementFeedbackCallback (const bitten::feedback_msg::ConstPtr& moveFeedback)
{
    ROS_INFO("GOT SOME FEEDBACK");
    if (moveFeedback->flags & GOAL_REACHED)
    {
        switch(INPUT_MODE)
        {
            case WP_MODE:
                // ROS_INFO("ENTERED WP_MODE SWITCH");
                commanderFeedbackMsg.recID = WP_ID;
                commanderFeedbackMsg.flags |= GOAL_REACHED;
                fbTransmitReady = true;
                robotOccupied = false;
            break;

            case MANUAL_MODE:
                // ROS_INFO("Entered MANUAL_MODE in movementFeedback");
                // commanderFeedbackMsg.recID = MANUAL_ID;
                // commanderFeedbackMsg.flags |= GOAL_REACHED;
                // fbTransmitReady = true;
                robotOccupied = false;
            break;

            
        }
    }
}

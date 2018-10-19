/* -----------------------------------------------------------------------
 * Filename: pubnode3.cpp
 * Author: Frederik Snedevind
 * Purpose: Create the 'Commander' node in the Leica Robot system
 * Date of dev.: September 2018 - January 2019
 * 
 * -----------------------------------------------------------------------
 *                      -------  Libraries   -------
 * ----------------------------------------------------------------------- */

#include <sensor_msgs/JointState.h>
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
void InitRobot();

void manualCallback (const bitten::control_msg::ConstPtr& manual);
void wpCallback     (const bitten::control_msg::ConstPtr& wp);
void testCallback   (const bitten::control_msg::ConstPtr& test);

 /* ----------------------------------------------------------------------
 *                    -------  Message objects   -------
 * ----------------------------------------------------------------------- */
MsgType_s manualInputMsg;               /* Used to collect data from:       manual node                                         */
MsgType_s wpInputMsg;                   /* Used to collect data from:       waypoint node                                       */
MsgType_s testInputMsg;                 /* Used to collect data from:       test node                                           */

feedbackMsg_s fbMsg;

//sensor_msgs::JointState msg;
trajectory_msgs::JointTrajectory msg;
trajectory_msgs::JointTrajectoryPoint point_msg;

bitten::feedback_msg commanderFeedbackMsg;     /* Used to send feedback to various nodes */


 /* ----------------------------------------------------------------------
 *                    -------  Global Variables   -------
 * ----------------------------------------------------------------------- */
bool goalExists = false;
bool waiting = false;
bool ping = true;
bool fbTransmitReady = false;
bool jointStatesTransmitReady = false;
bool not_run = true;

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

    commanderFeedbackMsg.senderID = COMMANDER_ID;
    ROS_INFO("Initiating system...");
    InitRobot();

    ros::init(argc , argv , "commander_node");
    ros::NodeHandle n;

    ROS_INFO("Subscribing to %s...", topicNames[MANUAL_TOPIC].c_str());
    ros::Subscriber manual_sub  = n.subscribe<bitten::control_msg>("manual_topic" , 3*LOOP_RATE_INT , &manualCallback);
    if (manual_sub)
        ROS_INFO("Subscribed to %s!\n", topicNames[MANUAL_TOPIC].c_str());
    else
        ROS_INFO("Couldn't subscribe to %s.\n", topicNames[MANUAL_TOPIC].c_str());



    ROS_INFO("Subscribing to %s...", topicNames[WP_TOPIC].c_str());
    ros::Subscriber wp_sub  = n.subscribe<bitten::control_msg>("wp_topic" , 3*LOOP_RATE_INT , &wpCallback);
    if (wp_sub)
        ROS_INFO("Subscribed to %s!\n", topicNames[WP_TOPIC].c_str());
    else
        ROS_INFO("Couldn't subscribe to %s.\n", topicNames[WP_TOPIC].c_str());

    ROS_INFO("Publishing on \"joint_path_command\"");
    ros::Publisher commander_pub    = n.advertise<trajectory_msgs::JointTrajectory>  ("joint_path_command" , 3*LOOP_RATE_INT);
    

    ROS_INFO("Publishing on \"%s\"",topicNames[FEEDBACK_TOPIC].c_str());
    ros::Publisher commander_fb_pub = n.advertise<bitten::feedback_msg>      ("feedback_topic" , 3*LOOP_RATE_INT);
    if (commander_fb_pub)
        ROS_INFO("Publishing succesful\n");

    ros::Publisher test_pub = n.advertise<trajectory_msgs::JointTrajectory>("test0_topic", 3*LOOP_RATE_INT);

    ros::Rate loop_rate(2);

    ros::spinOnce();
    INPUT_MODE = POLL_MODE;

    point_msg.positions.resize(6);
    point_msg.velocities.resize(6);
    point_msg.accelerations.resize(6);
    point_msg.effort.resize(1);

    sleep(1);
    ROS_INFO("Ready for operation");

/*  -------------------------------------------------
         SUPERLOOP
    ------------------------------------------------- */
    while(ros::ok())
    {

        msg.joint_names.clear();
        msg.points.clear();
        point_msg.positions.clear();
        point_msg.effort.clear();
        point_msg.accelerations.clear();
        point_msg.velocities.clear();



        switch(INPUT_MODE)
        {
            case MANUAL_MODE:

            for (int i = TX90.minLink-1; i < TX90.maxLink; i++)
            {
                msg.joint_names.push_back(TX90.jointNames[i]);
                point_msg.positions.push_back(TX90.currPos[i]);

            }
            point_msg.accelerations.push_back(0);
            point_msg.effort.push_back(0);
            point_msg.velocities.push_back(0);

            msg.points.push_back(point_msg);
            jointStatesTransmitReady = true;
            // if (! --pub_counter)
            // {
            //     commander_pub.publish(msg);
            //     pub_counter = LOOP_RATE_INT / 10 ;
            // }

            
            break;

            case WP_MODE:
                if (goalExists)
                {
                    msg.joint_names.clear();
                    point_msg.positions.clear();

                    for (int i = 0; i < 6; i++)
                    {
                        if (TX90.currPos[i] != goalArray[i])
                        {
                            if (goalArray[i] > TX90.currPos[i])
                            {
                                if ( -(TX90.currPos[i] - goalArray[i]) > 1/LOOP_RATE_INT*TX90.maxVelocity[i]*TX90.currVelocity)
                                    TX90.currPos[i] += 1/LOOP_RATE_INT*TX90.maxVelocity[i]*TX90.currVelocity;
                                else
                                    TX90.currPos[i] += goalArray[i] - TX90.currPos[i];
                            }
                                
                            else if (goalArray[i] < TX90.currPos[i])
                            {
                                if ((TX90.currPos[i] - goalArray[i]) > 1/LOOP_RATE_INT*TX90.maxVelocity[i]*TX90.currVelocity)
                                    TX90.currPos[i] -= 1/LOOP_RATE_INT*TX90.maxVelocity[i]*TX90.currVelocity;
                                else
                                    TX90.currPos[i] -= TX90.currPos[i] - goalArray[i];
                            }

                            msg.joint_names.push_back(TX90.jointNames[i]);
                            point_msg.positions.push_back(TX90.currPos[i]);
                            // point_msg.time_from_start = ros::Duration(1);

                        }
                        else
                        {
                            jointsdone |= (1 << i);
                        }
                    }
                    msg.points.push_back(point_msg);
                    jointStatesTransmitReady = true;


                    if (jointsdone == 63)
                    {
                        ROS_INFO("Reached %s",wpInputMsg.programName.c_str());
                        goalExists = false;
                        commanderFeedbackMsg.flags = WAYPOINT_REACHED;
                        fbTransmitReady = true;
                        jointsdone = 0;
                    }
                }
            break;







            case TEST_MODE:
            if (not_run)
            {
                for(int i = 0; i < 6; i++)
                {
                    msg.joint_names.push_back(TX90.jointNames[i]);
                    if (i != 0)
                        point_msg.positions.push_back(0);
                    else
                        point_msg.positions.push_back(0);
                    
                    point_msg.velocities.push_back(1);
                    point_msg.accelerations.push_back(0);
                }


                point_msg.effort.push_back(0);
                point_msg.time_from_start = ros::Duration(0.5);
                msg.points.push_back(point_msg);
                jointStatesTransmitReady = true;
                not_run = false;
            }
            break;










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

        if (INPUT_MODE == MANUAL_MODE || INPUT_MODE == WP_MODE)
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
            commander_pub.publish(msg);
            jointStatesTransmitReady = false;
        }
        
        if (fbTransmitReady)
        {
            commander_fb_pub.publish(commanderFeedbackMsg);
            fbTransmitReady = false;
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
        for (int i = 0; i < 6; i++)
            manualInputMsg.jointVelocity[i] = manual->jointVelocity[i];

        for (int i = TX90.minLink-1; i < TX90.maxLink; i++)
        {
            if (manualInputMsg.jointVelocity[i] > 0)
            {
                if (TX90.currPos[i] + (1/LOOP_RATE_INT * manualInputMsg.jointVelocity[i]*TX90.maxVelocity[i]*TX90.currVelocity) < TX90.maxRotation[i])
                    TX90.currPos[i] += (1/LOOP_RATE_INT)*(manualInputMsg.jointVelocity[i])*TX90.maxVelocity[i]*TX90.currVelocity;
            }
            
            else if (manualInputMsg.jointVelocity[i] < 0)
            {
                if (TX90.currPos[i] - (1/LOOP_RATE_INT * manualInputMsg.jointVelocity[i]*TX90.maxVelocity[i]*TX90.currVelocity) > TX90.minRotation[i])
                    TX90.currPos[i] += (1/LOOP_RATE_INT)*(manualInputMsg.jointVelocity[i])*TX90.maxVelocity[i]*TX90.currVelocity;
            }   
        }
    }
}

 /* ----------------------------------------------------------------------
 *                -------  Waypoint Callback function   -------
 * ----------------------------------------------------------------------- */ 
void wpCallback     (const bitten::control_msg::ConstPtr& wp)
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
                ROS_INFO("Terminated connection to %s", wpInputMsg.nodeName.c_str());
            }
        break;

        case NEW_WAYPOINT:
            if (INPUT_MODE == WP_MODE)
            {
                if (goalExists == false)
                {
                    wpInputMsg.programName = wp->programName;
                    ROS_INFO("Setting new goal to: %s",wp->programName.c_str());
                    goalExists = true;
                    for (int i = 0; i < 6; i++)
                        goalArray[i] = wp ->jointPosition[i];
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
void testCallback   (const bitten::control_msg::ConstPtr& test)
{
    /*
     * MANDAG: Snak om hvordan TEST beskeden ser ud. én TEST besked med flere waypoints?
     * */

}


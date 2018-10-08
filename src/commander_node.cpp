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
#include <array>
#include <sstream>
#include <iostream>
#include <stdio.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "commander_node.h"
#include "global_node_definitions.h"
#include "bitten/control_msg.h"


 /* ----------------------------------------------------------------------
 *                      -------  Initializing   -------
 * ----------------------------------------------------------------------- */
void InitRobot();
// void manualCallback(const beginner_tutorials::control_msg::ConstPtr& manual)

void manualCallback (const bitten::control_msg::ConstPtr& manual);
void wpCallback     (/*const beginner_tutorials::FBmsgType_s::ConstPtr& wp*/);
void testCallback   (/*const beginner_tutorials::FBmsgType_s::ConstPtr& test*/);

 /* ----------------------------------------------------------------------
 *                    -------  Message objects   -------
 * ----------------------------------------------------------------------- */
MsgType_s manualInputMsg;               /* Used to collect data from:       manual node                                         */
MsgType_s wpInputMsg;                   /* Used to collect data from:       waypoint node                                       */
MsgType_s testInputMsg;                 /* Used to collect data from:       test node                                           */

 /* ----------------------------------------------------------------------
 *                          -------  Main   -------
 * ----------------------------------------------------------------------- */        
int main(int argc , char **argv)
{
    ROS_INFO("Initiating system...");
    InitRobot();

    ros::init(argc , argv , "commander_node");
    ros::NodeHandle n;
    ROS_INFO("Subscribing to \"manual_topic\"");
    ros::Subscriber manual_sub      = n.subscribe<bitten::control_msg>("manual_topic" , 3*LOOP_RATE_INT , &manualCallback);
    
    ROS_INFO("Subscribing to \"wp_topic\"");
    // ros::Subscriber wp_sub          = n.subscribe<bitten::control_msg>         ("wp_topic"     , 3*LOOP_RATE_INT , &wpCallback);
    ROS_INFO("Subscribing to \"test_topic\"");
    // ros::Subscriber test_sub        = n.subscribe<bitten::control_msg>         ("test_topic"   , 3*LOOP_RATE_INT , &testCallback);

    ROS_INFO("Publishing on \"joint_states\" topic");
    ros::Publisher commander_pub    = n.advertise<sensor_msgs::JointState>  ("joint_states" , 3*LOOP_RATE_INT);
    ros::Rate loop_rate(LOOP_RATE_INT);

    sensor_msgs::JointState msg;
    CONTROL_MODE = WAYPOINT_MODE;

    ros::spinOnce();
/*  -------------------------------------------------
         SUPERLOOP
    ------------------------------------------------- */
    while(ros::ok())
    {

        msg.name.clear();
        msg.position.clear();


        for (int i = TX90.minLink-1; i < TX90.maxLink; i++)
        {
            msg.name.push_back(TX90.jointNames[i]);
            msg.position.push_back(TX90.currPos[i]);
        }

        commander_pub.publish(msg);
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

 /* ----------------------------------------------------------------------
 *                -------  Waypoint Callback function   -------
 * ----------------------------------------------------------------------- */ 
void wpCallback     (/* */ )
{
    /*
     * MANDAG: Snak om hvordan WP beskeden ser ud. én WP besked med ønsket position og hastighed som skal afvikles, før næste læses?
     * */



}

 /* ----------------------------------------------------------------------
 *                  -------  test Callback function   -------
 * ----------------------------------------------------------------------- */ 
void testCallback   (/*const beginner_tutorials::FBmsgType::ConstPtr& test_topic*/)
{
    /*
     * MANDAG: Snak om hvordan TEST beskeden ser ud. én TEST besked med flere waypoints?
     * */

}
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
 *                       -------  Constants   -------
 * ----------------------------------------------------------------------- */
const double loop_rate_int    = 50;
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
    ros::Subscriber manual_sub      = n.subscribe<bitten::control_msg>         ("manual_topic" , 3*loop_rate_int , &manualCallback);
    ROS_INFO("Subscribing to \"wp_topic\"");
    // ros::Subscriber wp_sub          = n.subscribe<bitten::control_msg>         ("wp_topic"     , 3*loop_rate_int , &wpCallback);
    ROS_INFO("Subscribing to \"test_topic\"");
    // ros::Subscriber test_sub        = n.subscribe<bitten::control_msg>         ("test_topic"   , 3*loop_rate_int , &testCallback);

    ROS_INFO("Publishing on \"joint_states\" topic");
    ros::Publisher commander_pub    = n.advertise<sensor_msgs::JointState>  ("joint_states" , 3*loop_rate_int);

    ROS_INFO("Looping at: %d Hz" , loop_rate_int);
    ros::Rate loop_rate(loop_rate_int);


    sensor_msgs::JointState msg;
    CONTROL_MODE = WAYPOINT_MODE;
    ROS_INFO("Entering Superloop!");
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

    TX90.currVelocity   = 0.5;
    
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
    for (int i = TX90.minLink-1; i < TX90.maxLink; i++)
    {
        if (manualInputMsg.jointVelocity[i] > 0)
        {
            if (TX90.currPos[i] + (1/loop_rate_int * manualInputMsg.jointVelocity[i])*TX90.currVelocity < TX90.maxRotation[i])
                TX90.currPos[i] += (1/loop_rate_int * manualInputMsg.jointVelocity[i])*TX90.currVelocity;
        }
          
        else if (manualInputMsg.jointVelocity[i] < 0)
        {
            if (TX90.currPos[i] - (1/loop_rate_int * -manualInputMsg.jointVelocity[i])*TX90.currVelocity > TX90.minRotation[i])
                TX90.currPos[i] -= (1/loop_rate_int * -manualInputMsg.jointVelocity[i])*TX90.currVelocity;
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

// Jeg tilføjede noget!

// jeg tilføjede noget igen!

// Jeg tilføjede noget igen, igen!


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

 /* ----------------------------------------------------------------------
 *                      -------  Initializing   -------
 * ----------------------------------------------------------------------- */
void InitRobot();
// void manualCallback(const beginner_tutorials::control_Msg::ConstPtr& manual)

void manualCallback (/*const beginner_tutorials::FBMsgType_s::ConstPtr& manual */);
void wpCallback     (/*const beginner_tutorials::FBMsgType_s::ConstPtr& wp*/);
void testCallback   (/*const beginner_tutorials::FBMsgType_s::ConstPtr& test*/);

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
    // ros::Subscriber manual_sub      = n.subscribe<ManualInputMsg>         ("manual_topic" , 3*loop_rate_int , &manualCallback);
    ROS_INFO("Subscribing to \"wp_topic\"");
    // ros::Subscriber wp_sub          = n.subscribe<ManualInputMsg>         ("wp_topic"     , 3*loop_rate_int , &wpCallback);
    ROS_INFO("Subscribing to \"test_topic\"");
    // ros::Subscriber test_sub        = n.subscribe<ManualInputMsg>         ("test_topic"   , 3*loop_rate_int , &testCallback);

    ROS_INFO("Publishing on \"joint_states\" topic");
    ros::Publisher commander_pub    = n.advertise<sensor_msgs::JointState>  ("joint_states" , 3*loop_rate_int);

    ROS_INFO("Looping at: %d Hz" , loop_rate_int);
    ros::Rate loop_rate(loop_rate_int);


    sensor_msgs::JointState Msg;
    CONTROL_MODE = WAYPOINT_MODE;
    ROS_INFO("Entering Superloop!");
/*  -------------------------------------------------
         SUPERLOOP
    ------------------------------------------------- */
    while(ros::ok())
    {
        for (int i = TX90.MinLink-1; i < TX90.MaxLink; i++)
        {
            Msg.name.push_back(TX90.JointNames[i]);
            Msg.position.push_back(TX90.CurrPos[i]);
        }

        commander_pub.publish(Msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

 /* ----------------------------------------------------------------------
 *                 -------  Initialize TX90 Robot   -------
 * ----------------------------------------------------------------------- */       
void InitRobot()
{
    TX90.MinLink    = 1;
    TX90.MaxLink    = 6;
    TX90.ResetStatePosition = {0 , 0 , 0 , 0 , 0 , 0};

    TX90.CurrVelocity   = 0.5;
    
    TX90.MaxRotation = {    3.14,               /* joint_1  */
                            2.57,               /* joint_2  */
                            2.53,               /* joint_3  */
                            4.71,               /* joint_4  */
                            2.44,               /* joint_5  */
                            4.71};              /* joint_6  */

    TX90.MinRotation = {    -3.14,              /* joint_1  */
                            -2.27,              /* joint_2  */
                            -2.53,              /* joint_3  */
                            -4.71,              /* joint_4  */
                            -2.01,              /* joint_5  */
                            -4.71};             /* joint_6  */ 

    TX90.MaxVelocity = {    (400/180)*3.14,     /* joint_1  */
                            (400/180)*3.14,     /* joint_2  */
                            (430/180)*3.14,     /* joint_3  */
                            (540/180)*3.14,     /* joint_4  */
                            (475/180)*3.14,     /* joint_5  */
                            (760/180)*3.14};    /* joint_6  */                   

    TX90.MaxEffort  =   {   318,                /* joint_1  */
                            166,                /* joint_2  */
                            76,                 /* joint_3  */
                            34,                 /* joint_4  */
                            29,                 /* joint_5  */
                            11};                /* joint_6  */
                            
    TX90.CurrPos    = { 0 , 0 , 0 , 0 , 0 , 0};

    TX90.JointNames = { "joint_1", 
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
void manualCallback (/*const beginner_tutorials::FBMsgType::ConstPtr& manual_topic*/)
{   
    for (int i = TX90.MinLink-1; i < TX90.MaxLink; i++)
    {
        if (ManualInputMsg.JointVelocity[i] > 0)
        {
            if (TX90.CurrPos[i] + (1/loop_rate_int * ManualInputMsg.JointVelocity[i])*TX90.CurrVelocity < TX90.MaxRotation[i])
                TX90.CurrPos[i] += (1/loop_rate_int * ManualInputMsg.JointVelocity[i])*TX90.CurrVelocity;
        }
          
        else if (ManualInputMsg.JointVelocity[i] < 0)
        {
            if (TX90.CurrPos[i] - (1/loop_rate_int * -ManualInputMsg.JointVelocity[i])*TX90.CurrVelocity > TX90.MinRotation[i])
                TX90.CurrPos[i] -= (1/loop_rate_int * -ManualInputMsg.JointVelocity[i])*TX90.CurrVelocity;
        }   
    }
}

 /* ----------------------------------------------------------------------
 *                -------  Waypoint Callback function   -------
 * ----------------------------------------------------------------------- */ 
void wpCallback     (/*const beginner_tutorials::FBMsgType::ConstPtr& wp_topic*/)
{
    /*
     * MANDAG: Snak om hvordan WP beskeden ser ud. én WP besked med ønsket position og hastighed som skal afvikles, før næste læses?
     * */
}

 /* ----------------------------------------------------------------------
 *                  -------  test Callback function   -------
 * ----------------------------------------------------------------------- */ 
void testCallback   (/*const beginner_tutorials::FBMsgType::ConstPtr& test_topic*/)
{
    /*
     * MANDAG: Snak om hvordan TEST beskeden ser ud. én TEST besked med flere waypoints?
     * */

}
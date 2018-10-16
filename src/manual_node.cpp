/* -----------------------------------------------------------------------
 * Filename: manual_node.cpp
 * Author: Kasper Jørgensen
 * Purpose: Create the 'manual' node in the Leica Robot system
 * Date of dev.: September 2018 - January 2019
 * 
 * -----------------------------------------------------------------------
 *                      -------  Libraries   -------
 * ----------------------------------------------------------------------- */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <bitten/control_msg.h>
#include <bitten/feedback_msg.h>
#include <manual_node.h>
#include <global_node_definitions.h>

 /* ----------------------------------------------------------------------
 *                          -------  Main   -------
 * ----------------------------------------------------------------------- */        
int main(int argc, char **argv)
{
    ROS_INFO("Initiating system...");
    ros::init(argc, argv, "manual_node");
    ros::NodeHandle n;

    ROS_INFO("Subscribing to \"%s\"", topicNames[JOY_TOPIC].c_str());
    ros::Subscriber joySub = n.subscribe<sensor_msgs::Joy>("joy", 1000, &joyCallback);
    
    ROS_INFO("Subscribing to %s",topicNames[FEEDBACK_TOPIC].c_str());
    ros::Subscriber feedbackSub = n.subscribe<bitten::feedback_msg>(topicNames[FEEDBACK_TOPIC], 3*LOOP_RATE_INT, &fbCallbackManual);
    if (feedbackSub)
        ROS_INFO("Subscribed to \"%s\"!", topicNames[FEEDBACK_TOPIC].c_str());
    else
        ROS_INFO("Couldn't subscribe to \"%s\".",topicNames[FEEDBACK_TOPIC].c_str());



    ROS_INFO("Publishing to \"%s\"", topicNames[MANUAL_TOPIC].c_str());
    ros::Publisher manualPub = n.advertise<bitten::control_msg>("manual_topic", 1000);

    manual_msg.nodeName = "manual node";

    ros::Rate loop_rate(LOOP_RATE_INT);
    /* -------------------------------------------------
    *     SUPERLOOP
    * ------------------------------------------------- */
    while (ros::ok())
    {
        if (connectionEstablished == true)
        {
            if (newConnection)
            {
                ROS_INFO("Connected!");
                newConnection = false;
            }

            if((manual_msg.jointVelocity[0] == 1) || (manual_msg.jointVelocity[0] == -1))
            {
                if((manual_msg.jointVelocity[2] >= -0.3) && (manual_msg.jointVelocity[2] <= 0.3))
                    manual_msg.jointVelocity[2] = 0;
            }
            if((manual_msg.jointVelocity[2] == 1) || (manual_msg.jointVelocity[2] == -1))
            {
                if((manual_msg.jointVelocity[0] >= -0.3) && (manual_msg.jointVelocity[0] <= 0.3))
                    manual_msg.jointVelocity[0] = 0;
            }
            if((manual_msg.jointVelocity[1] == 1) || (manual_msg.jointVelocity[1] == -1))
            {
                if((manual_msg.jointVelocity[4] >= -0.3) && (manual_msg.jointVelocity[4] <= 0.3))
                    manual_msg.jointVelocity[4] = 0;
            }
            if((manual_msg.jointVelocity[4] == 1) || (manual_msg.jointVelocity[4] == -1))
            {
                if((manual_msg.jointVelocity[1] >= -0.3) && (manual_msg.jointVelocity[1] <= 0.3))
                    manual_msg.jointVelocity[1] = 0;
            }

            transmitManualRdy = true;
        }
        else
        {
            static int timer = LOOP_RATE_INT;
            if (! --timer)
            {
                ROS_INFO("Trying to establish connection...");
                manual_msg.flags = ESTABLISH_CONNECTION;
                timer = LOOP_RATE_INT;
                transmitManualRdy = true;
            }
        }

        if (transmitManualRdy == true)
        {
            manualPub.publish(manual_msg);
            transmitManualRdy = false;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

/* ----------------------------------------------------------------------
 *                -------  Joy Callback function   -------
 * ----------------------------------------------------------------------- */ 

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    //joint 1
    manual_msg.jointVelocity[0] = joy->axes[0];
    //joint 2
    manual_msg.jointVelocity[1] = joy->axes[4];
    //joint 3
    manual_msg.jointVelocity[2] = joy->axes[1];
    //joint 4
    if(joy->buttons[15])
        manual_msg.jointVelocity[3] = -1;
    else if(joy->buttons[16])
        manual_msg.jointVelocity[3] = 1;
    else
        manual_msg.jointVelocity[3] = 0;
    //joint 5
    manual_msg.jointVelocity[4] = joy->axes[3];
    //joint 6
    if(joy->buttons[1])
        manual_msg.jointVelocity[5] = 1;
    else if(joy->buttons[3])
        manual_msg.jointVelocity[5] = -1;
    else
        manual_msg.jointVelocity[5] = 0;
    
    std::cout << "Joint 1: " << manual_msg.jointVelocity[0] << std::endl
            << "Joint 2: " << manual_msg.jointVelocity[1] << std::endl
            << "Joint 3: " << manual_msg.jointVelocity[2] << std::endl
            << "Joint 4: " << manual_msg.jointVelocity[3] << std::endl
            << "Joint 5: " << manual_msg.jointVelocity[4] << std::endl
            << "Joint 6: " << manual_msg.jointVelocity[5] << std::endl << std::endl;

    
}





/* ----------------------------------------------------------------------
 *                -------  Feedback Callback function   -------
 * ----------------------------------------------------------------------- */ 


void fbCallbackManual(const bitten::feedback_msg::ConstPtr& feedbackManual)
{
    if (feedbackManual->recID == MANUAL_ID)
    {
        if (feedbackManual->flags & PING)
        {
            manual_msg.flags = PONG;
            transmitManualRdy = true;
        }


        if (connectionEstablished == false && feedbackManual->flags & ACK)
            connectionEstablished = true;

        else if (connectionEstablished == false && feedbackManual->flags == DENIED)
            ROS_INFO("Connection denied");

    }
}

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
#include <bitten/canopen_msg.h>
#include <global_node_definitions.h>
#include <manual_node.h>

 /* ----------------------------------------------------------------------
 *                          -------  Main   -------
 * ----------------------------------------------------------------------- */        
int main(int argc, char **argv)
{
    ROS_INFO("Initiating node");
    ros::init(argc, argv, "manual_node");
    ros::NodeHandle n;

    ROS_INFO("Subscribing to \"%s\"", topicNames[JOY_TOPIC].c_str());
    ros::Subscriber joySub = n.subscribe<sensor_msgs::Joy>("joy", 1000, &joyCallback);
    
    ROS_INFO("Subscribing to can_topic");
    ros::Subscriber canSub = n.subscribe<bitten::canopen_msg>("can_topic", 1000, &canCallback);

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
                manual_msg.flags |= ESTABLISH_CONNECTION;
                timer = LOOP_RATE_INT;
                transmitManualRdy = true;
            }
        }

        if (transmitManualRdy == true)
        {
            manualPub.publish(manual_msg);
            manual_msg.flags = 0;
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
    
    for(int i = 0; i < 6; i++)
    {
        ROS_INFO("Joint %d: %f", i+1, manual_msg.jointVelocity[i]);
    }
    std::cout << std::endl;
}

/* ----------------------------------------------------------------------
 *                -------  Feedback Callback function   -------
 * ----------------------------------------------------------------------- */
void canCallback(const bitten::canopen_msg::ConstPtr& can)
{
    switch(can->can_id)
    {
    case 0x8CFDD633:
    {
        int check = can->data[0] % 16;
        if(check == 4)
        {
            float res = can->data[1] * 0xFF + can->data[0];
            res = -(res / float(0xFA * 0xFF + 0x04));
            ROS_INFO("0xCFDD633 %f", res);
            manual_msg.jointVelocity[4] = res;
        }
        else if(check == 0) 
        {
            float res = can->data[1] * 0xFF + can->data[0];
            res = res / float(0xFA * 0xFF + 0x10);
            ROS_INFO("0xCFDD633 %f", res);
            manual_msg.jointVelocity[4] = res;
        }
        else
        {
            float res = 0;
            ROS_INFO("0xCFDD633 %f", res);
            manual_msg.jointVelocity[4] = res;
        }
        
        check = can->data[2] % 16;
        if(check == 4)
        {
            float res = can->data[3] * 0xFF + can->data[2];
            res = -(res / float(0xFA * 0xFF + 0x04));
            ROS_INFO("0xCFDD634 %f", res);
            manual_msg.jointVelocity[1] = res;
        }
        else if(check == 0) 
        {
            float res = can->data[3] * 0xFF + can->data[2];
            res = res / float(0xFA * 0xFF + 0x10);
            ROS_INFO("0xCFDD634 %f", res);
            manual_msg.jointVelocity[1] = res;
        }
        else
        {
            float res = 0;
            ROS_INFO("0xCFDD634 %f", res);
            manual_msg.jointVelocity[1] = res;
        }
        break;
    }
    case 0x8CFDD634:
    {
        int check = can->data[0] % 16;
        if(check == 4)
        {
            float res = can->data[1] * 0xFF + can->data[0];
            res = -(res / float(0xFA * 0xFF + 0x04));
            ROS_INFO("0xCFDD634 %f", res);
            manual_msg.jointVelocity[0] = res;
        }
        else if(check == 0) 
        {
            float res = can->data[1] * 0xFF + can->data[0];
            res = res / float(0xFA * 0xFF + 0x10);
            ROS_INFO("0xCFDD634 %f", res);
            manual_msg.jointVelocity[0] = res;
        }
        else
        {
            float res = 0;
            ROS_INFO("0xCFDD634 %f", res);
            manual_msg.jointVelocity[0] = res;
        }

        check = can->data[2] % 16;
        if(check == 4)
        {
            float res = can->data[3] * 0xFF + can->data[2];
            res = -(res / float(0xFA * 0xFF + 0x04));
            ROS_INFO("0xCFDD634 %f", res);
            manual_msg.jointVelocity[2] = res;
        }
        else if(check == 0) 
        {
            float res = can->data[3] * 0xFF + can->data[2];
            res = res / float(0xFA * 0xFF + 0x10);
            ROS_INFO("0xCFDD634 %f", res);
            manual_msg.jointVelocity[2] = res;
        }
        else
        {
            float res = 0;
            ROS_INFO("0xCFDD634 %f", res);
            manual_msg.jointVelocity[2] = res;
        }
        break;
    }
    case 0x8CFDD733:
    {
        int check = can->data[0] % 16;
        if(check == 4)
        {
            float res = can->data[1] * 0xFF + can->data[0];
            res = -(res / float(0xFA * 0xFF + 0x04));
            ROS_INFO("0xCFDD733 %f", res);
            manual_msg.jointVelocity[5] = res;
        }
        else if(check == 0) 
        {
            float res = can->data[1] * 0xFF + can->data[0];
            res = res / float(0xFA * 0xFF + 0x10);
            ROS_INFO("0xCFDD733 %f", res);
            manual_msg.jointVelocity[5] = res;
        }
        else
        {
            float res = 0;
            ROS_INFO("0xCFDD733 %f", res);
            manual_msg.jointVelocity[5] = res;
        }
        break;
    }
    case 0x8CFDD734:
    {
        int check = can->data[0] % 16;
        if(check == 4)
        {
            float res = can->data[1] * 0xFF + can->data[0];
            res = -(res / float(0xFA * 0xFF + 0x04));
            ROS_INFO("0xCFDD734 %f", res);
            manual_msg.jointVelocity[3] = res;
        }
        else if(check == 0) 
        {
            float res = can->data[1] * 0xFF + can->data[0];
            res = res / float(0xFA * 0xFF + 0x10);
            ROS_INFO("0xCFDD734 %f", res);
            manual_msg.jointVelocity[3] = res;
        }
        else
        {
            float res = 0;
            ROS_INFO("0xCFDD734 %f", res);
            manual_msg.jointVelocity[3] = res;
        }
        break;
    }
    default:
        break; 
    }
    ROS_INFO(" ");
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
            manual_msg.flags |= PONG;
            transmitManualRdy = true;
        }


        if (connectionEstablished == false && feedbackManual->flags & ACK)
        {
            connectionEstablished = true;
        }
            

        else if (connectionEstablished == false && feedbackManual->flags == DENIED)
            ROS_INFO("Connection denied");

    }
}

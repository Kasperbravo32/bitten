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
#include <bitten/control_msg.h>
#include <bitten/canopen_msg.h>
#include <bitten/feedback_msg.h>
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
    
    ROS_INFO("Subscribing to \"can_topic\"");
    ros::Subscriber canSub = n.subscribe<bitten::canopen_msg>("can_topic", 1000, &canCallback);

    ROS_INFO("Subscribing to \"%s\"",topicNames[FEEDBACK_TOPIC].c_str());
    ros::Subscriber feedbackSub = n.subscribe<bitten::feedback_msg>(topicNames[FEEDBACK_TOPIC], 3*LOOP_RATE_INT, &fbCallback);
    if (feedbackSub)
        ROS_INFO("Subscribed to \"%s\"!", topicNames[FEEDBACK_TOPIC].c_str());
    else
        ROS_INFO("Couldn't subscribe to \"%s\".",topicNames[FEEDBACK_TOPIC].c_str());

    ROS_INFO("Publishing to \"%s\"", topicNames[MANUAL_TOPIC].c_str());
    ros::Publisher manualPub = n.advertise<bitten::control_msg>("manual_topic", 1000);

    manual_msg.nodeName = "manual node";
    manual_msg.id = MANUAL_ID;
    
    ros::Rate loop_rate(LOOP_RATE_INT);
    bool newConnection = true;

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
}

/* ----------------------------------------------------------------------
 *                -------  Feedback Callback function   -------
 * ----------------------------------------------------------------------- */
void canCallback(const bitten::canopen_msg::ConstPtr& can)
{
    switch(can->can_id)
    {
    case 0x8CFDD633: //right joystick, basic message
    {
        float actual_pos;
        float max_pos;
        if(can->data[0] & 0x04) //tilt left
        {
            actual_pos = can->data[1] * 0xFF + can->data[0];
            max_pos = 0xFA * 0xFF + 0x04;
            manual_msg.jointVelocity[4] = -(actual_pos / max_pos);
        }
        else if(can->data[0] & 0x10) //tilt right
        {
            actual_pos = can->data[1] * 0xFF + can->data[0];
            max_pos = 0xFA * 0xFF + 0x10;
            manual_msg.jointVelocity[4] = actual_pos / max_pos;
        }
        else
            manual_msg.jointVelocity[4] = 0;
        
        if(can->data[2] & 0x04) //tilt backwards
        {
            actual_pos = can->data[3] * 0xFF + can->data[2];
            max_pos = 0xFA * 0xFF + 0x04;
            manual_msg.jointVelocity[1] = -(actual_pos / max_pos);
        }
        else if(can->data[2] & 0x10) //tilt forwards
        {
            actual_pos = can->data[3] * 0xFF + can->data[2];
            max_pos = 0xFA * 0xFF + 0x10;
            manual_msg.jointVelocity[1] = actual_pos / max_pos;
        }
        else
            manual_msg.jointVelocity[1] = 0;

        //push buttons
        if(can->data[5] & 0x04)
            manual_msg.buttons[0] = 1;
        else
            manual_msg.buttons[0] = 0;

        if(can->data[5] & 0x10)
            manual_msg.buttons[1] = 1;
        else
            manual_msg.buttons[1] = 0;

        if(can->data[6] & 0x01)
            manual_msg.buttons[2] = 1;
        else
            manual_msg.buttons[2] = 0;

        if(can->data[6] & 0x04)
            manual_msg.buttons[3] = 1;
        else
            manual_msg.buttons[3] = 0;

        if(can->data[7] & 0x10)
            manual_msg.buttons[4] = 1;
        else
            manual_msg.buttons[4] = 0;
        
        if(can->data[7] & 0x04)
            manual_msg.buttons[5] = 1;
        else
            manual_msg.buttons[5] = 0;

        break;
    }
    case 0x8CFDD634: //left joystick, basic message
    {
        float actual_pos;
        float max_pos;
        if(can->data[0] & 0x04) //tilt left
        {
            actual_pos = can->data[1] * 0xFF + can->data[0];
            max_pos = 0xFA * 0xFF + 0x04;
            manual_msg.jointVelocity[0] = -(actual_pos / max_pos);
        }
        else if(can->data[0] & 0x10) //tilt right
        {
            actual_pos = can->data[1] * 0xFF + can->data[0];
            max_pos = 0xFA * 0xFF + 0x10;
            manual_msg.jointVelocity[0] = actual_pos / max_pos;
        }
        else
            manual_msg.jointVelocity[0] = 0;

        if(can->data[2] & 0x04) //tilt backwards
        {
            actual_pos = can->data[3] * 0xFF + can->data[2];
            max_pos = 0xFA * 0xFF + 0x04;
            manual_msg.jointVelocity[2] = -(actual_pos / max_pos);
        }
        else if(can->data[2] & 0x10) //tilt forwards
        {
            actual_pos = can->data[3] * 0xFF + can->data[2];
            max_pos = 0xFA * 0xFF + 0x10;
            manual_msg.jointVelocity[2] = actual_pos / max_pos;
        }
        else
            manual_msg.jointVelocity[2] = 0;

        //push buttons
        if(can->data[5] & 0x04)
            manual_msg.buttons[6] = 1;
        else
            manual_msg.buttons[6] = 0;

        if(can->data[5] & 0x10)
            manual_msg.buttons[7] = 1;
        else
            manual_msg.buttons[7] = 0;

        if(can->data[6] & 0x01)
            manual_msg.buttons[8] = 1;
        else
            manual_msg.buttons[8] = 0;

        if(can->data[6] & 0x04)
            manual_msg.buttons[9] = 1;
        else
            manual_msg.buttons[9] = 0;

        if(can->data[7] & 0x10)
            manual_msg.buttons[10] = 1;
        else
            manual_msg.buttons[10] = 0;
        
        if(can->data[7] & 0x04)
            manual_msg.buttons[11] = 1;
        else
            manual_msg.buttons[11] = 0;

        break;
    }
    case 0x8CFDD733: //right joystick, extended message
    {
        float actual_pos;
        float max_pos;
        if(can->data[0] & 0x04) //roll left
        {
            actual_pos = can->data[1] * 0xFF + can->data[0];
            max_pos = 0xFA * 0xFF + 0x04;
            manual_msg.jointVelocity[5] = -(actual_pos / max_pos);
        }
        else if(can->data[0] & 0x10) //roll right
        {
            actual_pos = can->data[1] * 0xFF + can->data[0];
            max_pos = 0xFA * 0xFF + 0x10;
            manual_msg.jointVelocity[5] = actual_pos / max_pos;
        }
        else
            manual_msg.jointVelocity[5] = 0;

        break;
    }
    case 0x8CFDD734: //left joystick, extended message
    {
        float actual_pos;
        float max_pos;
        if(can->data[0] & 0x04) //roll left
        {
            actual_pos = can->data[1] * 0xFF + can->data[0];
            max_pos = 0xFA * 0xFF + 0x04;
            manual_msg.jointVelocity[3] = -(actual_pos / max_pos);
        }
        else if(can->data[0] & 0x10)  //roll right
        {
            actual_pos = can->data[1] * 0xFF + can->data[0];
            max_pos = 0xFA * 0xFF + 0x10;
            manual_msg.jointVelocity[3] = actual_pos / max_pos;
        }
        else
            manual_msg.jointVelocity[3] = 0;

        break;
    }
    default:
        break; 
    }
}

/* ----------------------------------------------------------------------
 *                -------  Feedback Callback function   -------
 * ----------------------------------------------------------------------- */
void fbCallback(const bitten::feedback_msg::ConstPtr& feedback)
{
    if (feedback->recID == MANUAL_ID)
    {
        if (feedback->flags & PING)
        {
            manual_msg.flags |= PONG;
            transmitManualRdy = true;
        }

        if (connectionEstablished == false && feedback->flags & ACK)
            connectionEstablished = true;            

        else if (connectionEstablished == false && feedback->flags == DENIED)
            ROS_INFO("Connection denied");
    }
}

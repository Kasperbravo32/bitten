/* -----------------------------------------------------------------------
 * Filename: commander_node.cpp
 * Author: Frederik Snedevind & Kasper Banke Jørgensen
 * Purpose: Create the 'Commander' node in the Leica Robot system
 * Date of dev.: September 2018 - January 2019
 * 
 * -----------------------------------------------------------------------
 *                      -------  Libraries   -------
 * ----------------------------------------------------------------------- */
#include <trajectory_msgs/JointTrajectory.h>

#include <array>
#include <sstream>
#include <iostream>
#include <fstream>
#include <stdio.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "movement_node.h"
#include "terminal_node.h"
#include "commander_node.h"
#include "global_node_definitions.h"

#include "bitten/feedback_msg.h"
#include "bitten/control_msg.h"

using namespace std;
 /* ----------------------------------------------------------------------
 *                      -------  Initializing   -------
 * ----------------------------------------------------------------------- */
void manualCallback             (const bitten::control_msg::ConstPtr& manual        );
void wpCallback                 (const bitten::control_msg::ConstPtr& wp            );
void terminalCallback           (const bitten::control_msg::ConstPtr& terminal      );
void movementFeedbackCallback   (const bitten::feedback_msg::ConstPtr& moveFeedback );

 /* ----------------------------------------------------------------------
 *                    -------  Message objects   -------
 * ----------------------------------------------------------------------- */
bitten::feedback_msg commanderFeedbackMsg;     /* Used to send feedback to various nodes */
bitten::control_msg passOnMsg;
 /* ----------------------------------------------------------------------
 *                    -------  Global Variables   -------
 * ----------------------------------------------------------------------- */
bool goalExists                 = false;
bool waiting                    = false;
bool fbTransmitReady            = false;
bool jointStatesTransmitReady   = false;
bool robotOccupied              = false;
bool recording                  = false;

bool not_run                    = true;
bool ping                       = true;

const   int PING_RATE = LOOP_RATE_INT / 2;
        int poll_timer = LOOP_RATE_INT*2;
        int ping_timer = PING_RATE;
        int ping_timeout = PING_RATE;
        int pub_counter = LOOP_RATE_INT / 10;
        int waypointsRecorded = 0;
double goalArray[6];

uint8_t jointsdone = 0;

string currentRecordFile;

fstream RecordFile;
 /* ----------------------------------------------------------------------
 *                          -------  Main   -------
 * ----------------------------------------------------------------------- */        
int main(int argc , char **argv)
{
    ROS_INFO("Initiating %s",nodeNames[COMMANDER_NODE].c_str());
    
    ros::init(argc , argv , "commander_node");
    ros::NodeHandle n;
    commanderFeedbackMsg.senderID = COMMANDER_ID;   

    ros::Subscriber manual_sub          = n.subscribe<bitten::control_msg>  ("manual_topic"     , 5 , &manualCallback);
    ros::Subscriber wp_sub              = n.subscribe<bitten::control_msg>  ("wp_topic"         , 10, &wpCallback);
    ros::Subscriber movement_feedback   = n.subscribe<bitten::feedback_msg> ("movement_feedback", 10, &movementFeedbackCallback);
    ros::Subscriber terminal_sub        = n.subscribe<bitten::control_msg>  ("terminal_topic"   , 5 , &terminalCallback);

    ros::Publisher commander_pub        = n.advertise<bitten::control_msg>  ("movement_topic"   , LOOP_RATE_INT);
    ros::Publisher commander_fb_pub     = n.advertise<bitten::feedback_msg> ("feedback_topic"   , LOOP_RATE_INT);

    ros::Rate loop_rate(LOOP_RATE_INT);
    INPUT_MODE = POLL_MODE;

    passOnMsg.nodeName = nodeNames[COMMANDER_NODE];

    sleep(1);
    if (manual_sub && wp_sub && commander_pub && commander_fb_pub && movement_feedback)
        ROS_INFO("Initiated %s",nodeNames[COMMANDER_NODE].c_str());
    else
        ROS_INFO("Didn't initiate %s",nodeNames[COMMANDER_NODE].c_str());

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
    // extern Robot_s TX90;

    static int debounceCounter = 0;
    static bool buttonClicked = false;

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

    if (manual->buttons[0] == 1)
    {
        debounceCounter++;

        if (debounceCounter == 5)
        {
            if (recording == true)
            {
                cout << "Record button hit. Adding waypoint_" << waypointsRecorded << endl;
                RecordFile << "\nwaypoint_" << waypointsRecorded;
                
                // std::array<double, 6> *tempCurrPos = getCurrPos();
                // double *tempCurrPos = getCurrPos();
                
                for (int i = 0; i < 6; i++)
                {
                    RecordFile << "\t" << TX90.getCurrPos(i);
                    cout << TX90.getCurrPos(i) << "\t";
                }
            }
            waypointsRecorded++;
        }
    }
    else if (manual->buttons[0] == 0)
        debounceCounter = 0;

    if (INPUT_MODE == MANUAL_MODE)
    {
        passOnMsg.buttons = manual->buttons;
        passOnMsg.jointVelocity = manual->jointVelocity;
        passOnMsg.id = MANUAL_ID;
        jointStatesTransmitReady = true; 
    }
}

 /* ----------------------------------------------------------------------
 *              -------  Waypoint Callback function   -------
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
 *         -------  MovementFeedback Callback function   -------
 * ----------------------------------------------------------------------- */
void movementFeedbackCallback (const bitten::feedback_msg::ConstPtr& moveFeedback)
{
    ROS_INFO("GOT SOME FEEDBACK");
    if (moveFeedback->flags & GOAL_REACHED)
    {
        switch(INPUT_MODE)
        {
            case WP_MODE:
                commanderFeedbackMsg.recID = WP_ID;
                commanderFeedbackMsg.flags |= GOAL_REACHED;
                fbTransmitReady = true;
                robotOccupied = false;
            break;

            case MANUAL_MODE:
                robotOccupied = false;
            break;

            
        }
    }
}

 /* ----------------------------------------------------------------------
 *             -------  Terminal Callback function   -------
 * ----------------------------------------------------------------------- */
void terminalCallback (const bitten::control_msg::ConstPtr& terminal)
{
    if (terminal->flags & MODE_MANUAL_F)
    {
        if (goalExists == false)
        {
            if (INPUT_MODE != MANUAL_MODE)
            {
                INPUT_MODE = MANUAL_MODE;
                std::cout << "Operating Mode: Manual mode." << std::endl;
            }            
        }
        else
        {
            std::cout << "Currently occupied. Finish current operation before changing modes" << std::endl;
        }
    }

    if (terminal->flags & MODE_WAYPOINT_F)
    {
        if (INPUT_MODE != WP_MODE)
        {
            INPUT_MODE = WP_MODE;
            std::cout << "Operating Mode: Waypoint mode." << std::endl;
        }
    }

    if (terminal->flags & MODE_NONE_F)
    {
        if (goalExists == false)
        {
            if (INPUT_MODE != POLL_MODE)
            {
                INPUT_MODE == POLL_MODE;
                std::cout << "Operating Mode: No-Control." << std::endl;
            }
        }
        else
            std::cout << "Currently occupied. Finish current operation before changing modes" << std::endl;
    }

    if (terminal->flags & PLAY_TEST_F)
    {
        if (INPUT_MODE == WP_MODE)
        {
            std::cout << "Received request to start test..." << std::endl;
            commanderFeedbackMsg.flags = START_TEST;                        /* Reset flags, and set the START_TEST flag                 */
            commanderFeedbackMsg.programName = terminal->programName;       /* Attach the filename of the test to the feedback message  */
            commanderFeedbackMsg.senderID = COMMANDER_ID;                   /* Set the Sender ID                                        */
            commanderFeedbackMsg.recID = WP_ID;                             /* Set the Receiver ID                                      */
            fbTransmitReady = true;                                         /* We're ready to transmit the message, on feedback topic   */
        }
    }

    if (terminal->flags & START_RECORD)
    {
        if (INPUT_MODE == MANUAL_MODE)
        {
            if (recording == false)
            {
                recording = true;
                currentRecordFile = "/home/frederik/catkin_ws/src/bitten/tests/";
                currentRecordFile += terminal->programName;

                RecordFile.open(currentRecordFile, ios_base::out);

                if (RecordFile.is_open())
                {
                    waypointsRecorded = 0;
                    cout << "Opened recording file!" << std::endl;
                    RecordFile << "test_test";
                }
                else
                    cout << "Couldn't open recording file :( " << std::endl;
            }

        }
    }
    
    if (terminal->flags & STOP_RECORD)
    {
        if (recording == true)
        {
            recording = false;
            RecordFile.close();

        }
    }
}
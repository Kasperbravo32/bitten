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
#include <pwd.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "movement_node.h"
#include "terminal_node.h"
#include "commander_node.h"
#include "global_node_definitions.h"

#include "bitten/feedback_msg.h"
#include "bitten/control_msg.h"

// using namespace std;
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
bool gotPositions               = false;

bool NewWaypoint                = true;
bool not_run                    = true;
bool ping                       = true;

uint8_t ping_flags              = 0;
bool ping_bools[3]              = { false,
                                    false,
                                    false};

const   int PING_RATE = LOOP_RATE_INT / 2;
        int poll_timer = LOOP_RATE_INT*2;
        int ping_timer = PING_RATE;
        int ping_timeout = PING_RATE;
        int pub_counter = LOOP_RATE_INT / 10;
        int waypointsRecorded = 0;
double goalArray[6];

double tempCurrPos[6];

uint8_t jointsdone = 0;

std::string currentRecordFile;

std::fstream RecordFile;

TX90_c TX90;

passwd* pw = getpwuid(getuid());
std::string path(pw->pw_dir);

std::string testsPath        = path + "/catkin_ws/src/bitten/tests/";

 /* ----------------------------------------------------------------------
 *                          -------  Main   -------
 * ----------------------------------------------------------------------- */        
int main(int argc , char **argv)
{    
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
        // switch(INPUT_MODE)
        // {
        //     case MANUAL_MODE:
        //         break;

        //     case WP_MODE:

        //         break;

        //     case POLL_MODE:

        //         if (! --poll_timer)
        //             poll_timer = 2*LOOP_RATE_INT;
        //         break;

        //     default:

        //         break;
        // }
        if (! --ping_timer)
        {
            /* We're using a bit bank for maintaining ping status of each node.
                * We're tracking ping on 3 nodes:
                *  manual_node
                *  terminal_node
                *  waypoint_node
                * manual_nodes ping state is on bit 0 of ping_flags variable
                * terminal_node ping state on bit 1
                * waypoint_node ping state on bit 2
                * The remaining flags are reserved for future use (there's probably no future use): */
            commanderFeedbackMsg.flags = PING;
            fbTransmitReady = true;

            if (ping_flags & 0b111)
            {
                ping_flags = 0;
                ping_timer = PING_RATE;
            }
            else
            {
                for (int i = 0; i < 3; i++)
                {
                    if (! ping_flags & (1 << i))
                    {
                        std::cout << "Ping timed out on node: ";
                        switch(i)
                        {
                            case 0:
                                std::cout << "Manual_node." << std::endl;
                            break;
                            
                            case 1:
                                std::cout << "Terminal_node." << std::endl;
                            break;

                            case 2:
                                std::cout << "Waypoint_node." << std::endl;
                            break;
                        }
                    }
                }
                    
                ROS_INFO("Ping timeout, returning to poll_mode");
                ping_timer = PING_RATE;
                ping_flags = 7;
            }
        }
        
        if (jointStatesTransmitReady)
        {
            commander_pub.publish(passOnMsg);
            jointStatesTransmitReady = false;
            passOnMsg.flags = 0;
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
    static int debounceCounter = 0;

    if (manual->flags & PONG)
        ping_flags |= (1 << 0);

    if (manual->flags & ESTABLISH_CONNECTION)
    {
        if (INPUT_MODE == POLL_MODE)
        {
            INPUT_MODE = MANUAL_MODE;
            // ROS_INFO("Established connection to %s",nodeNames[MANUAL_NODE].c_str());
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
                if (gotPositions == true)
                {
                    NewWaypoint = true;
                    RecordFile << "\nwaypoint_" << waypointsRecorded;
                    std::cout << "Adding waypoint_" << waypointsRecorded << std::endl;

                    for (int i = 0; i < 6; i++)
                        RecordFile << "\t" << tempCurrPos[i];
                }

                else if (NewWaypoint == true)
                {
                    NewWaypoint = false;
                    passOnMsg.flags |= GET_CURR_POS;
                    jointStatesTransmitReady = true;
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
            }
        break;

        case NEW_WAYPOINT:
        // std::cout << "Moving robot to: " << wp->programName << "...";
            if (INPUT_MODE == WP_MODE)
            {
                
                if (robotOccupied == false)
                {
                    passOnMsg.programName = wp->programName;
                    
                    robotOccupied = true;

                    passOnMsg.jointPosition = wp->jointPosition;
                    passOnMsg.id = WP_ID;
                    jointStatesTransmitReady = true;       
                }   
            }
        break;

        case PONG:
            ping_flags |= (1 << 2);
        break;
    }
}

 /* ----------------------------------------------------------------------
 *         -------  MovementFeedback Callback function   -------
 * ----------------------------------------------------------------------- */
void movementFeedbackCallback (const bitten::feedback_msg::ConstPtr& moveFeedback)
{
    if (moveFeedback->flags & GOAL_REACHED)
    {
        switch(INPUT_MODE)
        {
            case WP_MODE:
                // std::cout << " OK!" << std::endl;
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

    if (moveFeedback->flags & SENT_CURR_POS)
    {
        gotPositions = true;
        for (int i = 0; i < 6; i++)
            tempCurrPos[i] = moveFeedback->positions[i];
    }
    else
        gotPositions = false;
}

 /* ----------------------------------------------------------------------
 *             -------  Terminal Callback function   -------
 * ----------------------------------------------------------------------- */
void terminalCallback (const bitten::control_msg::ConstPtr& terminal)
{
    if (terminal-> flags & PONG)
        ping_flags |= (1 << 1);

    if (terminal->flags & MODE_MANUAL_F)
    {
        if (goalExists == false)
        {
            if (INPUT_MODE != MANUAL_MODE)
                INPUT_MODE = MANUAL_MODE;
        }

        else
            std::cout << "Currently occupied. Finish current operation before changing modes" << std::endl;
    }

    if (terminal->flags & MODE_WAYPOINT_F)
    {
        if (INPUT_MODE != WP_MODE)
            INPUT_MODE = WP_MODE;
    }

    if (terminal->flags & MODE_NONE_F)
    {
        if (goalExists == false)
        {
            if (INPUT_MODE != POLL_MODE)
                INPUT_MODE == POLL_MODE;
        }
        else
            std::cout << "Currently occupied. Finish current operation before changing modes" << std::endl;
    }

    if (terminal->flags & PLAY_TEST_F)
    {
        if (INPUT_MODE == WP_MODE)
        {
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
                currentRecordFile = testsPath + terminal->programName;

                RecordFile.open(currentRecordFile, std::ios_base::out);

                if (RecordFile.is_open())
                {
                    waypointsRecorded = 0;
                    RecordFile << "test_test";
                }
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
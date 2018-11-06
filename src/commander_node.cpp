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
bool fbTransmitReady            = false;
bool jointStatesTransmitReady   = false;
bool robotOccupied              = false;
bool recording                  = false;
bool gotPositions               = false;
bool ping_bools[2]              ={false,
                                  false};

bool NewWaypoint                = true;
bool ping                       = true;

uint8_t ping_flags              = 0;

const   int PING_RATE = LOOP_RATE_INT / 2;
        int ping_timer = PING_RATE;
        int waypointsRecorded = 0;

double tempCurrPos[6];

std::fstream RecordFile;

passwd* pw = getpwuid(getuid());
std::string path(pw->pw_dir);
std::string testsPath        = path + "/catkin_ws/src/bitten/tests/";
std::string currentRecordFile;
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

    if (manual_sub && wp_sub && commander_pub && commander_fb_pub && movement_feedback && terminal_sub)
        ROS_INFO("Initiated %s",nodeNames[COMMANDER_NODE].c_str());
    else
        ROS_INFO("Didn't initiate %s",nodeNames[COMMANDER_NODE].c_str());

/*  -------------------------------------------------
         SUPERLOOP
    ------------------------------------------------- */
    while(ros::ok())
    {
        if (! --ping_timer)
        {
            commanderFeedbackMsg.flags = PING;
            commanderFeedbackMsg.recID = ALL_ID;
            fbTransmitReady = true;

            if (INPUT_MODE == POLL_MODE && ping_flags & 0b10 && ping_bools[0] == true)
                INPUT_MODE = MANUAL_MODE;

            if (ping_flags == 3 && ping_bools[0] == true && ping_bools[1] == true)
            {
                ping_flags = 0;
                ping_timer = PING_RATE;
            }

            else
            {
                for (int i = 0; i < 2; i++)
                {
                    if (ping_flags & (1 << i))
                        ping_bools[i] = true;

                    else
                    {
                        if (ping_bools[i] != false)
                        {
                            std::cout << "Ping timed out on node: ";
                            ping_bools[i] = false;
                            switch(i)
                            {
                                case 0:
                                    std::cout << "Manual_node." << std::endl;
                                    if (INPUT_MODE == MANUAL_MODE)
                                        INPUT_MODE = NO_MODE;
                                break;

                                case 1:
                                    std::cout << "Waypoint_node." << std::endl;
                                break;
                            }
                        }
                    }
                }
                ping_flags = 0;
                ping_timer = PING_RATE;
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
    return 0;
}

 /* ----------------------------------------------------------------------
 *                 -------  manual Callback function   -------
 * ----------------------------------------------------------------------- */       
void manualCallback (const bitten::control_msg::ConstPtr& manual)
{
    static int debounceCounter = 0;

    if (manual->flags & PONG)
        ping_flags += 1;

    if (INPUT_MODE == MANUAL_MODE)
    {
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
    if (wp->flags & PONG)
        ping_flags |= (1 << 1);

    if (wp->flags & NEW_WAYPOINT)
    {
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
    }
    
    if (wp->flags & TEST_DONE_FLAG)
    {
        std::cout << "Completed test. Returning to manual control" << std::endl;
        if (INPUT_MODE == WP_MODE)
        {
            INPUT_MODE = MANUAL_MODE;
            robotOccupied = false;
        }
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
            if (INPUT_MODE != NO_MODE)
                INPUT_MODE = NO_MODE;
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

    if (terminal->flags & GO_HOME_F)
    {
        if (INPUT_MODE != WP_MODE)
        {
            // passOnMsg.senderID = MANUAL_ID;
            passOnMsg.nodeName = nodeNames[MANUAL_NODE];
            passOnMsg.programName = "terminal_input";
            passOnMsg.jointPosition = {0,0,0,0,0,0};
            jointStatesTransmitReady = true;
        }
    }
}
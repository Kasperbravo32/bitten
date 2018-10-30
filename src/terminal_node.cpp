/* -----------------------------------------------------------------------
 * Filename: terminal_node.cpp
 * Author: Frederik Snedevind & Kasper Banke Jørgensen
 * Purpose: Create the 'terminal' node in the Leica Robot system
 * Date of dev.: September 2018 - January 2019
 * 
 * -----------------------------------------------------------------------
 *                      -------  Libraries   -------
 * ----------------------------------------------------------------------- */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <global_node_definitions.h>

#include <iostream>
#include "string.h"

#include "bitten/feedback_msg.h"
#include "bitten/control_msg.h"
#include "terminal_node.h"

/* -----------------------------------------------------------------------
 *                     -------  Initializers  -------
 * ----------------------------------------------------------------------- */
void commanderFeedbackCallback (const bitten::feedback_msg::ConstPtr& commanderFeedbackMsg);

std::string KeywordStrings[NUMBER_OF_KEYWORDS] = {  "help",
                                                    "func1",
                                                    "func2" };

void (* KeywordFunctions[NUMBER_OF_KEYWORDS])( void ) = {   help_function,
                                                            function_1,
                                                            function_2      };

 /* ----------------------------------------------------------------------
 *                    -------  Message objects   -------
 * ----------------------------------------------------------------------- */
bitten::control_msg terminalMsg;

/* ----------------------------------------------------------------------
 *                    -------  Global Variables   -------
 * ----------------------------------------------------------------------- */
bool terminalTxRdy = false;

 /* ----------------------------------------------------------------------
 *                         -------  Main   -------
 * ----------------------------------------------------------------------- */

using namespace std;
int main (int argc , char **argv) 
{
    ROS_INFO("Initiating %s", nodeNames[TERMINAL_NODE].c_str());

    ros::init(argc , argv , "terminal_node");
    ros::NodeHandle n;

    ros::Subscriber feedback_sub    = n.subscribe<bitten::feedback_msg>("feedback_topic" , 5 , commanderFeedbackCallback);
    ros::Publisher terminal_pub     = n.advertise<bitten::control_msg>("terminal_topic" , 5);

    ros::Rate loop_rate(LOOP_RATE_INT);
    sleep(1);

    if (feedback_sub && terminal_pub)
    {
        ROS_INFO("Initiated %s", nodeNames[TERMINAL_NODE].c_str());
        for (int i = 0; i < 100; i++)
            cout << endl;

        cout << "Welcome to the Nice FB Terminal!" << endl;
    }
        
    else
        ROS_INFO("Failed to initiate %s",nodeNames[TERMINAL_NODE].c_str());

    std::string input;
/*  -------------------------------------------------
         SUPERLOOP
    ------------------------------------------------- */
    while(ros::ok())
    {
        cout << endl << "<< ";
        cin >> input;

        for (int i = 0; i < NUMBER_OF_KEYWORDS; i++)
        {
            if (input == KeywordStrings[i])
            {
                KeywordFunctions[i]();

            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}

/* ----------------------------------------------------------------------
 *             -------   commander Feedback Callback   -------
 * ----------------------------------------------------------------------- */
void commanderFeedbackCallback (const bitten::feedback_msg::ConstPtr& commanderFeedbackMsg)
{


}


void help_function() {
    cout << endl << "You can enter the following functions: " << endl;

    for (int i = 0; i < NUMBER_OF_KEYWORDS; i++)
        cout << "- " << KeywordStrings[i] << endl;
}

void function_1() {
    cout << "function1 called";
}

void function_2() {
    cout << "function 2 called";
}


/* -----------------------------------------------------------------------
 * Filename: movement_node.cpp
 * Author: Frederik Snedevind
 * Purpose: Create the 'movement' node in the Leica Robot system
 * Date of dev.: September 2018 - January 2019
 * 
 * -----------------------------------------------------------------------
 *                      -------  Libraries   -------
 * ----------------------------------------------------------------------- */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Joy.h>
#include <bitten/control_msg.h>
#include <bitten/feedback_msg.h>
#include <global_node_definitions.h>

#include "trajectory_msgs/JointTrajectory.h"
#include "bitten/feedback_msg.h"
#include "bitten/control_msg.h"

/* -----------------------------------------------------------------------
 *                     -------  Initializers  -------
 * ----------------------------------------------------------------------- */

 /* ----------------------------------------------------------------------
 *                    -------  Message objects   -------
 * ----------------------------------------------------------------------- */
trajectory_msgs::JointTrajectory        jointPathMsg;
trajectory_msgs::JointTrajectoryPoint   jointPathPointMsg;

/* ----------------------------------------------------------------------
 *                    -------  Global Variables   -------
 * ----------------------------------------------------------------------- */
bool robotInitialized = false;
void InitRobot();

 /* ----------------------------------------------------------------------
 *                         -------  Main   -------
 * ----------------------------------------------------------------------- */        
int main () {

ros::Rate loop_rate(LOOP_RATE_INT);

/*  -------------------------------------------------
         SUPERLOOP
    ------------------------------------------------- */
    while(ros::ok())
    {


        ros::spinOnce();
        loop_rate.sleep();
    }
}
/* ----------------------------------------------------------------------
 *             -------  commander_node Callback function   -------
 * ----------------------------------------------------------------------- */       

/* ----------------------------------------------------------------------
 *              -------  robot_state Callback function   -------
 * ----------------------------------------------------------------------- */       



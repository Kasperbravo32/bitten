 #pragma once
 /* ----------------------------------------------------------------------
 *                      -------  Initializing   -------
 * ----------------------------------------------------------------------- */
void fbCallback(const bitten::feedback_msg::ConstPtr& feedback);

struct Waypoint_s {
    double jointPosition[6];
    double velocity[6];
    double effort[6];
    std::string waypointName;
};

/* ----------------------------------------------------------------------
 *                       -------  Global variables   -------
 * ----------------------------------------------------------------------- */
bitten::control_msg wp_msg;
bitten::feedback_msg fb_msg;

Waypoint_s Waypoint_0;
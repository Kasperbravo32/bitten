 #pragma once

struct Waypoint_s {
    double jointPosition[6];
    double velocity[6];
    double effort[6];
    std::string waypointName;
};

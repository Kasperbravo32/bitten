#pragma once

struct Waypoint_s {
    float jointPositions[6];
    float velocity[6];
    float effort[6];
};

enum CONTROL_MODE {                         
    RESET_MODE,
    BREAK_STUFF_MODE,
    WAYPOINT_MODE,
    TEST_1_MODE,
    TEST_2_MODE
} CONTROL_MODE;


enum INPUT_MODE {
    MANUAL_MODE,
    WP_MODE,
    TEST_MODE,
    POLL_MODE
} INPUT_MODE;




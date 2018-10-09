

struct Robot_s {
    int     links;

    int     minLink;    
    int     maxLink;

    double currVelocity;

    std::array<double,6> resetStatePosition;
    std::array<double,6> maxRotation;
    std::array<double,6> minRotation;
    std::array<double,6> maxVelocity;
    std::array<double,6> maxEffort;
    std::array<double,6> currPos;
    std::array<std::string,6> jointNames;

    bool    tool;
};

struct Waypoint_s {
    double jointPositions[6];
    double velocity[6];
    double effort[6];
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
    TEST_MODE
} INPUT_MODE;


/* Defining robots      */
Robot_s TX90;                             /* Staubli TX90 Robot                                                                   */

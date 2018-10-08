

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

struct MsgType_s
{
    std::string nodeName;                  /* Used to write the name of the waypoint/manual/test-node    */
    std::string programName;               /* Used to see if e.g. test 1 in test_node is running                                   */
    
    uint32_t flags;                         /* Used for ???                                                                         */
    uint8_t id;                             /* Used to see if controlling node is test/waypoint/manual node                         */

    std::array<uint8_t, 6> jointToMove;   /* Contains wanted joints to move. values can be either -1, 0 or 1. -1 for negative direction, 1 for positive, 0 for no action  */
    std::array<float, 6> jointPosition;    /* Contains the wanted positions of each joint. used in waypoint mode, not in manual.   */
    std::array<float, 6> jointVelocity;    /* Percentage of maximum velocity to move joints with                                   */
    
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

/* Defining waypoints   */
Waypoint_s waypoint_0;                      /* Waypoint 0, is origo (0.0.0)                                                         */
Waypoint_s waypointBank[5];                 /* Bank to contain 5 waypoints. only used for tests                                     */

/* Defining messages    */
MsgType_s manualInputMsg;               /* Used to collect data from:       manual node                                         */
MsgType_s wpInputMsg;                   /* Used to collect data from:       waypoint node                                       */
MsgType_s testInputMsg;                 /* Used to collect data from:       test node                                           */


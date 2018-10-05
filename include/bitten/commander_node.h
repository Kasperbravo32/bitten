

struct FBRobot_s {
    int     Links;

    int     MinLink;    
    int     MaxLink;

    std::array<double,6> ResetStatePosition;
    std::array<double,6> MaxRotation;
    std::array<double,6> MinRotation;
    std::array<double,6> MaxVelocity;
    std::array<double,6> MaxEffort;
    std::array<double,6> CurrPos;
    std::array<std::string,6> joint_names;

    bool    tool;
};

struct Waypoint_s {
    double joint_positions[6];
    double velocity[6];
    double effort[6];
};

struct FBMsgType_s
{
    std::string node_name;                  /* Used to write the name of the waypoint/manual/test-node    */
    std::string program_name;               /* Used to see if e.g. test 1 in test_node is running                                   */
    
    uint32_t flags;                         /* Used for ???                                                                         */
    uint8_t id;                             /* Used to see if controlling node is test/waypoint/manual node                         */

    std::array<uint8_t, 6> joint_to_move;   /* Contains wanted joints to move. values can be either -1, 0 or 1. -1 for negative direction, 1 for positive, 0 for no action  */
    std::array<float, 6> joint_position;    /* Contains the wanted positions of each joint. used in waypoint mode, not in manual.   */
    std::array<float, 6> joint_velocity;    /* Percentage of maximum velocity to move joints with                                   */
    
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
FBRobot_s TX90;                             /* Staubli TX90 Robot                                                                   */

/* Defining waypoints   */
Waypoint_s Waypoint_0;                      /* Waypoint 0, is origo (0.0.0)                                                         */
Waypoint_s WaypointBank[5];                 /* Bank to contain 5 waypoints. only used for tests                                     */

/* Defining messages    */
FBMsgType_s manual_input_msg;               /* Used to collect data from:       manual node                                         */
FBMsgType_s wp_input_msg;                   /* Used to collect data from:       waypoint node                                       */
FBMsgType_s test_input_msg;                 /* Used to collect data from:       test node                                           */


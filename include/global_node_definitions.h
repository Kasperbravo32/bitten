#include <array>
#include <string.h>


void DEBUG_INFO();


const double LOOP_RATE_INT = 50;

enum NODE_INDICES {
    JOY_NODE,
    MANUAL_NODE,
    WP_NODE,
    TEST_NODE,
    COMMANDER_NODE,
    CONTROLLER_NODE,

    NUMBER_OF_NODES
};

enum TOPIC_INDICES {
    JOY_TOPIC,
    MANUAL_TOPIC,
    WP_TOPIC,
    TEST_TOPIC,
    JOINT_STATES_TOPIC,
    FEEDBACK_TOPIC,

    NUMBER_OF_TOPICS
};

enum NODE_IDS {
    COMMANDER_ID,
    CONTROLLER_ID,
    TEST_ID,
    WP_ID,
    MANUAL_ID,
    JOY_ID,

    NUMBER_OF_IDS
};

std::string nodeNames[NUMBER_OF_NODES] = {
    "joy",
    "Manual Node",
    "Waypoint Node",
    "Test Node",
    "Commander Node",
    "Controller Node"
};

std::string topicNames[NUMBER_OF_TOPICS] = {
    "joy",
    "manual_topic",
    "wp_topic",
    "test_topic",
    "joint_states",
    "feedback_topic"
};

struct MsgType_s
{
    std::string nodeName;                       /* Used to write the name of the waypoint/manual/test-node                              */
    std::string programName;                    /* Used to see if e.g. test 1 in test_node is running                                   */
    
    uint32_t flags;                             /* Used for ???                                                                         */
    uint8_t id;                                 /* Used to see if controlling node is test/waypoint/manual node                         */

    std::array<uint8_t  ,   6> jointToMove;     /* Contains wanted joints to move. values can be either -1, 0 or 1. -1 for negative direction, 1 for positive, 0 for no action  */
    std::array<float    ,   6> jointPosition;   /* Contains the wanted positions of each joint. used in waypoint mode, not in manual.   */
    std::array<float    ,   6> jointVelocity;   /* Percentage of maximum velocity to move joints with                                   */
    std::array<uint8_t  ,   8> buttons;         /* Array to list status of the buttons (square, triangle, X, circle + arrows)           */

};


// haej 

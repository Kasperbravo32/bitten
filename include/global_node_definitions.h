#include <array>
#include <string.h>


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

    NUMBER_OF_TOPICS
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
    "joint_states"
};



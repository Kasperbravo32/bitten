#pragma once

#include <array>
#include <string.h>
#include "ros/ros.h"

void DEBUG_INFO();


const float LOOP_RATE_INT = 50;

enum NODE_IDS {
    COMMANDER_ID,
    CONTROLLER_ID,
    TEST_ID,
    WP_ID,
    MANUAL_ID,
    JOY_ID,
    MOVEMENT_ID,
    TERMINAL_ID,

    NUMBER_OF_IDS
};


enum NODE_INDICES {
    JOY_NODE,
    MANUAL_NODE,
    WP_NODE,
    COMMANDER_NODE,
    CONTROLLER_NODE,
    MOVEMENT_NODE,
    TERMINAL_NODE,

    NUMBER_OF_NODES
};


std::string nodeNames[NUMBER_OF_NODES] = {
    "joy",
    "Manual Node",
    "Waypoint_Node",
    "Commander Node",
    "Controller Node",
    "Movement Node",
    "Terminal Node"
};


enum TOPIC_INDICES {
    JOY_TOPIC,
    MANUAL_TOPIC,
    WP_TOPIC,
    JOINT_STATES_TOPIC,
    FEEDBACK_TOPIC,
    JOINT_PATH_COMMAND_TOPIC,
    MOVEMENT_TOPIC,
    TERMINAL_TOPIC,

    NUMBER_OF_TOPICS
};


std::string topicNames[NUMBER_OF_TOPICS] = {
    "joy",
    "manual_topic",
    "wp_topic",
    "joint_states",
    "feedback_topic",
    "joint_path_command",
    "movement_topic",
    "terminal_topic"
};


struct MsgType_s
{
    std::string nodeName;                       /* Used to write the name of the waypoint/manual/test-node                              */
    std::string programName;                    /* Used to see if e.g. test 1 in test_node is running                                   */
    
    uint32_t flags;                             /* Used for setting flags, initiating various processes, asking for stuff etc           */
    uint8_t id;                                 /* Used to see if controlling node is test/waypoint/manual node                         */

    std::array<float    ,   6> jointPosition;   /* Contains the wanted positions of each joint. used in waypoint mode, not in manual.   */
    std::array<float    ,   6> jointVelocity;   /* Percentage of maximum velocity to move joints with                                   */
    std::array<uint8_t  ,   8> buttons;         /* Array to list status of the buttons (square, triangle, X, circle + arrows)           */
};


struct feedbackMsg_s
{
    uint8_t senderID;
    uint8_t recID;

    uint32_t flags;
};


enum FLAGS {
    /* 1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 32768 */
    /* 65536, 131072, 262144, 524288, 1048576, 2097152, 4194304 8388608, 16777216  */
    ACK                     = 1,    
    DENIED                  = 2,
    WAYPOINT_REACHED        = 4,
    WAYPOINT_INTERRUPTED    = 8,
    TEST_DONE_FLAG          = 16,
    TEST_INTERRUPTED        = 32,
    ROBOT_CRASHED           = 64,
    DEADMAN_STATUS          = 128,
    ESTABLISH_CONNECTION    = 256,
    TERMINATE_CONNECTION    = 512,
    PING                    = 1024,
    PONG                    = 2048,
    PAUSE                   = 4096,
    RESUME                  = 8192,
    START_OPERATION         = 16384,
    STOP_OPERATION          = 32786,
    NEW_WAYPOINT            = 65536,
    GOAL_REACHED            = 131072,
    START_TEST              = 262144,
    GET_CURR_POS            = 524288,
    SENT_CURR_POS           = 1048576,
    
    
};

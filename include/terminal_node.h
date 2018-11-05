#pragma once
/* -----------------------------------------------------------------------
 *                     -------  Functions  -------
 * ----------------------------------------------------------------------- */

/* -----------------------------------------------------------------------
 *                    -------  Enumerations  -------
 * ----------------------------------------------------------------------- */
enum KEYWORD_INDICES {
    HELP,
    MODE,
    PLAY_TEST,
    DELETE_TEST,
    RECORD,
    CLEAR,
    CLEAR_FOLDER,
    GO_HOME,

    

    NUMBER_OF_KEYWORDS
};

//     /* 1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 32768 */
//     /* 65536, 131072, 262144, 524288, 1048576, 2097152, 4194304 8388608, 16777216  */
//     ACK                     = 1,    
//     DENIED                  = 2,
//     WAYPOINT_REACHED        = 4,
//     WAYPOINT_INTERRUPTED    = 8,
//     TEST_DONE_FLAG          = 16,
//     TEST_INTERRUPTED        = 32,
//     ROBOT_CRASHED           = 64,
//     DEADMAN_STATUS          = 128,
//     ESTABLISH_CONNECTION    = 256,
//     TERMINATE_CONNECTION    = 512,
//     PING                    = 1024,
//     PONG                    = 2048,
//     PAUSE                   = 4096,
//     RESUME                  = 8192,
//     START_OPERATION         = 16384,
//     STOP_OPERATION          = 32786,
//     NEW_WAYPOINT            = 65536,
//     GOAL_REACHED            = 131072,
enum TERMINAL_FLAGS {
//     /* 1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 32768 */
//     /* 65536, 131072, 262144, 524288, 1048576, 2097152, 4194304 8388608, 16777216  */
    MODE_MANUAL_F       = 1,
    MODE_WAYPOINT_F     = 2,
    MODE_NONE_F         = 4,
    GET_POSITION_F      = 8,
    PLAY_TEST_F         = 16,
    DELETE_TEST_F       = 32,
    RECORD_TEST_F       = 64,
    START_RECORD        = 128,
    STOP_RECORD         = 256,
    GO_HOME_F           = 512,



};

/* -----------------------------------------------------------------------
 *                      -------  Variables  -------
 * ----------------------------------------------------------------------- */
std::string KeywordStrings[NUMBER_OF_KEYWORDS] = {  "help",
                                                    "mode",
                                                    "play_test",
                                                    "delete_test",
                                                    "record",
                                                    "clear",
                                                    "purge",
                                                    "home",
                                                     };

bool help_func();
bool mode_func();
bool play_test_func();
bool delete_test_func();
bool record_func();
bool clearScreen();
bool clearTestFolder();
bool goHome();

int getNumberOfTests();
void readExistingTests();






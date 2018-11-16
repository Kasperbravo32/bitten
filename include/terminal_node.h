#pragma once
/* -----------------------------------------------------------------------
 *                    -------  Enumerations  -------
 * ----------------------------------------------------------------------- */
enum KEYWORD_INDICES {
    HELP,
    MODE,
    PLAY_TEST,
    LOOP_TEST,
    DELETE_TEST,
    RECORD,
    CLEAR,
    CLEAR_FOLDER,
    GO_HOME,

    NUMBER_OF_KEYWORDS
};

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
    LOOP_TEST_F         = 1024,
};

/* -----------------------------------------------------------------------
 *                      -------  Variables  -------
 * ----------------------------------------------------------------------- */
std::string KeywordStrings[NUMBER_OF_KEYWORDS] = {  "help",
                                                    "mode",
                                                    "play_test",
                                                    "loop_test",
                                                    "delete_test",
                                                    "record",
                                                    "clear",
                                                    "purge",
                                                    "home",
                                                     };

bool help_func();
bool mode_func();
bool play_test_func();
bool loop_test_func();
bool delete_test_func();
bool record_func();
bool clearScreen();
bool clearTestFolder();
bool goHome();

int getNumberOfTests();
void readExistingTests();

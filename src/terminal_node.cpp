/* -----------------------------------------------------------------------
 * Filename: terminal_node.cpp
 * Author: Frederik Snedevind & Kasper Banke Jørgensen
 * Purpose: Create the 'terminal' node in the Leica Robot system
 * Date of dev.: September 2018 - January 2019
 * 
 * -----------------------------------------------------------------------
 *                      -------  Libraries   -------
 * ----------------------------------------------------------------------- */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "bitten/feedback_msg.h"
#include <iostream>
#include <fstream>
#include "string.h"
#include <pwd.h>

#include "global_node_definitions.h"

#include "bitten/control_msg.h"
#include "terminal_node.h"

using namespace std;
 /* ----------------------------------------------------------------------
 *                 -------  Forward Declarations   -------
 * ----------------------------------------------------------------------- */

 /* ----------------------------------------------------------------------
 *                    -------  Message objects   -------
 * ----------------------------------------------------------------------- */
bitten::control_msg terminalMsg;

 /* ----------------------------------------------------------------------
 *             -------  Variables, Constants & Objects  -------
 * ----------------------------------------------------------------------- */
string ExistingFiles[10];
string::size_type sz;

bool recording = false;

bool (* KeywordFunctions[NUMBER_OF_KEYWORDS])( void ) = {   help_func,
                                                            mode_func,
                                                            play_test_func,
                                                            delete_test_func,
                                                            record_func,
                                                            clearScreen,
                                                            clearTestFolder      };

passwd* pw = getpwuid(getuid());
string path(pw->pw_dir);

string configFilePath   = path + "/catkin_ws/src/bitten/tests/TEST_INFO.txt";
string testsPath        = path + "/catkin_ws/src/bitten/tests/";
 /* ----------------------------------------------------------------------
 *                         -------  Main   -------
 * ----------------------------------------------------------------------- */
int main (int argc , char **argv) 
{
    ros::init(argc , argv , "terminal_node");
    ros::NodeHandle n;
    
    ros::Publisher  terminal_pub    = n.advertise<bitten::control_msg>  ("terminal_topic" , 5);

    ros::Rate loop_rate(LOOP_RATE_INT);
    sleep(2);

    if (terminal_pub)
        ROS_INFO("Initiated %s", nodeNames[TERMINAL_NODE].c_str());
    else
        ROS_INFO("Failed to initiate %s",nodeNames[TERMINAL_NODE].c_str());
    
    string input;

    terminalMsg.nodeName = nodeNames[TERMINAL_NODE];
    terminalMsg.id = TERMINAL_ID;
/*  -------------------------------------------------
         SUPERLOOP
    ------------------------------------------------- */
    while(ros::ok())
    {
        cout << endl << ">> ";
        cin >> input;

        static bool foundKeywordMatch = false;
        for (int i = 0; i < NUMBER_OF_KEYWORDS; i++)
        {
            if (input == KeywordStrings[i])
            {
                foundKeywordMatch = true;
                if (KeywordFunctions[i]())
                {
                    terminal_pub.publish(terminalMsg);
                    terminalMsg.programName.clear();
                    terminalMsg.flags = 0;
                }
            }
            if (!foundKeywordMatch)
                cout << "Didn't recognize input." << endl;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}

bool clearScreen()
{
    for (int i = 0; i < 100; i++)
        cout << endl;
}


bool help_func() 
{
    cout << endl << "You can enter the following functions: " << endl;
    for (int i = 0; i < NUMBER_OF_KEYWORDS; i++)
        cout << "- " << KeywordStrings[i] << endl;
}

bool mode_func() {
    
    string input;
    static int input_i;

    cout << "Change mode to one of the following:" << endl;
    cout << "____________________________________" << endl;
    cout << "1: Manual control" << endl;
    cout << "2: Waypoint control" << endl;
    cout << "3: No control (safe mode)" << endl;
    
    do
    {
        cout << "Input: ";
        cin >> input;
        input_i = stoi(input , &sz);
        
        if (input_i != 1 && input_i != 2 && input_i != 3)
            cout << "Not a valid option." << endl;

    } while (input_i != 1 && input_i != 2 && input_i != 3);

    cout << "Changing mode to: ";

    switch(input_i)
    {
        case 1:
            cout << "Manual Control..." << endl;
            terminalMsg.flags |= MODE_MANUAL_F;
        break;

        case 2:
            cout << "Waypoint Control..." << endl;
            terminalMsg.flags |= MODE_WAYPOINT_F;
        break;

        case 3:
            cout << "No Control..." << endl;
            terminalMsg.flags |= MODE_NONE_F;
        break;
    }
    
    return true;
}

bool play_test_func() {
    
    static int chosenTest;
    cout << "Choose a file to play: " << endl << "_______________________" << endl;
    readExistingTests();
    cout << endl << "Your choice: ";
    cin >> chosenTest;

    do
    {
        cin >> chosenTest;
        if (ExistingFiles[chosenTest] == "" && chosenTest != 1337)
            cout << "Not a valid option!" << endl;
    } 
    while (ExistingFiles[chosenTest] == "" && chosenTest != 1337);
    
    if (chosenTest != 1337)
    {
        terminalMsg.flags = MODE_WAYPOINT_F | PLAY_TEST_F;
        terminalMsg.programName = ExistingFiles[chosenTest];
    }

    else
    {
        terminalMsg.flags = MODE_WAYPOINT_F | PLAY_TEST_F;
        terminalMsg.programName = "open_a_beer.txt";
        cout << "Øl på vej!" << endl;
    }
    
    return true;
}

bool delete_test_func()
{
    int fileToDelete;
    string fileToDeleteString;

    readExistingTests();
    
    cout << "File to delete: ";
    cin >> fileToDelete;

    while (ExistingFiles[fileToDelete] == "")
    {
        cout << "Not a valid option!" << endl;
        cin >> fileToDelete;
    }
    
    cout << "Deleting: " << ExistingFiles[fileToDelete];
    fileToDeleteString = testsPath + ExistingFiles[fileToDelete];
    remove(fileToDeleteString.c_str());
}


bool record_func()
{
    static int amountOfTests;
    static string filename;

    if (recording == false)
    {
        amountOfTests = getNumberOfTests();                                                 /* Open the TEST_INFO file, get current number of tests */

        if (amountOfTests < 10)
        {
            recording = true;
            filename = testsPath + "test_";
            filename += amountOfTests + '0';
            filename += ".txt";

            ofstream fileToCreate(filename);

            if (fileToCreate.is_open())
            {
                cout << "Started recording to: test_" << amountOfTests << ".txt" << endl;
                fileToCreate.close();
            }

            amountOfTests++;    

            ofstream configfile_out(configFilePath, ios_base::trunc | ios_base::out);
            configfile_out << amountOfTests;
            configfile_out.close(); 

            /* Set the controlling mode to manual, and tell commander to start recording. Also pass along the filename, path is always the same */
            terminalMsg.flags |= MODE_MANUAL_F | START_RECORD;
            terminalMsg.programName = "test_";
            terminalMsg.programName += amountOfTests - 1 + '0';
            terminalMsg.programName += ".txt";

            return true;
        }

        else
            cout << "Max test capacity reached. 'purge' before recording anymore." << endl;     
    }

    else
    {
        cout << "Stopped recording to: " << "test_" << (getNumberOfTests() - 1) << ".txt" << endl;
        recording = false;
        terminalMsg.flags = STOP_RECORD;  

        return true;
    }
}


int getNumberOfTests()
{
    /* Reads the TEST_INFO.txt file, and returns the amount of test files created, e.g. the total amount of tests */

    ifstream configfile(configFilePath);
    ofstream configfile_out;

    int a;

    if (configfile.is_open())
    {
        configfile >> a;
        configfile.close();
        return a;
    }
    else
        cout << "Couldn't open file TEST_INFO.txt" << endl;

}


void readExistingTests()
{
    /* Reads out the existing test files */
    static int a , o;
    bool FirstFileFound = false;

    a = getNumberOfTests();
    o = 0;

    for (int i = 0; i < 10; i++)
        ExistingFiles[i] = "";

    fstream FileChecker;

    string filename = testsPath + "test_";
    string temp_filename;
    
    for (int i = 0; i < a; i++)
    {
        temp_filename = filename;

        temp_filename += i + '0';
        temp_filename += ".txt";

        FileChecker.open(temp_filename);

        if (FileChecker.is_open())
        {
            if (FirstFileFound == false)
            {
                cout << "Existing Files are: " << endl;
                FirstFileFound = true;
            }

            cout << o++ << ": - test_" << i << ".txt" << endl;
            ExistingFiles[o-1] = "test_";
            ExistingFiles[o-1] += i + '0';
            ExistingFiles[o-1] += ".txt";

            FileChecker.close();
        }
    }

    if (FirstFileFound == false)
        cout << "No file exists.";

}


bool clearTestFolder()
{
    if (recording == false)
    {
        string filePath = testsPath + "test_";
        // filePath += "/catkin_ws/src/bitten/tests/test_";
        string fileExtension = ".txt";

        string fileToDelete;
        int filesDeletedCounter = 0;
    
        for (int i = 0; i < getNumberOfTests(); i++)
        {
            fileToDelete = filePath;
            fileToDelete += i + '0';
            fileToDelete += fileExtension;

            fstream FileChecker(fileToDelete);

            if (FileChecker.is_open())
            {
                FileChecker.close();
                remove(fileToDelete.c_str());
                filesDeletedCounter++;
            }
        }


        ofstream testCounterFile(configFilePath , ios_base::trunc | ios_base::out);
        
        if (testCounterFile.is_open())
        {
            testCounterFile << 0;
            testCounterFile.close();
            cout << "Deleted: " << filesDeletedCounter << " files." << endl;
        }
        else
            cout << "Couldn't open config file" << endl;
    }
    else
        cout << "Currently recording. stop recording before purging" << endl;
}
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
#include <iostream>
#include <fstream>
#include <pwd.h>

#include "string.h"
#include "global_node_definitions.h"
#include "bitten/control_msg.h"
#include "terminal_node.h"

using namespace std;
 /* ----------------------------------------------------------------------
 *                    -------  Message objects   -------
 * ----------------------------------------------------------------------- */
bitten::control_msg terminalMsg;

 /* ----------------------------------------------------------------------
 *             -------  Variables, Constants & Objects  -------
 * ----------------------------------------------------------------------- */
bool terminalTxRdy      = false;
bool recording          = false;
bool foundKeywordMatch  = false;

bool (* KeywordFunctions[NUMBER_OF_KEYWORDS])( void ) = {   help_func,
                                                            mode_func,
                                                            play_test_func,
                                                            loop_test_func,
                                                            delete_test_func,
                                                            record_func,
                                                            clearScreen,
                                                            clearTestFolder,
                                                            goHome      };

passwd* pw = getpwuid(getuid());
string path(pw->pw_dir);

string configFilePath   = path + "/catkin_ws/src/bitten/tests/TEST_INFO.txt";
string testsPath        = path + "/catkin_ws/src/bitten/tests/";
string input_s;
string ExistingFiles[10];
// string::size_type sz;

int input_i;

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
    
    terminalMsg.nodeName = nodeNames[TERMINAL_NODE];
    terminalMsg.id = TERMINAL_ID;
/*  -------------------------------------------------
         SUPERLOOP
    ------------------------------------------------- */
    while(ros::ok())
    {
        cout << ">> ";
        
        getline(cin , input_s);

        stringstream input_ss(input_s);
        foundKeywordMatch = false;

        for (int i = 0; i < NUMBER_OF_KEYWORDS; i++)
        {
            if (input_s == KeywordStrings[i])
            {
                foundKeywordMatch = true;
                clearScreen();

                if (KeywordFunctions[i]())
                {
                    terminal_pub.publish(terminalMsg);
                    terminalMsg.programName.clear();
                    terminalMsg.flags = 0;
                }
            }
        }

        if (input_s == "")
        {
            /* Secret stuff happens in here */
        }

        else if (!foundKeywordMatch)
            cout << "Didn't recognize input." << endl;

        if (terminalTxRdy == true)
        {
            terminal_pub.publish(terminalMsg);
            terminalTxRdy = false;
            terminalMsg.flags = 0;
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
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

bool mode_func() 
{
    cout << "Change mode to one of the following:" << endl;
    cout << "____________________________________" << endl;
    cout << "1: Manual control" << endl;
    cout << "2: Waypoint control" << endl;
    cout << "3: No control (safe mode)" << endl;
    
    do
    {
        cout << endl << "Input: ";
        getline(cin , input_s);

        stringstream input_ss(input_s);

        if (input_ss >> input_i)
        {
            if (input_i == 1 || input_i == 2 || input_i == 3)
                break;
            else
                cout << "Not a valid option" << endl;
        }

        else
            cout << "Not a valid option" << endl;

    } while (input_i != 1 && input_i != 2 && input_i != 3);

    cout << endl << "Changing mode to: ";
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
    cout << endl;
    
    return true;
}

bool play_test_func() 
{    
    cout << "Choose a file to play: " << endl << "_______________________" << endl;
    readExistingTests();
    
    while(1)
    {
        cout << endl << "Your choice: ";
        getline(cin , input_s);
        stringstream input_ss(input_s);
        if (input_ss >> input_i)
        {
            if (input_i == 1337 || ExistingFiles[input_i] != "")
                break;
            else
                cout << "Not a valid number." << endl;
        }
        else
            cout << "Not valid input." << endl;
    }
    
    if (input_i != 1337)
    {
        terminalMsg.programName = ExistingFiles[input_i];
        cout << "Playing file: " << ExistingFiles[input_i] << endl;
    }
        
    else
    {
        terminalMsg.programName = "open_a_beer.txt";
        cout << "Øl på vej!" << endl;
    }

    terminalMsg.flags = MODE_WAYPOINT_F | PLAY_TEST_F;
    return true;
}

bool loop_test_func() 
{
    static int chosenTest, chosenLoops;
    cout << "Choose a file to loop: " << endl << "_______________________" << endl;
    readExistingTests();
    
    while(1)
    {
        cout << endl << "Your choice: ";
        getline(cin , input_s);
        stringstream input_ss(input_s);
        if (input_ss >> input_i)
        {
            if (input_i == 1337 || ExistingFiles[input_i] != "")
            {
                chosenTest = input_i;
                break;
            }
            else
                cout << "Not a valid number." << endl;
        }
        else
            cout << "Not valid input." << endl;
    }


    while(1)
    {
        cout << endl << "Enter number of times to loop: [1:63]: ";
        getline(cin , input_s);
        stringstream input_ss(input_s);

        if (input_ss >> input_i)
        {
            if (input_i >= 1 && input_i <= 63)
            {
                chosenLoops = input_i;
                break;
            }
                
            else
                cout << "Not a valid input." << endl;
        }
        else
            cout << "Not a valid input." << endl;
    }
    
    if (chosenTest != 1337)
    {
        terminalMsg.programName = ExistingFiles[chosenTest];
        cout << "Playing file: " << ExistingFiles[chosenTest] << " " << chosenLoops << " times." << endl;
    }
        
    else
    {
        terminalMsg.programName = "open_a_beer.txt";
        cout << "Øl på vej!" << endl;
    }

    terminalMsg.flags = MODE_WAYPOINT_F | LOOP_TEST_F;
    terminalMsg.id = chosenLoops;
    return true;


}

bool delete_test_func()
{
    int fileToDelete;
    string fileToDeleteString;
    readExistingTests();
    
    do
    {
        cout << "File to delete: ";
        getline(cin , input_s);
        stringstream input_ss(input_s);

        if (input_ss >> input_i)
        {
            if (ExistingFiles[input_i] != "")
            {
                cout << "Deleting: " << ExistingFiles[input_i] << endl;
                fileToDeleteString = testsPath + ExistingFiles[fileToDelete];
                remove(fileToDeleteString.c_str());            
                break;
            }
            else
                cout << "Not a valid option" << endl;
        }
        else
            cout << "Not a valid option" << endl;
    } while (1);   
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

    int amountOfTests;

    if (configfile.is_open())
    {
        configfile >> amountOfTests;
        configfile.close();
        return amountOfTests;
    }
    else
        cout << "Couldn't open file TEST_INFO.txt" << endl;
}


void readExistingTests()
{
    /* Reads out the existing test files */
    static int amountOfTests , index;
    bool FirstFileFound = false;

    amountOfTests = getNumberOfTests();
    index = 0;

    for (int i = 0; i < 10; i++)
        ExistingFiles[i] = "";

    fstream FileChecker;

    string filename = testsPath + "test_";
    string temp_filename;
    
    for (int i = 0; i < amountOfTests; i++)
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

            cout << index++ << ": - test_" << i << ".txt" << endl;
            ExistingFiles[index-1] = "test_";
            ExistingFiles[index-1] += i + '0';
            ExistingFiles[index-1] += ".txt";

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

bool goHome()
{
    terminalMsg.flags |= GO_HOME_F;
    return true;
}

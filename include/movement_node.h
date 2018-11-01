#pragma once

/* Declare Struct to define new robots, containing various info, like amount of links, max rotation in each link, max speed etc.    */
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
std::array<double,6> goalPosition;
std::array<double,6> lastGoalPosition;

std::array<int,6>    jointsAtGoal;

std::array<std::string,6> jointNames;

bool tool;
};

/* Defining robots      */
// extern Robot_s TX90;                  /* Staubli TX90 Robot          */

std::array<double,6> *getCurrPos();


class TX90_c {

    public:
        
        void setCurrPos(double arr[6]);
        void setGoalPos(double arr[6]);    /* put lastGoalPos control in here */
        void setJointsAtGoal(int n);
        void setCurrVelocity(double n);

        double getCurrPos(int n);
        double getGoalPos(int n);
        double getCurrVelocity();
        int     getJointsAtGoal();

    private:

        std::array<double,6>    currPos;
        std::array<double,6>    goalPosition;
        std::array<int,6>       jointsAtGoal;
        double                  currVelocity;
        std::array<double,6>    lastGoalPosition; 

        int                     links   = 6;
        int                     minLink = 1;   
        int                     maxLink = 6;

        bool                    tool = false;

        std::array<double,6>    resetStatePosition = { 0,0,0,0,0,0};

        std::array<double,6>    maxRotation        = { 3.14,                   /* joint_1  */
                                                       2.57,                   /* joint_2  */
                                                       2.53,                   /* joint_3  */
                                                       4.71,                   /* joint_4  */
                                                       2.44,                   /* joint_5  */
                                                       4.71};                  /* joint_6  */


        std::array<double,6>    minRotation        = { -3.14,                  /* joint_1  */
                                                       -2.27,                  /* joint_2  */
                                                       -2.53,                  /* joint_3  */
                                                       -4.71,                  /* joint_4  */
                                                       -2.01,                  /* joint_5  */
                                                       -4.71};                 /* joint_6  */


        std::array<double,6>    maxVelocity        = { (400.0/180.0)*3.14,     /* joint_1  */
                                                       (400.0/180.0)*3.14,     /* joint_2  */
                                                       (430.0/180.0)*3.14,     /* joint_3  */
                                                       (540.0/180.0)*3.14,     /* joint_4  */
                                                       (475.0/180.0)*3.14,     /* joint_5  */
                                                       (760.0/180.0)*3.14};    /* joint_6  */


        std::array<double,6>    maxEffort          = { 318.0,                  /* joint_1  */
                                                       166.0,                  /* joint_2  */
                                                       76.0,                   /* joint_3  */
                                                       34.0,                   /* joint_4  */
                                                       29.0,                   /* joint_5  */
                                                       11.0};                  /* joint_6  */
        

        std::array<std::string,6> jointNames    = { "joint_1", 
                                                    "joint_2", 
                                                    "joint_3", 
                                                    "joint_4", 
                                                    "joint_5", 
                                                    "joint_6"};
};

Robot_c::setCurrPos(double arr[6])
{
/* Takes an array of length [6] in.
 * Array should contain doubles.
 * Overwrites the content of private member
 * Robot_c.currPos[6] with input
 */
    for (int i = 0; i < 6; i++)
    {
        currPos[i] = arr[i];
    }
}



void setCurrPos(double arr[6]);
        void setGoalPos(double arr[6]);    /* put lastGoalPos control in here */
        void setJointsAtGoal(int n);
        void setCurrVelocity(double n);

        double getCurrPos(int n);
        double getGoalPos(int n);
        double getCurrVelocity();
        int     getJointsAtGoal();
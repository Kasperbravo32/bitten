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

// std::array<double,6> *getCurrPos();


class TX90_c {

    public:
        
        void            setCurrPos(int n , double val);
        void            setGoalPos(int n , double val);    /* put lastGoalPos control in here */
        void            setJointsAtGoal(int arr[6]);
        void            setCurrVelocity(double n);

        double          getCurrPos(int n);
        double          getGoalPos(int n);
        double          getCurrVelocity();
        double          getMaxRotation(int n);
        double          getMinRotation(int n);
        double          getMaxVelocity(int n);
        double          getResetStatePos(int n);

        
        uint8_t         getJointsAtGoal();
        std::string     getJointName(int n);

        std::array<double,6> *getPointerFunc( void )
        {
            return &currPos;
        }

    private:

        std::array<double,6>    currPos;
        std::array<double,6>    goalPosition;
        std::array<int,6>       jointsAtGoal;
        double                  currVelocity         = 0.4;
        std::array<double,6>    lastGoalPosition; 

        static const int        links               = 6;
        static const int        minLink             = 1;   
        static const int        maxLink             = 6;

        static const bool       tool                = false;

        const std::array<double,6>    resetStatePosition = {{    0,0,0,0,0,0}};

        const std::array<double,6>    maxRotation        = {{    3.14,                   /* joint_1  */
                                                                 2.57,                   /* joint_2  */
                                                                 2.53,                   /* joint_3  */
                                                                 4.71,                   /* joint_4  */
                                                                 2.44,                   /* joint_5  */
                                                                 4.71}};                  /* joint_6  */

        const std::array<double,6>    minRotation        = {{     -3.14,                  /* joint_1  */
                                                                 -2.27,                  /* joint_2  */
                                                                 -2.53,                  /* joint_3  */
                                                                 -4.71,                  /* joint_4  */
                                                                 -2.01,                  /* joint_5  */
                                                                 -4.71}};                 /* joint_6  */


        const std::array<double,6>    maxVelocity        = {{    (400.0/180.0)*3.14,     /* joint_1  */
                                                                 (400.0/180.0)*3.14,     /* joint_2  */
                                                                 (430.0/180.0)*3.14,     /* joint_3  */
                                                                 (540.0/180.0)*3.14,     /* joint_4  */
                                                                 (475.0/180.0)*3.14,     /* joint_5  */
                                                                 (760.0/180.0)*3.14}};    /* joint_6  */

        const std::array<double,6>    maxEffort          = {{    318.0,                  /* joint_1  */
                                                                 166.0,                  /* joint_2  */
                                                                 76.0,                   /* joint_3  */
                                                                 34.0,                   /* joint_4  */
                                                                 29.0,                   /* joint_5  */
                                                                 11.0}};                  /* joint_6  */
        

        const std::array<std::string,6> jointNames    = {{      "joint_1", 
                                                                "joint_2", 
                                                                "joint_3", 
                                                                "joint_4", 
                                                                "joint_5", 
                                                                "joint_6"}};
};

extern TX90_c TX90;

void TX90_c::setCurrPos(int n, double val)
{
/* Takes an array of length [6] in.
 * Array should contain doubles.
 * Overwrites the content of private member
 * Robot_c.currPos[] with input             */
    currPos[n] = val;
}

void TX90_c::setGoalPos(int n , double val)
{
/* Overwrites content of private member Robot_c.goalPosition[] with input
 * Along with copying existing goalPosition[] into lastGoalPosition[]   */

    // TX90_c::lastGoalPosition[n] = TX90_c::goalPosition[n];
    goalPosition[n] = val;

}


void TX90_c::setJointsAtGoal(int arr[6])
{
    for (int i = 0; i < 6; i++)
        jointsAtGoal[i] = arr[i];
}

void TX90_c::setCurrVelocity(double n)
{
    currVelocity = n;
}

double TX90_c::getCurrPos(int n)
{
    return currPos[n];
}

double TX90_c::getGoalPos(int n)
{
    return goalPosition[n];
}

double TX90_c::getCurrVelocity()
{
    return currVelocity;
}

uint8_t TX90_c::getJointsAtGoal()
{
    uint8_t counter;
    for (int i = 0; i < 6; i++)
    {
        if (jointsAtGoal[i] == 1)
        counter |= (1 << i);
    }
    return counter;
}


std::string TX90_c::getJointName(int n)
{
    return jointNames[n];
}

double TX90_c::getMaxVelocity(int n)
{
    return maxVelocity[n];
}

double TX90_c::getResetStatePos(int n)
{
    return resetStatePosition[n];
}

double TX90_c::getMaxRotation(int n)
{
    return maxRotation[n];
}

double TX90_c::getMinRotation(int n)
{
    return minRotation[n];
}
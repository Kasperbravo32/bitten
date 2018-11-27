#pragma once

/* Define the robot as a class, to keep track of, and modify all values, positions, states etc. */
class TX90_c {

    public:
        
        void            setCurrPos(int n , double val);                 /* Function to change the current digital position of the robot                                             */
        void            setGoalPos(int n , double val);                 /* Function to change to current digital GoalPosition of the robot                                          */
        void            setCurrVelocity(double n);                      /* Function to change the current velocity of the robot, 0-100% (0-1)                                       */
        void            setJointsAtGoal(int arr[6]);                    /* Function to change whether or not each joint is at its intended goal position, within an error margin    */

        double          getCurrPos(int n);                              /* Function to get (return) the current position of a certain joint (joint_0 - joint_5)                                     */
        double          getGoalPos(int n);                              /* Function to get the current goal position of a single joint (joint_0 - joint_5)                                          */
        double          getCurrVelocity();                              /* Function to get the current velocity of the robot                                                                        */
        double          getMaxRotation(int n);                          /* Function to get the max possible rotation of the robot in positive direction, on a single joint (joint_0 - joint_5)      */
        double          getMinRotation(int n);                          /* Function to get the possible rotation of the robot in negative direction, on a single joint                              */
        double          getMaxVelocity(int n);                          /* Function to get the max possible allowed speed of the robot, on a single joint                                           */
        double          getResetStatePos(int n);                        /* Function to get the coordinate of the resetstateposition, of a single joint                                              */
        uint8_t         getJointsAtGoal();                              /* Returns an 8-bit integer, where the first 6 bits ([bit_0:bit_5]) determines whether or not each joint is at its goal position    */
        std::string     getJointName(int n);                            /* Returns the URDF name of a single joint                                                                                          */

        void            setLastPos(int n, double pos); 
        double          getLastPos(int n); 

    private:

        std::array<double,6>    currPos;                                /* Array of doubles, keeps track of current position of each joint. is being updated in the movement node, by the feedback_states topic, from robot_state node  */
        std::array<double,6>    goalPos;                                /* Array of doubles, keeps track of goalPosition. Gets transmitted when the robot is supposed to move somewhere                                                 */
        std::array<double,6>    lastPos;                                /* Array of doubles, keeps track of goalPosition. Gets transmitted when the robot is supposed to move somewhere                                                 */
        std::array<int,6>       jointsAtGoal;                           /* Array of ints, keeps track of, whether or not a joint is at its goal position. Could be remade into an 8-bits u_integer                                      */
        double                  currVelocity         = 0.4;             /* Double, value 0-1, controls the stepsize used when calculating new goalposition, should reset in increased or decreased speed                                */
       
        static const int        links               = 6;                /* Int to define the amount of links on the robot   */
        static const int        minLink             = 1;                /* Int to define the value of the first link        */
        static const int        maxLink             = 6;                /* Int to define the value of the Last link         */

        static const bool       tool                = false;

        const std::array<double,6>    resetStatePos      = {{    0,0,0,0,0,0}};

        const std::array<double,6>    maxRotation        = {{    3.14,                      /* joint_1  */
                                                                 2.57,                      /* joint_2  */
                                                                 2.53,                      /* joint_3  */
                                                                 4.71,                      /* joint_4  */
                                                                 2.44,                      /* joint_5  */
                                                                 4.71}};                    /* joint_6  */

        const std::array<double,6>    minRotation        = {{    -3.14,                     /* joint_1  */
                                                                 -2.27,                     /* joint_2  */
                                                                 -2.53,                     /* joint_3  */
                                                                 -4.71,                     /* joint_4  */
                                                                 -2.01,                     /* joint_5  */
                                                                 -4.71}};                   /* joint_6  */


        const std::array<double,6>    maxVelocity        = {{    (400.0/180.0)*3.14,        /* joint_1  */
                                                                 (400.0/180.0)*3.14,        /* joint_2  */
                                                                 (430.0/180.0)*3.14,        /* joint_3  */
                                                                 (540.0/180.0)*3.14,        /* joint_4  */
                                                                 (475.0/180.0)*3.14,        /* joint_5  */
                                                                 (760.0/180.0)*3.14}};      /* joint_6  */

        const std::array<double,6>    maxEffort          = {{    318.0,                     /* joint_1  */
                                                                 166.0,                     /* joint_2  */
                                                                 76.0,                      /* joint_3  */
                                                                 34.0,                      /* joint_4  */
                                                                 29.0,                      /* joint_5  */
                                                                 11.0}};                    /* joint_6  */
        

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

double TX90_c::getCurrPos(int n)
{
    return currPos[n];
}

void TX90_c::setGoalPos(int n , double val)
{
/* Overwrites content of private member Robot_c.goalPosition[] with input
 * Along with copying existing goalPosition[] into lastGoalPosition[]   */
    goalPos[n] = val;
}

double TX90_c::getGoalPos(int n)
{
    return goalPos[n];
}

void TX90_c::setJointsAtGoal(int arr[6])
{
    for (int i = 0; i < 6; i++)
        jointsAtGoal[i] = arr[i];
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

void TX90_c::setCurrVelocity(double n)
{
    currVelocity = n;
}

double TX90_c::getCurrVelocity()
{
    return currVelocity;
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
    return resetStatePos[n];
}

double TX90_c::getMaxRotation(int n)
{
    return 0.995 * maxRotation[n];
}

double TX90_c::getMinRotation(int n)
{
    return 0.995 * minRotation[n];
}

void TX90_c::setLastPos(int n, double pos)
{
    lastPos[n] = pos;
}

double TX90_c::getLastPos(int n)
{
    return lastPos[n];
}

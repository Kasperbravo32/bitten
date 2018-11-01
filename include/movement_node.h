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
        void setmaxRotation(double arr[6]);

    


    private:
        int     links;
        int     minLink;    
        int     maxLink;

        bool    tool;

        double  currVelocity;

        std::array<int,6>    jointsAtGoal;
        std::array<double,6> resetStatePosition = {0,0,0,0,0,0};
        std::array<double,6> maxRotation;
        std::array<double,6> minRotation;
        std::array<double,6> maxVelocity;
        std::array<double,6> maxEffort;
        std::array<double,6> currPos;
        std::array<double,6> goalPosition;
        std::array<double,6> lastGoalPosition;
        std::array<std::string,6> jointNames;
        
        



};

Robot_c::setmaxRotation(double arr[6])
{

}

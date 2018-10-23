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

std::array<int,6>    jointsAtGoal;

std::array<std::string,6> jointNames;


bool    tool;
};

/* Defining robots      */
Robot_s TX90;                             /* Staubli TX90 Robot                                                                     */
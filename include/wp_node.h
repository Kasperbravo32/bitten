 /* ----------------------------------------------------------------------
 *                      -------  Initializing   -------
 * ----------------------------------------------------------------------- */
void fbCallback(const bitten::feedback_msg::ConstPtr& feedback);

struct Waypoint_s {
    double jointPosition[6];
    double velocity[6];
    double effort[6];
    std::string waypointName;
};

/* ----------------------------------------------------------------------
 *                       -------  Global variables   -------
 * ----------------------------------------------------------------------- */
bitten::control_msg wp_msg;
bitten::feedback_msg fb_msg;

bool readyForNextWp = false;
bool transmitWpReady = false;
bool connectionEstablished = false;

Waypoint_s Waypoint_0;

/*_Terminaleksempel på waypoint record mode_______________________________
|
| Move your robot to Waypoint #1: ok
| Do you want to add another waypoint? [y/n]: y
| Move robot to waypoint #2: y
| Move your robot to Waypoint #2: ok
| Do you want to add another waypoint? [y/n]: y
| Move your robot to Waypoint #3: ok
| Do you want to add another waypoint? [y/n]: n
|
| Recorded '3' waypoints. Enter speed coefficient [0 - 100]: 25
| Do you want to play waypoints?: y
|
| Playing '3' waypoints.
| Moving robot to WP#1
| Done.
| Moving robot to WP#2
| Done.
| Moving robot to WP#3
| Done.
|
| No more waypoints. returning to standby
|________________________________________________________________________
*/
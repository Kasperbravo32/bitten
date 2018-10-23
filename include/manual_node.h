 /* -----------------------------------------------------------------------
 *                      -------  Libraries   -------
 * ----------------------------------------------------------------------- */
#include <sensor_msgs/Joy.h>

 /* ----------------------------------------------------------------------
 *                      -------  Initializing   -------
 * ----------------------------------------------------------------------- */
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
void canCallback(const bitten::canopen_msg::ConstPtr& can);
void fbCallback(const bitten::feedback_msg::ConstPtr& feedback);

/* ----------------------------------------------------------------------
 *                       -------  Global variables   -------
 * ----------------------------------------------------------------------- */
bitten::control_msg manual_msg;

bool connectionEstablished = false;
bool transmitManualRdy = false;
bool newConnection = true;
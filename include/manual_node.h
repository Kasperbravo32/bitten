 /* -----------------------------------------------------------------------
 *                      -------  Libraries   -------
 * ----------------------------------------------------------------------- */
#include <sensor_msgs/Joy.h>

 /* ----------------------------------------------------------------------
 *                      -------  Initializing   -------
 * ----------------------------------------------------------------------- */
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
void fbCallbackManual(const bitten::feedback_msg::ConstPtr& feedbackManual);

/* ----------------------------------------------------------------------
 *                       -------  Global variables   -------
 * ----------------------------------------------------------------------- */
bitten::control_msg manual_msg;

bool connectionEstablished = false;
bool transmitManualRdy = false;
bool newConnection = true;
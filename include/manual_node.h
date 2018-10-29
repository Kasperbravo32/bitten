 /* -----------------------------------------------------------------------
 *                      -------  Libraries   -------
 * ----------------------------------------------------------------------- */
// #include <sensor_msgs/Joy.h>

 /* ----------------------------------------------------------------------
 *                      -------  Initializing   -------
 * ----------------------------------------------------------------------- */
// void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
void canCallback(const bitten::can_msg::ConstPtr& can);
void fbCallback(const bitten::feedback_msg::ConstPtr& feedback);


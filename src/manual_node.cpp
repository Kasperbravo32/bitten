#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Joy.h>
#include <bitten/control_msg.h>

struct msg_type
{
    std::string node_name;
    std::string program_name;
    uint32_t flags;
    uint8_t id;
    float joint_position[6];
    float joint_velocity[6];
    uint8_t joint_to_move[6];
}fb_control_msg;

int8_t msg_received = 0;
float joint_vel[6];

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    msg_received = 1;

    fb_control_msg.joint_velocity[0] = joy->axes[0];
    fb_control_msg.joint_velocity[1] = joy->axes[4];
    fb_control_msg.joint_velocity[2] = joy->axes[1];
    
    if(joy->buttons[15])
        fb_control_msg.joint_velocity[3] = -1;
    else if(joy->buttons[16])
        fb_control_msg.joint_velocity[3] = 1;
    else
        fb_control_msg.joint_velocity[3] = 0;

    fb_control_msg.joint_velocity[4] = joy->axes[3];
    
    if(joy->buttons[1])
        fb_control_msg.joint_velocity[5] = 1;
    else if(joy->buttons[3])
        fb_control_msg.joint_velocity[5] = -1;
    else
        fb_control_msg.joint_velocity[5] = 0;

    std::cout << "joint1: " << fb_control_msg.joint_velocity[0] << std::endl
            << "joint2: " << fb_control_msg.joint_velocity[1] << std::endl
            << "joint3: " << fb_control_msg.joint_velocity[2] << std::endl
            << "joint4: " << fb_control_msg.joint_velocity[3] << std::endl
            << "joint5: " << fb_control_msg.joint_velocity[4] << std::endl
            << "joint6: " << fb_control_msg.joint_velocity[5] << std::endl << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "manual_node");
    ros::NodeHandle n;
    // ros::NodeHandle m;

    fb_control_msg.node_name = "manual_node";

    ros::Subscriber joy_sub = n.subscribe<sensor_msgs::Joy>("joy", 1000, &joyCallback);

    ros::Publisher manual_pub = n.advertise<bitten::control_msg>("manual_topic", 1000);

    ros::Rate loop_rate(10);
    int count = 0;

    ros::spinOnce();

    while (ros::ok())
    {
        if(msg_received)
        {
            msg_received = 0;
            bitten::control_msg msg;
            
            msg.node_name = "manual node";
            msg.joints_velocity[0] = fb_control_msg.joint_velocity[0];
            msg.joints_velocity[1] = fb_control_msg.joint_velocity[1];
            msg.joints_velocity[2] = fb_control_msg.joint_velocity[2];
            msg.joints_velocity[3] = fb_control_msg.joint_velocity[3];
            msg.joints_velocity[4] = fb_control_msg.joint_velocity[4];
            msg.joints_velocity[5] = fb_control_msg.joint_velocity[5];
            
            manual_pub.publish(msg);
        }
        
    //   std_msgs::String msg;

    //   std::stringstream ss;
    //   ss << "hello world " << count;
    //   msg.data = ss.str();

    //   ROS_INFO("%s", msg.data.c_str());
        

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }

    return 0;
}
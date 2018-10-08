#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Joy.h>
#include <bitten/control_msg.h>

struct msg_type
{
    std::string nodeName;
    std::string programName;
    uint32_t flags;
    uint8_t id;
    float jointPosition[6];
    float jointVelocity[6];
    uint8_t jointToMove[6];
}controlMsg;

int8_t msgReceived = 0;
float jointVel[6];

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    msgReceived = 1;

    //joint 1
    controlMsg.jointVelocity[0] = joy->axes[0];
    //joint 2
    controlMsg.jointVelocity[1] = joy->axes[4];
    //joint 3
    controlMsg.jointVelocity[2] = joy->axes[1];
    //joint 4
    if(joy->buttons[15])
        controlMsg.jointVelocity[3] = -1;
    else if(joy->buttons[16])
        controlMsg.jointVelocity[3] = 1;
    else
        controlMsg.jointVelocity[3] = 0;
    //joint 5
    controlMsg.jointVelocity[4] = joy->axes[3];
    //joint 6
    if(joy->buttons[1])
        controlMsg.jointVelocity[5] = 1;
    else if(joy->buttons[3])
        controlMsg.jointVelocity[5] = -1;
    else
        controlMsg.jointVelocity[5] = 0;

    std::cout << "joint1: " << controlMsg.jointVelocity[0] << std::endl
            << "joint2: " << controlMsg.jointVelocity[1] << std::endl
            << "joint3: " << controlMsg.jointVelocity[2] << std::endl
            << "joint4: " << controlMsg.jointVelocity[3] << std::endl
            << "joint5: " << controlMsg.jointVelocity[4] << std::endl
            << "joint6: " << controlMsg.jointVelocity[5] << std::endl << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "manual_node");
    ros::NodeHandle n;

    controlMsg.nodeName = "manual_node";

    ros::Subscriber joySub = n.subscribe<sensor_msgs::Joy>("joy", 1000, &joyCallback);

    ros::Publisher manualPub = n.advertise<bitten::control_msg>("manual_topic", 1000);

    ros::Rate loop_rate(10);
    int count = 0;

    ros::spinOnce();

    while (ros::ok())
    {
        if(msgReceived)
        {
            msgReceived = 0;
            bitten::control_msg msg;
            
            msg.node_name = "manual node";
            msg.jointsVelocity[0] = controlMsg.jointVelocity[0];
            msg.jointsVelocity[1] = controlMsg.jointVelocity[1];
            msg.jointsVelocity[2] = controlMsg.jointVelocity[2];
            msg.jointsVelocity[3] = controlMsg.jointVelocity[3];
            msg.jointsVelocity[4] = controlMsg.jointVelocity[4];
            msg.jointsVelocity[5] = controlMsg.jointVelocity[5];
            
            manualPub.publish(msg);
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
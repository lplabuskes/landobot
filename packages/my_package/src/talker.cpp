#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <cstdlib>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    ros::Rate loop_rate(1);

    
    std::string vehicle_name = std::getenv("VEHICLE_NAME");
    std::stringstream ss;
    ss << "Hello from " << vehicle_name << "!";
    std_msgs::String msg;
    msg.data = ss.str();

    while (ros::ok())
    {
        ROS_INFO("%s", msg.data.c_str());

        chatter_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }


    return 0;
}
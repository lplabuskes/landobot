#include <cstdlib>
#include <string>
#include <signal.h>

#include "ros/ros.h"
#include "duckietown_msgs/WheelsCmdStamped.h"


const float THROTTLE_LEFT = 0.5;
const int DIRECTION_LEFT = 1;
const float THROTTLE_RIGHT = 0.3;
const int DIRECTION_RIGHT = -1;

sig_atomic_t volatile g_shutdown_flag = 0;

void on_shutdown(int sig)
{
    g_shutdown_flag = 1;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wheel_control");
    ros::NodeHandle nh;
    signal(SIGINT, on_shutdown);

    std::string vehicle_name = std::getenv("VEHICLE_NAME");
    ros::Publisher pub = nh.advertise<duckietown_msgs::WheelsCmdStamped>(
                        "/"+vehicle_name+"/wheels_driver_node/wheels_cmd", 1);
    ros::Rate loop_rate(10);

    duckietown_msgs::WheelsCmdStamped msg;
    msg.vel_left = THROTTLE_LEFT*DIRECTION_LEFT;
    msg.vel_right = THROTTLE_RIGHT*DIRECTION_RIGHT;

    while (!g_shutdown_flag)
    {
        pub.publish(msg);
        loop_rate.sleep();
    }
    
    msg.vel_left = 0.0;
    msg.vel_right = 0.0;
    pub.publish(msg);

    ros::shutdown();

    return 0;
}
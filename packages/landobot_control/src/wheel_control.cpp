#include <cstdlib>
#include <string>
#include <signal.h>

#include "ros/ros.h"
#include "duckietown_msgs/Twist2DStamped.h"


sig_atomic_t volatile g_shutdown_flag = 0;

float VELOCITY = 0.3;
float OMEGA = 4.0;

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
    ros::Publisher pub = nh.advertise<duckietown_msgs::Twist2DStamped>(
                        "/"+vehicle_name+"/car_cmd_switch_node/cmd", 1);
    ros::Rate loop_rate(10);

    duckietown_msgs::Twist2DStamped msg;
    msg.v = VELOCITY;
    msg.omega = OMEGA;

    while (!g_shutdown_flag)
    {
        pub.publish(msg);
        loop_rate.sleep();
    }
    
    msg.v = 0.0;
    msg.omega = 0.0;
    pub.publish(msg);

    ros::shutdown();

    return 0;
}
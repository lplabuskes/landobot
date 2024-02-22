#include "ros/ros.h"
#include "duckietown_msgs/WheelEncoderStamped.h"


class EncoderReader
{
    private:
        ros::NodeHandle nh_;
        ros::Subscriber left_encoder_sub_;
        ros::Subscriber right_encoder_sub_;
        ros::Timer timer_;
        int32_t left_data_ = 0, right_data_ = 0;

    public:
        EncoderReader()
        {
            // Set up subscriptions
            left_encoder_sub_ = nh_.subscribe("left_wheel_encoder_node/tick", 1000, &EncoderReader::left_encoder_callback, this);
            right_encoder_sub_ = nh_.subscribe("right_wheel_encoder_node/tick", 1000, &EncoderReader::right_encoder_callback, this);
            // Set up timer
            timer_ = nh_.createTimer(ros::Duration(0.05), &EncoderReader::timer_callback, this);
        }

        void left_encoder_callback(const duckietown_msgs::WheelEncoderStamped::ConstPtr& msg)
        {
            ROS_INFO_ONCE("Left encoder resolution: %u", msg->resolution);
            ROS_INFO_ONCE("Left encoder type: %u", msg->type);

            left_data_ = msg->data;
        }

        void right_encoder_callback(const duckietown_msgs::WheelEncoderStamped::ConstPtr& msg)
        {
            ROS_INFO_ONCE("Right encoder resolution: %u", msg->resolution);
            ROS_INFO_ONCE("Right encoder type: %u", msg->type);

            right_data_ = msg->data;
        }

        void timer_callback(const ros::TimerEvent& e)
        {
            ROS_INFO("Ticks [LEFT, RIGHT]: [%d, %d]", left_data_, right_data_);
        }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "encoder_reader");
    EncoderReader er;
    ros::spin();

    return 0;
}
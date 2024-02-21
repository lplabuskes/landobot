#include <cstdlib>
#include <sstream>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "camera-reader";

class CameraReader
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;

    public:
        CameraReader()
            : it_(nh_)
        {
            // Subscribe to image from duckiebot
            image_transport::TransportHints hints("compressed");
            std::stringstream camera_topic;
            camera_topic << "/" << std::getenv("VEHICLE_NAME") << "/camera_node/image";
            image_sub_ = it_.subscribe(camera_topic.str(), 1, &CameraReader::image_callback, this, hints);

            cv::namedWindow(OPENCV_WINDOW);
        }

        ~CameraReader()
        {
            cv::destroyWindow(OPENCV_WINDOW);
        }

        void image_callback(const sensor_msgs::ImageConstPtr& msg)
        {
            cv_bridge::CvImageConstPtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }

            // Update GUI Window
            cv::imshow(OPENCV_WINDOW, cv_ptr->image);
            cv::waitKey(1);
        }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_reader");
    CameraReader cr;
    ros::spin();
    return 0;
}
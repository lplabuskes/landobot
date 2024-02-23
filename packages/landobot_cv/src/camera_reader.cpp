#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "camera-reader";
static const std::string OPENCV_WINDOW_2 = "camera-gray";

class CameraReader
{
    ros::NodeHandle nh_;
    ros::Subscriber info_sub_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;

    bool intrinsics_initialized_ = false;
    cv::Mat map1_, map2_;

    public:
        CameraReader()
            : it_(nh_)
        {
            // Subscribe to image from duckiebot
            image_transport::TransportHints hints("compressed");
            image_sub_ = it_.subscribe("camera_node/image", 1, &CameraReader::image_callback, this, hints);

            // Subscribe to camera info
            info_sub_ = nh_.subscribe("camera_node/camera_info", 1, &CameraReader::camera_info_callback, this);

            cv::namedWindow(OPENCV_WINDOW);
        }

        ~CameraReader()
        {
            cv::destroyWindow(OPENCV_WINDOW);
        }

        void camera_info_callback(const sensor_msgs::CameraInfoPtr& info)
        {
            if (intrinsics_initialized_)
            {
                ROS_INFO("shouldn't be here");
                return;
            }
            cv::Mat camera_matrix(3, 3, CV_64FC1, &(info->K));
            cv::Mat distortion(5, 1, CV_64FC1);
            for (int i = 0; i < 5; ++i)
            {
                distortion.at<double>(i, 0) = (info->D)[i];
            }
            cv::Mat R = cv::Mat::eye(3, 3, CV_64FC1);
            cv::Size size(info->width, info->height);
            cv::Mat new_camera_matrix = cv::getOptimalNewCameraMatrix(camera_matrix, distortion, size, 0);
            cv::initUndistortRectifyMap(camera_matrix, distortion, R, new_camera_matrix, size, CV_32FC1, map1_, map2_);
            intrinsics_initialized_ = true;
            info_sub_.shutdown();
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

            if (!intrinsics_initialized_)
            {
                return;
            }

            cv::Mat img2;
            cv::remap(cv_ptr->image, img2, map1_, map2_, cv::INTER_LINEAR);

            // Update GUI Window
            cv::imshow(OPENCV_WINDOW, cv_ptr->image);
            cv::imshow(OPENCV_WINDOW_2, img2);
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
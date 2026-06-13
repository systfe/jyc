
#include "ros2_compat.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp> 

using namespace cv;
using namespace std;

// HSV阈值
static int iLowH = 0;        // 色调
static int iHighH = 180;     

static int iLowS = 0;        // 饱和度
static int iHighS = 30;      

static int iLowV = 210;      // 亮度要
static int iHighV = 255;

void Cam_RGB_Callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    //ROS_INFO("Cam_RGB_Callback");
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    Mat imgOriginal = cv_ptr->image;
    
    //将RGB图片转换成HSV
    Mat imgHSV;
    vector<Mat> hsvSplit;
    cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);

    Mat imgThresholded;

    //1. 使用上面的色相、饱和度和亮度阈值范围对图像进行二值化
    Mat maskHSV;
    inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), maskHSV); 

    // 2. RGB空间的白色检测
    Mat maskRGB;
    inRange(imgOriginal, Scalar(200, 200, 200), Scalar(255, 255, 255), maskRGB);

    imgThresholded = maskHSV & maskRGB;  // 使用与运算,要求同时满足两个条件

    //显示处理结果
    imshow("RGB", imgOriginal);
    imshow("Result", imgThresholded);
    cv::waitKey(1);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
   
    auto node = std::make_shared<rclcpp::Node>("hsv_adjust");
    std::string image_topic = node->declare_parameter<std::string>("image_topic", "/robot/image_raw");
    auto rgb_sub = node->create_subscription<sensor_msgs::msg::Image>(
        image_topic, 1, Cam_RGB_Callback);

    rclcpp::WallRate loop_rate(30);

    //生成图像显示和参数调节的窗口空见
    namedWindow("Threshold", WINDOW_AUTOSIZE);

    createTrackbar("LowH", "Threshold", &iLowH, 179); // 色相，范围 0-179
    createTrackbar("HighH", "Threshold", &iHighH, 179);

    createTrackbar("LowS", "Threshold", &iLowS, 255); // 饱和度，范围 0-255
    createTrackbar("HighS", "Threshold", &iHighS, 255);

    createTrackbar("LowV", "Threshold", &iLowV, 255); // 亮度，范围 0-255
    createTrackbar("HighV", "Threshold", &iHighV, 255);

    namedWindow("RGB"); 
    namedWindow("Result"); 
    while (rclcpp::ok())
    {
        
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    rclcpp::shutdown();
}

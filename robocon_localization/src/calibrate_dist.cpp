#include "ros2_compat.h"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <cmath>

using namespace cv;

void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    Mat image_raw = cv_ptr->image;

    Mat image_hsv;
    cv::cvtColor(image_raw, image_hsv, COLOR_BGR2HSV);

    const int center_x = image_hsv.cols / 2;
    const int center_y = image_hsv.rows / 2;
    int distance = -1;
    Point red_point(-1, -1);

    Mat image_show = image_raw.clone(); // Create a copy of the raw image for display

    for (int y = 0; y < image_hsv.rows; ++y) {
        for (int x = 0; x < image_hsv.cols; ++x) {
            Vec3b pixel = image_hsv.at<Vec3b>(y, x);
            const bool is_red =
                ((pixel[0] <= 10) || (pixel[0] >= 160 && pixel[0] <= 180)) &&
                pixel[1] >= 100 &&
                pixel[2] >= 100;
            if (!is_red) {
                continue;
            }

            const int dx = x - center_x;
            const int dy = y - center_y;
            const int pixel_distance = static_cast<int>(std::sqrt(dx * dx + dy * dy));
            if (distance == -1 || pixel_distance < distance) {
                distance = pixel_distance;
                red_point = Point(x, y);
            }
        }
    }

    if (distance != -1) {
        ROS_INFO("红球像素距离: %d pixels", distance);
        int cross_size = 10;
        line(image_show, Point(red_point.x - cross_size, red_point.y), Point(red_point.x + cross_size, red_point.y), Scalar(0, 255, 255), 2);
        line(image_show, Point(red_point.x, red_point.y - cross_size), Point(red_point.x, red_point.y + cross_size), Scalar(0, 255, 255), 2);
        line(image_show, Point(center_x, center_y), red_point, Scalar(0, 255, 0), 2);

    } else {
        ROS_INFO("未扫描到红色");
    }

    // Display the image in a window
    cv::imshow("result", image_show);
    cv::waitKey(30);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("calibrate_dist");

    std::string image_topic = node->declare_parameter<std::string>("image_topic", "/robot/image_raw");
    auto sub = node->create_subscription<sensor_msgs::msg::Image>(
        image_topic, rclcpp::SensorDataQoS(), imageCallback);

    cv::namedWindow("result", cv::WINDOW_AUTOSIZE);
    rclcpp::spin(node);

    cv::destroyWindow("result");
    rclcpp::shutdown();
    return 0;
}

#include "ros2_compat.h"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <chrono>
#include <string>

using namespace cv;
using namespace std::chrono_literals;

static bool received_image = false;

void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    received_image = true;

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

    Mat image_show = image_raw.clone(); // 拷贝原始图像用于显示

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

    // 在窗口中显示图像
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
    ROS_INFO("距离标定节点已启动，正在订阅图像话题: %s", image_topic.c_str());
    ROS_INFO("如果窗口一直显示等待图像，请先启动 RTSP 相机或确认 /robot/image_raw 有数据。");

    int wait_ticks = 0;
    auto wait_timer = node->create_wall_timer(100ms, [node, image_topic, &wait_ticks]() {
        if (received_image) {
            return;
        }

        cv::Mat waiting(260, 760, CV_8UC3, cv::Scalar(0, 0, 0));
        cv::putText(
            waiting,
            "waiting for image topic:",
            cv::Point(35, 105),
            cv::FONT_HERSHEY_SIMPLEX,
            0.9,
            cv::Scalar(0, 255, 255),
            2,
            cv::LINE_AA);
        cv::putText(
            waiting,
            image_topic,
            cv::Point(35, 155),
            cv::FONT_HERSHEY_SIMPLEX,
            0.8,
            cv::Scalar(180, 180, 180),
            2,
            cv::LINE_AA);
        cv::putText(
            waiting,
            "start camera publisher first",
            cv::Point(35, 210),
            cv::FONT_HERSHEY_SIMPLEX,
            0.75,
            cv::Scalar(180, 180, 180),
            2,
            cv::LINE_AA);
        cv::imshow("result", waiting);
        cv::waitKey(1);

        wait_ticks++;
        if (wait_ticks % 20 == 0) {
            RCLCPP_WARN(
                node->get_logger(),
                "还没有收到图像，请确认 %s 正在发布",
                image_topic.c_str());
        }
    });

    rclcpp::spin(node);

    cv::destroyWindow("result");
    rclcpp::shutdown();
    return 0;
}

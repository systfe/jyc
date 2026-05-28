#include "ros2_compat.h"

#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <algorithm>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <string>

class OmniMirrorSim : public rclcpp::Node {
public:
    OmniMirrorSim() : Node("omni_mirror_sim") {
        input_topic_ = declare_parameter<std::string>("input_topic", "/sim_camera/image_raw");
        output_topic_ = declare_parameter<std::string>("output_topic", "/omni_camera/image_raw");
        output_width_ = declare_parameter<int>("output_width", 640);
        output_height_ = declare_parameter<int>("output_height", 480);
        inner_radius_ratio_ = declare_parameter<double>("inner_radius_ratio", 0.05);
        outer_radius_ratio_ = declare_parameter<double>("outer_radius_ratio", 0.98);
        mirror_curve_ = declare_parameter<double>("mirror_curve", 1.75);
        source_radius_ratio_ = declare_parameter<double>("source_radius_ratio", 0.98);
        radial_k1_ = declare_parameter<double>("radial_k1", 0.0);
        radial_k2_ = declare_parameter<double>("radial_k2", 0.0);
        center_x_ratio_ = declare_parameter<double>("center_x_ratio", 0.5);
        center_y_ratio_ = declare_parameter<double>("center_y_ratio", 0.5);
        flip_azimuth_ = declare_parameter<bool>("flip_azimuth", false);
        flip_radial_ = declare_parameter<bool>("flip_radial", false);

        publisher_ = create_publisher<sensor_msgs::msg::Image>(output_topic_, 1);
        subscription_ = create_subscription<sensor_msgs::msg::Image>(
            input_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&OmniMirrorSim::imageCallback, this, std::placeholders::_1));

        RCLCPP_INFO(
            get_logger(),
            "omni_mirror_sim: %s -> %s (%dx%d)",
            input_topic_.c_str(),
            output_topic_.c_str(),
            output_width_,
            output_height_);
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat omni(output_height_, output_width_, CV_8UC3, cv::Scalar(0, 0, 0));
        renderMirrorImage(cv_ptr->image, omni);

        auto out_msg = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, omni).toImageMsg();
        out_msg->header.frame_id = "camera_link";
        publisher_->publish(*out_msg);
    }

    void renderMirrorImage(const cv::Mat& src, cv::Mat& dst) {
        const float cx = (dst.cols - 1) * static_cast<float>(center_x_ratio_);
        const float cy = (dst.rows - 1) * static_cast<float>(center_y_ratio_);
        const float max_radius = std::min(dst.cols, dst.rows) * 0.5f * outer_radius_ratio_;
        const float min_radius = max_radius * inner_radius_ratio_;
        const float usable_radius = std::max(1.0f, max_radius - min_radius);

        for (int y = 0; y < dst.rows; ++y) {
            for (int x = 0; x < dst.cols; ++x) {
                const float dx = x - cx;
                const float dy = y - cy;
                const float radius = std::sqrt(dx * dx + dy * dy);
                if (radius < min_radius || radius > max_radius) {
                    continue;
                }

                const float radial_norm = std::clamp((radius - min_radius) / usable_radius, 0.0f, 1.0f);
                float curved_radial = std::pow(radial_norm, static_cast<float>(mirror_curve_));
                curved_radial =
                    curved_radial *
                    (1.0f + static_cast<float>(radial_k1_) * curved_radial * curved_radial +
                     static_cast<float>(radial_k2_) * std::pow(curved_radial, 4.0f));
                curved_radial = std::clamp(curved_radial, 0.0f, 1.0f);
                if (flip_radial_) {
                    curved_radial = 1.0f - curved_radial;
                }

                float dir_x = dx / std::max(radius, 1.0f);
                const float dir_y = dy / std::max(radius, 1.0f);
                if (flip_azimuth_) {
                    dir_x = -dir_x;
                }

                const float src_cx = (src.cols - 1) * 0.5f;
                const float src_cy = (src.rows - 1) * 0.5f;
                const float src_radius =
                    std::min(src.cols, src.rows) * 0.5f * source_radius_ratio_ * curved_radial;
                const float src_x = src_cx + dir_x * src_radius;
                const float src_y = src_cy + dir_y * src_radius;

                if (src_x < 0.0f || src_x >= src.cols - 1 || src_y < 0.0f || src_y >= src.rows - 1) {
                    continue;
                }

                const int x0 = static_cast<int>(src_x);
                const int y0 = static_cast<int>(src_y);
                const float ax = src_x - x0;
                const float ay = src_y - y0;

                const cv::Vec3b& p00 = src.at<cv::Vec3b>(y0, x0);
                const cv::Vec3b& p10 = src.at<cv::Vec3b>(y0, x0 + 1);
                const cv::Vec3b& p01 = src.at<cv::Vec3b>(y0 + 1, x0);
                const cv::Vec3b& p11 = src.at<cv::Vec3b>(y0 + 1, x0 + 1);

                cv::Vec3b pixel;
                for (int c = 0; c < 3; ++c) {
                    const float top = p00[c] * (1.0f - ax) + p10[c] * ax;
                    const float bottom = p01[c] * (1.0f - ax) + p11[c] * ax;
                    pixel[c] = static_cast<uchar>(top * (1.0f - ay) + bottom * ay);
                }

                dst.at<cv::Vec3b>(y, x) = pixel;
            }
        }
    }

    std::string input_topic_;
    std::string output_topic_;
    int output_width_;
    int output_height_;
    double inner_radius_ratio_;
    double outer_radius_ratio_;
    double mirror_curve_;
    double source_radius_ratio_;
    double radial_k1_;
    double radial_k2_;
    double center_x_ratio_;
    double center_y_ratio_;
    bool flip_azimuth_;
    bool flip_radial_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OmniMirrorSim>());
    rclcpp::shutdown();
    return 0;
}

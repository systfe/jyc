#include "ros2_compat.h"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <array>
#include <cstdint>
#include <cmath>
#include <initializer_list>
#include <string>
#include <vector>
#include <builtin_interfaces/msg/time.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "distance_lookup.h"
#include "lines_map.h"
#include "lines_matcher.h"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace cv;

static DistanceLookup distanceLookup;
static LinesMap lines_map;
static float counter_x = 0;
static float counter_y = 0;
static float counter_yaw = 0;
static float odom_x = 0;
static float odom_y = 0;
static float odom_yaw = 0;
static bool odom_ready = false;
static bool odom_origin_ready = false;
static float odom_zero_x = 0;
static float odom_zero_y = 0;
static float odom_zero_yaw = 0;
static bool vision_initialized = false;
static float initial_counter_x = 0;
static float initial_counter_y = 0;
static float initial_counter_yaw = 0;
static bool use_odom_for_tracking = false;
static bool use_safety_axis_constraint = false;
static bool last_tracking_match_valid = false;
static int last_tracking_score = 0;
static int last_tracking_matched = 0;
static int last_tracking_features = 0;
static cv::Mat field_image;
static cv::Mat monitor_image;
static cv::Mat image_lines_all;
static cv::Mat image_lines_map;
static cv::Mat image_red_map;
static cv::Mat image_blue_map;
static cv::Mat image_magenta_map;
static cv::Mat image_purple_map;
static cv::Mat image_black_map;
static constexpr float FIELD_SIZE_METERS = 3.0f;
static constexpr int MONITOR_INFO_HEIGHT = 96;
static rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
static std::string pose_frame_id = "map";

struct AxisEstimate {
    bool valid = false;
    float angle_deg = 0.0f;
    float confidence = 0.0f;
};

enum class MatchFocus {
    Position,
    Yaw,
    Initial,
};

static AxisEstimate red_map_axis;
static AxisEstimate blue_map_axis;

struct ColorGate {
    std::vector<int64_t> rgb_reference;
    int hue_tolerance = 10;
    int saturation_lower_margin = 80;
    int saturation_upper_margin = 80;
    int value_lower_margin = 80;
    int value_upper_margin = 80;
    cv::Scalar bgr_lower_margin;
    cv::Scalar bgr_upper_margin;
    bool use_full_hue = false;
    bool use_bgr_gate = false;
    cv::Scalar hsv_lower;
    cv::Scalar hsv_upper;
    cv::Scalar bgr_lower;
    cv::Scalar bgr_upper;
};

struct ColorThresholdConfig {
    ColorGate red;
    ColorGate blue;
    ColorGate purple;
    ColorGate magenta;
    ColorGate black;
};

static ColorThresholdConfig color_thresholds = {
    {{250, 74, 59}, 8, 75, 60, 100, 20, cv::Scalar(255, 255, 255), cv::Scalar(255, 255, 255), false, false, cv::Scalar(), cv::Scalar(), cv::Scalar(), cv::Scalar()},
    {{92, 168, 222}, 8, 110, 65, 50, 35, cv::Scalar(77, 73, 52), cv::Scalar(33, 87, 78), false, true, cv::Scalar(), cv::Scalar(), cv::Scalar(), cv::Scalar()},
    {{1, 52, 108}, 21, 148, 10, 83, 147, cv::Scalar(48, 32, 1), cv::Scalar(147, 98, 79), false, true, cv::Scalar(), cv::Scalar(), cv::Scalar(), cv::Scalar()},
    {{255, 25, 255}, 20, 185, 25, 195, 0, cv::Scalar(255, 255, 255), cv::Scalar(255, 255, 255), false, false, cv::Scalar(), cv::Scalar(), cv::Scalar(), cv::Scalar()},
    {{0, 0, 0}, 0, 0, 80, 0, 55, cv::Scalar(255, 255, 255), cv::Scalar(255, 255, 255), true, false, cv::Scalar(), cv::Scalar(), cv::Scalar(), cv::Scalar()},
};

static int clampToByte(int64_t value)
{
    return static_cast<int>(std::clamp<int64_t>(value, 0, 255));
}

static std::vector<int64_t> sanitizeRgbReference(const rclcpp::Node::SharedPtr& node,
                                                 const std::string& name,
                                                 const std::vector<int64_t>& default_value,
                                                 const std::vector<int64_t>& values)
{
    if (values.size() != 3) {
        RCLCPP_WARN(
            node->get_logger(),
            "颜色参考值 %s 必须是 3 个整数，当前配置无效，继续使用默认值",
            name.c_str());
        return default_value;
    }

    return {
        clampToByte(values[0]),
        clampToByte(values[1]),
        clampToByte(values[2]),
    };
}

static cv::Scalar rgbReferenceToBgr(const std::vector<int64_t>& rgb)
{
    return cv::Scalar(rgb[2], rgb[1], rgb[0]);
}

static cv::Scalar rgbReferenceToHsv(const std::vector<int64_t>& rgb)
{
    cv::Mat bgr(1, 1, CV_8UC3);
    bgr.at<cv::Vec3b>(0, 0) = cv::Vec3b(
        static_cast<uchar>(rgb[2]),
        static_cast<uchar>(rgb[1]),
        static_cast<uchar>(rgb[0]));

    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
    const cv::Vec3b pixel = hsv.at<cv::Vec3b>(0, 0);
    return cv::Scalar(pixel[0], pixel[1], pixel[2]);
}

static int shiftedHue(int hue, int delta)
{
    int shifted = (hue + delta) % 180;
    if (shifted < 0) {
        shifted += 180;
    }
    return shifted;
}

static cv::Scalar lowerBgrFromReference(const cv::Scalar& bgr, const cv::Scalar& margin)
{
    return cv::Scalar(
        std::max(0.0, bgr[0] - margin[0]),
        std::max(0.0, bgr[1] - margin[1]),
        std::max(0.0, bgr[2] - margin[2]));
}

static cv::Scalar upperBgrFromReference(const cv::Scalar& bgr, const cv::Scalar& margin)
{
    return cv::Scalar(
        std::min(255.0, bgr[0] + margin[0]),
        std::min(255.0, bgr[1] + margin[1]),
        std::min(255.0, bgr[2] + margin[2]));
}

static void generateThresholdsFromReference(ColorGate& gate)
{
    const cv::Scalar hsv = rgbReferenceToHsv(gate.rgb_reference);
    const int reference_hue = static_cast<int>(hsv[0]);

    const int lower_hue = gate.use_full_hue ?
        0 :
        shiftedHue(reference_hue, -gate.hue_tolerance);
    const int upper_hue = gate.use_full_hue ?
        179 :
        shiftedHue(reference_hue, gate.hue_tolerance);

    gate.hsv_lower = cv::Scalar(
        lower_hue,
        std::max(0.0, hsv[1] - gate.saturation_lower_margin),
        std::max(0.0, hsv[2] - gate.value_lower_margin));
    gate.hsv_upper = cv::Scalar(
        upper_hue,
        std::min(255.0, hsv[1] + gate.saturation_upper_margin),
        std::min(255.0, hsv[2] + gate.value_upper_margin));

    const cv::Scalar bgr = rgbReferenceToBgr(gate.rgb_reference);
    gate.bgr_lower = lowerBgrFromReference(bgr, gate.bgr_lower_margin);
    gate.bgr_upper = upperBgrFromReference(bgr, gate.bgr_upper_margin);
}

static ColorGate readColorGate(const rclcpp::Node::SharedPtr& node,
                               const std::string& prefix,
                               const ColorGate& default_gate)
{
    ColorGate gate;
    gate = default_gate;
    const std::string rgb_param = prefix + "_rgb_reference";
    const std::vector<int64_t> configured_rgb =
        node->declare_parameter<std::vector<int64_t>>(rgb_param, default_gate.rgb_reference);
    gate.rgb_reference = sanitizeRgbReference(node, rgb_param, default_gate.rgb_reference, configured_rgb);
    generateThresholdsFromReference(gate);
    return gate;
}

static void loadColorThresholds(const rclcpp::Node::SharedPtr& node)
{
    color_thresholds.red = readColorGate(node, "red", color_thresholds.red);
    color_thresholds.blue = readColorGate(node, "blue", color_thresholds.blue);
    color_thresholds.purple = readColorGate(node, "purple", color_thresholds.purple);
    color_thresholds.magenta = readColorGate(node, "magenta", color_thresholds.magenta);
    color_thresholds.black = readColorGate(node, "black", color_thresholds.black);
    RCLCPP_INFO(node->get_logger(), "颜色参考值已加载，可在 color_reference.yaml 中调试");
}

static cv::Mat makeHsvMask(const cv::Mat& hsv_image, const cv::Scalar& lower, const cv::Scalar& upper)
{
    cv::Mat mask;
    if (lower[0] <= upper[0]) {
        cv::inRange(hsv_image, lower, upper, mask);
        return mask;
    }

    cv::Mat low_hue_mask;
    cv::Mat high_hue_mask;
    cv::inRange(
        hsv_image,
        cv::Scalar(0, lower[1], lower[2]),
        cv::Scalar(upper[0], upper[1], upper[2]),
        low_hue_mask);
    cv::inRange(
        hsv_image,
        cv::Scalar(lower[0], lower[1], lower[2]),
        cv::Scalar(179, upper[1], upper[2]),
        high_hue_mask);
    cv::bitwise_or(low_hue_mask, high_hue_mask, mask);
    return mask;
}

static cv::Mat makeColorMask(const cv::Mat& bgr_image,
                             const cv::Mat& hsv_image,
                             const ColorGate& gate)
{
    cv::Mat mask = makeHsvMask(hsv_image, gate.hsv_lower, gate.hsv_upper);
    if (!gate.use_bgr_gate) {
        return mask;
    }

    cv::Mat bgr_gate;
    cv::inRange(bgr_image, gate.bgr_lower, gate.bgr_upper, bgr_gate);
    cv::bitwise_and(mask, bgr_gate, mask);
    return mask;
}

static void createSeparatedBluePurpleMasks(const cv::Mat& bgr_image,
                                           const cv::Mat& hsv_image,
                                           cv::Mat& blue_mask,
                                           cv::Mat& purple_mask)
{
    // 蓝色和紫色的色相很接近，主要靠亮度和 BGR 通道形状区分：
    // 蓝色更偏浅青，紫色的红色通道保持较低。
    blue_mask = makeColorMask(bgr_image, hsv_image, color_thresholds.blue);
    purple_mask = makeColorMask(bgr_image, hsv_image, color_thresholds.purple);

    cv::Mat not_purple;
    cv::bitwise_not(purple_mask, not_purple);
    cv::bitwise_and(blue_mask, not_purple, blue_mask);
}

static float pixelsPerMeterX()
{
    return image_lines_map.empty() ? 1.0f : image_lines_map.cols / FIELD_SIZE_METERS;
}

static float pixelsPerMeterY()
{
    return image_lines_map.empty() ? 1.0f : image_lines_map.rows / FIELD_SIZE_METERS;
}

static float maxPoseX()
{
    return image_lines_map.empty() ? 0.0f : image_lines_map.cols / 2.0f;
}

static float maxPoseY()
{
    return image_lines_map.empty() ? 0.0f : image_lines_map.rows / 2.0f;
}

static bool isPoseInsideField(float pose_x, float pose_y)
{
    if (image_lines_map.empty()) {
        return true;
    }

    return pose_x >= -maxPoseX() && pose_x <= maxPoseX() &&
           pose_y >= -maxPoseY() && pose_y <= maxPoseY();
}

static geometry_msgs::msg::PoseStamped makePoseMessage(const builtin_interfaces::msg::Time& stamp)
{
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = pose_frame_id;

    pose_msg.pose.position.x = counter_x / pixelsPerMeterX();
    pose_msg.pose.position.y = -counter_y / pixelsPerMeterY();
    pose_msg.pose.position.z = 0.0;

    const double yaw_rad = -counter_yaw * M_PI / 180.0;
    pose_msg.pose.orientation.x = 0.0;
    pose_msg.pose.orientation.y = 0.0;
    pose_msg.pose.orientation.z = std::sin(yaw_rad / 2.0);
    pose_msg.pose.orientation.w = std::cos(yaw_rad / 2.0);

    return pose_msg;
}

static void publishRobotPose(const builtin_interfaces::msg::Time& stamp)
{
    if (!pose_pub) {
        return;
    }

    pose_pub->publish(makePoseMessage(stamp));
}

static void clampPoseInsideField(float& pose_x, float& pose_y)
{
    if (image_lines_map.empty()) {
        return;
    }

    pose_x = std::clamp(pose_x, -maxPoseX(), maxPoseX());
    pose_y = std::clamp(pose_y, -maxPoseY(), maxPoseY());
}

static float normalizeAngleDeg(float angle)
{
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

static float normalizeAxisAngleDeg(float angle)
{
    angle = normalizeAngleDeg(angle);
    while (angle > 90.0f) {
        angle -= 180.0f;
    }
    while (angle < -90.0f) {
        angle += 180.0f;
    }
    return angle;
}

static float axisAngleDiffDeg(float a, float b)
{
    return std::abs(normalizeAxisAngleDeg(a - b));
}

static AxisEstimate estimateAxisFromPoints(const std::vector<cv::Point2f>& points)
{
    AxisEstimate axis;
    if (points.size() < 8) {
        return axis;
    }

    cv::Point2f mean(0.0f, 0.0f);
    for (const auto& point : points) {
        mean += point;
    }
    mean.x /= static_cast<float>(points.size());
    mean.y /= static_cast<float>(points.size());

    double cov_xx = 0.0;
    double cov_xy = 0.0;
    double cov_yy = 0.0;
    for (const auto& point : points) {
        const double x = point.x - mean.x;
        const double y = point.y - mean.y;
        cov_xx += x * x;
        cov_xy += x * y;
        cov_yy += y * y;
    }
    cov_xx /= static_cast<double>(points.size());
    cov_xy /= static_cast<double>(points.size());
    cov_yy /= static_cast<double>(points.size());

    const double trace = cov_xx + cov_yy;
    const double delta = std::sqrt(std::max(0.0, (cov_xx - cov_yy) * (cov_xx - cov_yy) + 4.0 * cov_xy * cov_xy));
    const double lambda_major = 0.5 * (trace + delta);
    const double lambda_minor = 0.5 * (trace - delta);
    if (lambda_major < 1e-6) {
        return axis;
    }

    const double confidence = (lambda_major - lambda_minor) / lambda_major;
    if (confidence < 0.35) {
        return axis;
    }

    axis.valid = true;
    axis.confidence = static_cast<float>(confidence);
    axis.angle_deg = normalizeAxisAngleDeg(
        static_cast<float>(0.5 * std::atan2(2.0 * cov_xy, cov_xx - cov_yy) * 180.0 / CV_PI));
    return axis;
}

static AxisEstimate estimateAxisFromMap(const cv::Mat& map)
{
    std::vector<cv::Point2f> points;
    points.reserve(static_cast<size_t>(map.rows * map.cols / 16));

    for (int y = 0; y < map.rows; ++y) {
        for (int x = 0; x < map.cols; ++x) {
            if (map.at<uchar>(y, x) >= 240) {
                points.push_back(cv::Point2f(static_cast<float>(x), static_cast<float>(y)));
            }
        }
    }

    return estimateAxisFromPoints(points);
}

static int scoreSafetyAxisConsistency(const std::vector<cv::Point2f>& points,
                                      const AxisEstimate& map_axis,
                                      float pose_yaw,
                                      MatchFocus focus)
{
    const bool automatic_axis_constraint = points.size() >= 12;
    if (!use_safety_axis_constraint && !automatic_axis_constraint) {
        return 0;
    }

    if (!map_axis.valid) {
        return 0;
    }

    AxisEstimate observed_axis = estimateAxisFromPoints(points);
    if (!observed_axis.valid) {
        return 0;
    }
    if (!use_safety_axis_constraint && observed_axis.confidence < 0.50f) {
        return 0;
    }

    const float transformed_axis = normalizeAxisAngleDeg(observed_axis.angle_deg + pose_yaw);
    const float diff = axisAngleDiffDeg(transformed_axis, map_axis.angle_deg);
    const float tolerance = use_safety_axis_constraint ? 25.0f : 18.0f;
    if (diff <= tolerance) {
        return static_cast<int>((tolerance - diff) * observed_axis.confidence * 40.0f);
    }

    const float penalty_scale = focus == MatchFocus::Yaw ? 180.0f : 110.0f;
    return -static_cast<int>((diff - tolerance) * observed_axis.confidence * penalty_scale);
}

static cv::Mat createDirectionalGradientFromMask(const cv::Mat& mask, int innerRadius, int outerRadius)
{
    (void)innerRadius;
    cv::Mat binary_mask;
    cv::threshold(mask, binary_mask, 0, 255, cv::THRESH_BINARY);

    cv::Mat edge_mask;
    cv::morphologyEx(
        binary_mask,
        edge_mask,
        cv::MORPH_GRADIENT,
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
    cv::threshold(edge_mask, edge_mask, 0, 255, cv::THRESH_BINARY);

    cv::Mat inverted_edge_mask;
    cv::bitwise_not(edge_mask, inverted_edge_mask);

    cv::Mat edge_distances;
    cv::distanceTransform(inverted_edge_mask, edge_distances, cv::DIST_L2, 3);

    cv::Mat gradient = cv::Mat::zeros(mask.size(), CV_8UC1);
    for (int y = 0; y < gradient.rows; ++y) {
        for (int x = 0; x < gradient.cols; ++x) {
            if (edge_mask.at<uchar>(y, x) > 0) {
                gradient.at<uchar>(y, x) = 255;
                continue;
            }

            if (outerRadius <= 0) {
                continue;
            }

            const float distance = edge_distances.at<float>(y, x);
            if (distance <= outerRadius) {
                gradient.at<uchar>(y, x) =
                    cv::saturate_cast<uchar>(255.0f * (1.0f - distance / outerRadius));
            }
        }
    }

    return gradient;
}

static cv::Mat createFilledGradientFromMask(const cv::Mat& mask,
                                            const cv::Mat& blocked_mask,
                                            int outerRadius)
{
    cv::Mat binary_mask;
    cv::threshold(mask, binary_mask, 0, 255, cv::THRESH_BINARY);

    cv::Mat edge_mask;
    cv::morphologyEx(
        binary_mask,
        edge_mask,
        cv::MORPH_GRADIENT,
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
    cv::threshold(edge_mask, edge_mask, 0, 255, cv::THRESH_BINARY);

    cv::Mat inverted_edge_mask;
    cv::bitwise_not(edge_mask, inverted_edge_mask);

    cv::Mat edge_distances;
    cv::distanceTransform(inverted_edge_mask, edge_distances, cv::DIST_L2, 3);

    cv::Mat gradient = cv::Mat::zeros(mask.size(), CV_8UC1);
    for (int y = 0; y < gradient.rows; ++y) {
        for (int x = 0; x < gradient.cols; ++x) {
            if (binary_mask.at<uchar>(y, x) > 0) {
                gradient.at<uchar>(y, x) = 255;
                continue;
            }

            if (!blocked_mask.empty() && blocked_mask.at<uchar>(y, x) > 0) {
                continue;
            }

            if (outerRadius <= 0) {
                continue;
            }

            const float distance = edge_distances.at<float>(y, x);
            if (distance <= outerRadius) {
                gradient.at<uchar>(y, x) =
                    cv::saturate_cast<uchar>(255.0f * (1.0f - distance / outerRadius));
            }
        }
    }

    return gradient;
}

static cv::Mat createFilledSafetyZoneMaskFromPurple(const cv::Mat& purple_mask)
{
    cv::Mat binary_mask;
    cv::threshold(purple_mask, binary_mask, 0, 255, cv::THRESH_BINARY);

    const int close_size =
        std::max(5, (std::min(binary_mask.cols, binary_mask.rows) / 120) | 1);
    cv::Mat closed_mask;
    cv::morphologyEx(
        binary_mask,
        closed_mask,
        cv::MORPH_CLOSE,
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(close_size, close_size)));

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(closed_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    cv::Mat filled_mask = cv::Mat::zeros(purple_mask.size(), CV_8UC1);
    const double min_area =
        static_cast<double>(purple_mask.cols * purple_mask.rows) * 0.001;
    for (size_t i = 0; i < contours.size(); ++i) {
        const cv::Rect rect = cv::boundingRect(contours[i]);
        if (cv::contourArea(contours[i]) < min_area ||
            rect.width < purple_mask.cols / 12 ||
            rect.height < purple_mask.rows / 20) {
            continue;
        }
        cv::drawContours(filled_mask, contours, static_cast<int>(i), cv::Scalar(255), cv::FILLED);
    }

    return filled_mask;
}

static cv::Mat createMagentaMap(const cv::Mat& lines_image)
{
    if (lines_image.empty()) {
        return cv::Mat();
    }

    cv::Mat hsv;
    cv::cvtColor(lines_image, hsv, cv::COLOR_BGR2HSV);

    cv::Mat magenta_mask;
    magenta_mask = makeColorMask(lines_image, hsv, color_thresholds.magenta);
    cv::morphologyEx(
        magenta_mask,
        magenta_mask,
        cv::MORPH_OPEN,
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

    const int innerRadius = std::max(6, std::max(lines_image.cols, lines_image.rows) / 100);
    const int outerRadius = std::max(24, std::max(lines_image.cols, lines_image.rows) / 28);
    return createDirectionalGradientFromMask(magenta_mask, innerRadius, outerRadius);
}

static cv::Mat createPurpleMap(const cv::Mat& lines_image)
{
    if (lines_image.empty()) {
        return cv::Mat();
    }

    cv::Mat hsv;
    cv::cvtColor(lines_image, hsv, cv::COLOR_BGR2HSV);

    cv::Mat blue_mask;
    cv::Mat purple_mask;
    createSeparatedBluePurpleMasks(lines_image, hsv, blue_mask, purple_mask);
    cv::morphologyEx(
        purple_mask,
        purple_mask,
        cv::MORPH_OPEN,
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

    const int innerRadius = std::max(6, std::max(lines_image.cols, lines_image.rows) / 100);
    const int outerRadius = std::max(24, std::max(lines_image.cols, lines_image.rows) / 28);
    (void)innerRadius;
    cv::Mat safety_zone_mask = createFilledSafetyZoneMaskFromPurple(purple_mask);
    return createFilledGradientFromMask(purple_mask, safety_zone_mask, outerRadius);
}

static cv::Mat createBlackMap(const cv::Mat& lines_image)
{
    if (lines_image.empty()) {
        return cv::Mat();
    }

    cv::Mat hsv;
    cv::cvtColor(lines_image, hsv, cv::COLOR_BGR2HSV);

    cv::Mat black_mask;
    black_mask = makeColorMask(lines_image, hsv, color_thresholds.black);
    cv::morphologyEx(
        black_mask,
        black_mask,
        cv::MORPH_OPEN,
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

    const int radius = std::max(18, std::max(lines_image.cols, lines_image.rows) / 36);
    return createDirectionalGradientFromMask(black_mask, radius, radius);
}

static void syncPoseFromOdom()
{
    if (!use_odom_for_tracking) {
        return;
    }

    if (!odom_ready) {
        return;
    }

    if (vision_initialized) {
        counter_x = initial_counter_x + (odom_x - odom_zero_x) * pixelsPerMeterX();
        counter_y = initial_counter_y - (odom_y - odom_zero_y) * pixelsPerMeterY();
        counter_yaw = initial_counter_yaw - (odom_yaw - odom_zero_yaw);
        return;
    }

    counter_x = odom_x * pixelsPerMeterX();
    counter_y = -odom_y * pixelsPerMeterY();
    counter_yaw = -odom_yaw;
}

static int scorePointsOnMap(const std::vector<cv::Point2f>& points,
                            const cv::Mat& score_map,
                            const cv::Point2f& center,
                            float pose_x,
                            float pose_y,
                            float pose_yaw)
{
    if (score_map.empty()) {
        return 0;
    }

    const float rad = pose_yaw * CV_PI / 180.0f;
    const float cos_yaw = std::cos(rad);
    const float sin_yaw = std::sin(rad);
    int score = 0;

    for (const auto& point : points) {
        const float x = point.x - center.x;
        const float y = point.y - center.y;
        const int map_x = cvRound(x * cos_yaw - y * sin_yaw + center.x + pose_x);
        const int map_y = cvRound(x * sin_yaw + y * cos_yaw + center.y + pose_y);

        int local_best = 0;
        for (int dy = -3; dy <= 3; ++dy) {
            for (int dx = -3; dx <= 3; ++dx) {
                const int sample_x = map_x + dx;
                const int sample_y = map_y + dy;
                if (sample_x >= 0 && sample_x < score_map.cols &&
                    sample_y >= 0 && sample_y < score_map.rows) {
                    local_best = std::max(local_best,
                                          static_cast<int>(score_map.at<uchar>(sample_y, sample_x)));
                }
            }
        }
        score += local_best;
    }

    return score;
}

static bool hasEnoughFeaturePoints(const std::vector<cv::Point2f>& points)
{
    return points.size() >= 3;
}

static float featureReliability(const std::vector<cv::Point2f>& points)
{
    const size_t count = points.size();
    if (count == 0) {
        return 0.0f;
    }
    if (count < 3) {
        return 0.05f;
    }
    if (count < 6) {
        return 0.25f;
    }
    if (count < 9) {
        return 0.60f;
    }
    if (count < 12) {
        return 0.85f;
    }
    return 1.0f;
}

static int reliableScore(int raw_score, const std::vector<cv::Point2f>& points)
{
    return static_cast<int>(raw_score * featureReliability(points));
}

static int totalFeaturePoints(const std::vector<cv::Point2f>& magenta_points,
                              const std::vector<cv::Point2f>& purple_points,
                              const std::vector<cv::Point2f>& black_points,
                              const std::vector<cv::Point2f>& red_points,
                              const std::vector<cv::Point2f>& blue_points)
{
    return static_cast<int>(
        magenta_points.size() +
        purple_points.size() +
        black_points.size() +
        red_points.size() +
        blue_points.size());
}

static int countMatchedPointsOnMap(const std::vector<cv::Point2f>& points,
                                   const cv::Mat& score_map,
                                   const cv::Point2f& center,
                                   float pose_x,
                                   float pose_y,
                                   float pose_yaw,
                                   int threshold = 150)
{
    if (score_map.empty()) {
        return 0;
    }

    const float rad = pose_yaw * CV_PI / 180.0f;
    const float cos_yaw = std::cos(rad);
    const float sin_yaw = std::sin(rad);
    int matched = 0;

    for (const auto& point : points) {
        const float x = point.x - center.x;
        const float y = point.y - center.y;
        const int map_x = cvRound(x * cos_yaw - y * sin_yaw + center.x + pose_x);
        const int map_y = cvRound(x * sin_yaw + y * cos_yaw + center.y + pose_y);

        int local_best = 0;
        for (int dy = -3; dy <= 3; ++dy) {
            for (int dx = -3; dx <= 3; ++dx) {
                const int sample_x = map_x + dx;
                const int sample_y = map_y + dy;
                if (sample_x >= 0 && sample_x < score_map.cols &&
                    sample_y >= 0 && sample_y < score_map.rows) {
                    local_best = std::max(local_best,
                                          static_cast<int>(score_map.at<uchar>(sample_y, sample_x)));
                }
            }
        }

        if (local_best >= threshold) {
            ++matched;
        }
    }

    return matched;
}

static int minimumRefineScore(size_t point_count)
{
    return std::max(360, static_cast<int>(std::min<size_t>(point_count, 10) * 90));
}

struct MatchQuality {
    int score = 0;
    int matched = 0;
    int features = 0;
};

static int scorePose(const std::vector<cv::Point2f>& white_points,
                     const std::vector<cv::Point2f>& magenta_points,
                     const std::vector<cv::Point2f>& purple_points,
                     const std::vector<cv::Point2f>& black_points,
                     const std::vector<cv::Point2f>& red_points,
                     const std::vector<cv::Point2f>& blue_points,
                     const cv::Point2f& center,
                     float pose_x,
                     float pose_y,
                     float pose_yaw,
                     MatchFocus focus)
{
    (void)white_points;
    if (!isPoseInsideField(pose_x, pose_y)) {
        return -1;
    }

    const int magenta_score = reliableScore(
        scorePointsOnMap(magenta_points, image_magenta_map, center, pose_x, pose_y, pose_yaw),
        magenta_points);
    const int purple_score = reliableScore(
        scorePointsOnMap(purple_points, image_purple_map, center, pose_x, pose_y, pose_yaw),
        purple_points);
    const int black_score = reliableScore(
        scorePointsOnMap(black_points, image_black_map, center, pose_x, pose_y, pose_yaw),
        black_points);
    const int red_score =
        reliableScore(scorePointsOnMap(red_points, image_red_map, center, pose_x, pose_y, pose_yaw), red_points);
    const int blue_score =
        reliableScore(scorePointsOnMap(blue_points, image_blue_map, center, pose_x, pose_y, pose_yaw), blue_points);
    const int safety_axis_score =
        scoreSafetyAxisConsistency(red_points, red_map_axis, pose_yaw, focus) +
        scoreSafetyAxisConsistency(blue_points, blue_map_axis, pose_yaw, focus);

    return magenta_score + purple_score + black_score + red_score + blue_score + safety_axis_score;
}

static MatchQuality evaluatePoseQuality(const std::vector<cv::Point2f>& white_points,
                                        const std::vector<cv::Point2f>& magenta_points,
                                        const std::vector<cv::Point2f>& purple_points,
                                        const std::vector<cv::Point2f>& black_points,
                                        const std::vector<cv::Point2f>& red_points,
                                        const std::vector<cv::Point2f>& blue_points,
                                        const cv::Point2f& center,
                                        float pose_x,
                                        float pose_y,
                                        float pose_yaw,
                                        MatchFocus focus)
{
    MatchQuality quality;
    quality.features = totalFeaturePoints(magenta_points, purple_points, black_points, red_points, blue_points);
    quality.matched =
        countMatchedPointsOnMap(magenta_points, image_magenta_map, center, pose_x, pose_y, pose_yaw) +
        countMatchedPointsOnMap(purple_points, image_purple_map, center, pose_x, pose_y, pose_yaw) +
        countMatchedPointsOnMap(black_points, image_black_map, center, pose_x, pose_y, pose_yaw) +
        countMatchedPointsOnMap(red_points, image_red_map, center, pose_x, pose_y, pose_yaw) +
        countMatchedPointsOnMap(blue_points, image_blue_map, center, pose_x, pose_y, pose_yaw);
    quality.score = scorePose(
        white_points, magenta_points, purple_points, black_points, red_points, blue_points,
        center, pose_x, pose_y, pose_yaw, focus);
    return quality;
}

static bool isMatchQualityAcceptable(const MatchQuality& quality)
{
    if (quality.features < 6) {
        return false;
    }

    const int minimum_matched =
        std::clamp(quality.features / 8, 3, 9);
    if (quality.matched < minimum_matched) {
        return false;
    }

    const float matched_ratio =
        static_cast<float>(quality.matched) / static_cast<float>(std::max(quality.features, 1));
    return matched_ratio >= 0.10f && quality.score > 0;
}

static void refineYawWithSidelinePoints(const std::vector<cv::Point2f>& red_points,
                                        const std::vector<cv::Point2f>& blue_points,
                                        const cv::Point2f& center,
                                        float pose_x,
                                        float pose_y,
                                        float& pose_yaw)
{
    if (!hasEnoughFeaturePoints(red_points) && !hasEnoughFeaturePoints(blue_points)) {
        return;
    }

    int last_sum = 0;
    const int minimum_sum = minimumRefineScore(red_points.size() + blue_points.size());
    for (int i = 0; i < 20; ++i) {
        int best_sum = 0;
        float best_angle = 0.0f;
        for (float angle = -1.0f; angle <= 1.0f; angle += 1.0f) {
            const float test_yaw = pose_yaw + angle;
            const int sum =
                reliableScore(scorePointsOnMap(red_points, image_red_map, center, pose_x, pose_y, test_yaw), red_points) +
                reliableScore(scorePointsOnMap(blue_points, image_blue_map, center, pose_x, pose_y, test_yaw), blue_points);
            if (sum > best_sum) {
                best_sum = sum;
                best_angle = angle;
            }
        }

        if (best_sum < minimum_sum || best_sum <= last_sum || best_angle == 0.0f) {
            break;
        }

        pose_yaw += best_angle;
        last_sum = best_sum;
    }
}

static void refinePoseWithDetectedPoints(const std::vector<cv::Point2f>& white_points,
                                         const std::vector<cv::Point2f>& magenta_points,
                                         const std::vector<cv::Point2f>& purple_points,
                                         const std::vector<cv::Point2f>& black_points,
                                         const std::vector<cv::Point2f>& red_points,
                                         const std::vector<cv::Point2f>& blue_points,
                                         const cv::Point2f& center,
                                         float& pose_x,
                                         float& pose_y,
                                         float& pose_yaw)
{
    refineYawWithSidelinePoints(red_points, blue_points, center, pose_x, pose_y, pose_yaw);

    if (hasEnoughFeaturePoints(magenta_points)) {
        int last_sum = 0;
        for (int i = 0; i < 40; ++i) {
            MatchResult magenta_match = LinesMatcher::refineMatchWithWhitePoints(
                magenta_points, image_magenta_map, center, pose_x, pose_y, pose_yaw);

            if (magenta_match.max_sum < minimumRefineScore(magenta_points.size()) ||
                magenta_match.max_sum <= last_sum) {
                break;
            }

            pose_x += magenta_match.best_dx;
            pose_y += magenta_match.best_dy;
            clampPoseInsideField(pose_x, pose_y);
            pose_yaw += magenta_match.best_angle;
            last_sum = magenta_match.max_sum;
        }
    }

    if (hasEnoughFeaturePoints(purple_points)) {
        int last_sum = 0;
        for (int i = 0; i < 40; ++i) {
            MatchResult purple_match = LinesMatcher::refineMatchWithWhitePoints(
                purple_points, image_purple_map, center, pose_x, pose_y, pose_yaw);

            if (purple_match.max_sum < minimumRefineScore(purple_points.size()) ||
                purple_match.max_sum <= last_sum) {
                break;
            }

            pose_x += purple_match.best_dx;
            pose_y += purple_match.best_dy;
            clampPoseInsideField(pose_x, pose_y);
            pose_yaw += purple_match.best_angle;
            last_sum = purple_match.max_sum;
        }
    }

    if (hasEnoughFeaturePoints(black_points)) {
        int last_sum = 0;
        for (int i = 0; i < 30; ++i) {
            MatchResult black_match = LinesMatcher::refineMatchWithWhitePoints(
                black_points, image_black_map, center, pose_x, pose_y, pose_yaw);

            if (black_match.max_sum < minimumRefineScore(black_points.size()) ||
                black_match.max_sum <= last_sum) {
                break;
            }

            pose_x += black_match.best_dx;
            pose_y += black_match.best_dy;
            clampPoseInsideField(pose_x, pose_y);
            pose_yaw += black_match.best_angle;
            last_sum = black_match.max_sum;
        }
    }

    (void)white_points;
}

static void searchPoseWithWhitePoints(const std::vector<cv::Point2f>& white_points,
                                     const std::vector<cv::Point2f>& magenta_points,
                                     const std::vector<cv::Point2f>& purple_points,
                                     const std::vector<cv::Point2f>& black_points,
                                     const std::vector<cv::Point2f>& red_points,
                                      const std::vector<cv::Point2f>& blue_points,
                                      const cv::Point2f& center,
                                      int position_range,
                                      int position_step,
                                      int angle_range,
                                      int angle_step,
                                      MatchFocus focus,
                                      float& pose_x,
                                      float& pose_y,
                                      float& pose_yaw)
{
    if (magenta_points.empty() && purple_points.empty() &&
        black_points.empty() && red_points.empty() && blue_points.empty()) {
        return;
    }

    clampPoseInsideField(pose_x, pose_y);

    int best_score = scorePose(
        white_points, magenta_points, purple_points, black_points, red_points, blue_points,
        center, pose_x, pose_y, pose_yaw, focus);
    float best_x = pose_x;
    float best_y = pose_y;
    float best_yaw = pose_yaw;

    for (int dx = -position_range; dx <= position_range; dx += position_step) {
        for (int dy = -position_range; dy <= position_range; dy += position_step) {
            for (int angle = -angle_range; angle <= angle_range; angle += angle_step) {
                const float test_x = pose_x + dx;
                const float test_y = pose_y + dy;
                if (!isPoseInsideField(test_x, test_y)) {
                    continue;
                }
                const float test_yaw = pose_yaw + angle;
                const int score = scorePose(
                    white_points, magenta_points, purple_points, black_points, red_points, blue_points,
                    center, test_x, test_y, test_yaw, focus);

                if (score > best_score) {
                    best_score = score;
                    best_x = test_x;
                    best_y = test_y;
                    best_yaw = test_yaw;
                }
            }
        }
    }

    pose_x = best_x;
    pose_y = best_y;
    pose_yaw = best_yaw;
}

static void trackPoseWithDetectedPoints(const std::vector<cv::Point2f>& white_points,
                                        const std::vector<cv::Point2f>& magenta_points,
                                        const std::vector<cv::Point2f>& purple_points,
                                        const std::vector<cv::Point2f>& black_points,
                                        const std::vector<cv::Point2f>& red_points,
                                        const std::vector<cv::Point2f>& blue_points,
                                        const cv::Point2f& center,
                                        float& pose_x,
                                        float& pose_y,
                                        float& pose_yaw)
{
    const float previous_x = pose_x;
    const float previous_y = pose_y;
    const float previous_yaw = pose_yaw;

    // 三类搜索使用同一个颜色评分：每种颜色的模板匹配分直接相加。
    searchPoseWithWhitePoints(white_points, magenta_points, purple_points, black_points, red_points, blue_points, center,
                              0, 1, 18, 1, MatchFocus::Yaw, pose_x, pose_y, pose_yaw);
    refineYawWithSidelinePoints(red_points, blue_points, center, pose_x, pose_y, pose_yaw);

    float yaw_jump = std::abs(normalizeAngleDeg(pose_yaw - previous_yaw));
    if (yaw_jump > 14.0f) {
        // 大幅度 yaw 跳变通常是对称的 180 度解抢占了匹配结果。
        // 局部重试前先恢复 yaw，否则重试会继续沿着错误分支细化。
        pose_x = previous_x;
        pose_y = previous_y;
        pose_yaw = previous_yaw;
        searchPoseWithWhitePoints(white_points, magenta_points, purple_points, black_points, red_points, blue_points, center,
                                  0, 1, 8, 1, MatchFocus::Yaw, pose_x, pose_y, pose_yaw);
    }

    searchPoseWithWhitePoints(white_points, magenta_points, purple_points, black_points, red_points, blue_points, center,
                              54, 6, 10, 2, MatchFocus::Position, pose_x, pose_y, pose_yaw);
    searchPoseWithWhitePoints(white_points, magenta_points, purple_points, black_points, red_points, blue_points, center,
                              18, 2, 4, 1, MatchFocus::Position, pose_x, pose_y, pose_yaw);
    refinePoseWithDetectedPoints(white_points, magenta_points, purple_points, black_points, red_points, blue_points, center,
                                 pose_x, pose_y, pose_yaw);

    yaw_jump = std::abs(normalizeAngleDeg(pose_yaw - previous_yaw));
    const float max_position_jump = pixelsPerMeterX() * 0.37f;
    const float jump = std::hypot(pose_x - previous_x, pose_y - previous_y);
    if (jump > max_position_jump || yaw_jump > 28.0f) {
        pose_x = previous_x;
        pose_y = previous_y;
        pose_yaw = previous_yaw;
        searchPoseWithWhitePoints(white_points, magenta_points, purple_points, black_points, red_points, blue_points, center,
                                  0, 1, 10, 1, MatchFocus::Yaw, pose_x, pose_y, pose_yaw);
    }

    MatchQuality quality = evaluatePoseQuality(
        white_points, magenta_points, purple_points, black_points, red_points, blue_points,
        center, pose_x, pose_y, pose_yaw, MatchFocus::Position);
    if (!isMatchQualityAcceptable(quality)) {
        pose_x = previous_x;
        pose_y = previous_y;
        pose_yaw = previous_yaw;
        quality = evaluatePoseQuality(
            white_points, magenta_points, purple_points, black_points, red_points, blue_points,
            center, pose_x, pose_y, pose_yaw, MatchFocus::Position);
    }

    last_tracking_score = quality.score;
    last_tracking_matched = quality.matched;
    last_tracking_features = quality.features;
    last_tracking_match_valid = isMatchQualityAcceptable(quality);

    clampPoseInsideField(pose_x, pose_y);
}

static void setVisualInitialPose(float pose_x, float pose_y, float pose_yaw)
{
    initial_counter_x = pose_x;
    initial_counter_y = pose_y;
    clampPoseInsideField(initial_counter_x, initial_counter_y);
    initial_counter_yaw = pose_yaw;

    if (odom_ready) {
        odom_zero_x = odom_x;
        odom_zero_y = odom_y;
        odom_zero_yaw = odom_yaw;
        odom_origin_ready = true;
    }

    vision_initialized = true;
    syncPoseFromOdom();
    if (!use_odom_for_tracking || !odom_ready) {
        counter_x = initial_counter_x;
        counter_y = initial_counter_y;
        counter_yaw = initial_counter_yaw;
    }
}

static bool tryInitialVisualLocalization(const std::vector<cv::Point2f>& white_points,
                                         const std::vector<cv::Point2f>& magenta_points,
                                         const std::vector<cv::Point2f>& purple_points,
                                         const std::vector<cv::Point2f>& black_points,
                                         const std::vector<cv::Point2f>& red_points,
                                         const std::vector<cv::Point2f>& blue_points,
                                         const cv::Point2f& center)
{
    if (vision_initialized || image_lines_map.empty()) {
        return false;
    }

    const bool has_position_features =
        magenta_points.size() >= 6 ||
        purple_points.size() >= 6 ||
        (magenta_points.size() >= 3 && purple_points.size() >= 3);
    if (!has_position_features) {
        return false;
    }

    struct StartPose {
        const char* name;
        float world_x;
        float world_y;
        float world_yaw;
    };

    constexpr float half_field = FIELD_SIZE_METERS / 2.0f;
    constexpr float start_half = 0.15f;  // 300mm 出发区，中心距边缘 150mm。
    constexpr float start_center = half_field - start_half;
    const std::array<StartPose, 4> start_poses = {{
        {"start_1",  start_center,  start_center, -90.0f},
        {"start_2",  start_center, -start_center,  90.0f},
        {"start_3", -start_center, -start_center,  90.0f},
        {"start_4", -start_center,  start_center, -90.0f},
    }};

    int best_score = 0;
    const char* best_name = "";
    float best_x = 0;
    float best_y = 0;
    float best_yaw = 0;
    float best_world_x = 0;
    float best_world_y = 0;
    float best_world_yaw = 0;

    for (const auto& start_pose : start_poses) {
        float pose_x = start_pose.world_x * pixelsPerMeterX();
        float pose_y = -start_pose.world_y * pixelsPerMeterY();
        float pose_yaw = -start_pose.world_yaw;

        refinePoseWithDetectedPoints(white_points, magenta_points, purple_points, black_points, red_points, blue_points, center,
                                     pose_x, pose_y, pose_yaw);

        const MatchQuality quality = evaluatePoseQuality(
            white_points, magenta_points, purple_points, black_points, red_points, blue_points,
            center, pose_x, pose_y, pose_yaw, MatchFocus::Initial);
        if (!isMatchQualityAcceptable(quality)) {
            continue;
        }

        const int score = quality.score;
        if (score > best_score) {
            best_score = score;
            best_name = start_pose.name;
            best_x = pose_x;
            best_y = pose_y;
            best_yaw = pose_yaw;
            best_world_x = start_pose.world_x;
            best_world_y = start_pose.world_y;
            best_world_yaw = start_pose.world_yaw;
        }
    }

    if (best_score <= 0) {
        return false;
    }

    setVisualInitialPose(best_x, best_y, best_yaw);
    const MatchQuality quality = evaluatePoseQuality(
        white_points, magenta_points, purple_points, black_points, red_points, blue_points,
        center, best_x, best_y, best_yaw, MatchFocus::Initial);
    last_tracking_score = quality.score;
    last_tracking_matched = quality.matched;
    last_tracking_features = quality.features;
    last_tracking_match_valid = isMatchQualityAcceptable(quality);
    ROS_INFO(
        "初始视觉定位: %s center=(%.2fm, %.2fm) start_yaw=%.1fdeg score=%d corrected=(%.2fm, %.2fm, %.1fdeg)",
        best_name,
        best_world_x,
        best_world_y,
        best_world_yaw,
        best_score,
        counter_x / pixelsPerMeterX(),
        -counter_y / pixelsPerMeterY(),
        -counter_yaw);
    return true;
}

void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) 
{
    syncPoseFromOdom();

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    Mat image_raw = cv_ptr->image;
    Mat image_show = image_raw.clone();
    Mat image_scope = image_raw.clone();
    image_scope = Scalar(0, 0, 0);  // 将背景设置为黑色
    Mat image_lines = Mat::zeros(image_raw.size(), CV_8UC1);
    Mat image_corrected = image_lines.clone();
    Mat image_red = Mat::zeros(image_raw.size(), CV_8UC1);
    Mat image_blue = Mat::zeros(image_raw.size(), CV_8UC1);
    Mat image_black = Mat::zeros(image_raw.size(), CV_8UC1);
    Mat image_sideline_red = image_lines.clone();
    Mat image_sideline_blue = image_lines.clone();

    // 1. HSV空间的彩色区域检测。新场地是白底加彩色区域，不能再把白底当场线。
    Mat image_hsv;
    cv::cvtColor(image_raw, image_hsv, COLOR_BGR2HSV);

    Mat mask_red1;
    // 场地图红色 RGB(250,74,59)，OpenCV HSV 约为 (2,195,250)。
    mask_red1 = makeColorMask(image_raw, image_hsv, color_thresholds.red);
    image_red = mask_red1;

    image_black = makeColorMask(image_raw, image_hsv, color_thresholds.black);

    // 场地图洋红 RGB(255,25,255)，OpenCV HSV 约为 (150,230,255)。
    Mat image_magenta, image_purple;
    image_magenta = makeColorMask(image_raw, image_hsv, color_thresholds.magenta);
    createSeparatedBluePurpleMasks(image_raw, image_hsv, image_blue, image_purple);

    Mat image_white = image_magenta | image_purple;
    morphologyEx(image_white, image_white, MORPH_OPEN,
                 getStructuringElement(MORPH_RECT, Size(3, 3)));
    morphologyEx(image_magenta, image_magenta, MORPH_OPEN,
                 getStructuringElement(MORPH_RECT, Size(3, 3)));
    morphologyEx(image_purple, image_purple, MORPH_OPEN,
                 getStructuringElement(MORPH_RECT, Size(3, 3)));
    morphologyEx(image_blue, image_blue, MORPH_OPEN,
                 getStructuringElement(MORPH_RECT, Size(3, 3)));
    morphologyEx(image_black, image_black, MORPH_CLOSE,
                 getStructuringElement(MORPH_RECT, Size(3, 3)));
    dilate(image_black, image_black,
           getStructuringElement(MORPH_RECT, Size(3, 3)));

    int center_x = image_hsv.cols / 2;
    int center_y = image_hsv.rows / 2;
    const int self_mask_radius = std::min(image_hsv.cols, image_hsv.rows) / 8;
    circle(image_white, Point(center_x, center_y), self_mask_radius, Scalar(0), -1);
    circle(image_magenta, Point(center_x, center_y), self_mask_radius, Scalar(0), -1);
    circle(image_purple, Point(center_x, center_y), self_mask_radius, Scalar(0), -1);
    circle(image_red, Point(center_x, center_y), self_mask_radius, Scalar(0), -1);
    circle(image_blue, Point(center_x, center_y), self_mask_radius, Scalar(0), -1);
    circle(image_black, Point(center_x, center_y), self_mask_radius, Scalar(0), -1);
    const double min_feature_distance = 0.25;
    const double max_feature_distance = 4.50;
    const double rotated_cols = image_corrected.rows;
    const double rotated_rows = image_corrected.cols;
    const double camera_to_map_scale = std::min(
        static_cast<double>(image_lines_map.cols) / rotated_cols,
        static_cast<double>(image_lines_map.rows) / rotated_rows);
    const double projection_scale_correction = 1.09;
    const double map_pixels_per_meter = pixelsPerMeterX() * projection_scale_correction;
    cv::Point2f map_center(image_lines_map.cols / 2.0f, image_lines_map.rows / 2.0f);
    std::vector<cv::Point2f> projected_white_points;
    std::vector<cv::Point2f> projected_magenta_points;
    std::vector<cv::Point2f> projected_purple_points;
    std::vector<cv::Point2f> projected_black_points;
    std::vector<cv::Point2f> projected_red_points;
    std::vector<cv::Point2f> projected_blue_points;
    auto addProjectedPoint = [&](std::vector<cv::Point2f>& points, double distance_m, double rad) {
        cv::Point2f point(
            map_center.x - distance_m * map_pixels_per_meter * std::sin(rad),
            map_center.y + distance_m * map_pixels_per_meter * std::cos(rad));
        if (point.x >= 0 && point.x < image_lines_map.cols &&
            point.y >= 0 && point.y < image_lines_map.rows) {
            points.push_back(point);
        }
    };

    // 用射线扫描彩色边缘、红色和蓝色图像
    for (int angle = 0; angle < 360; angle += 1) {
        double rad = angle * CV_PI / 180.0;
        unsigned char last_pixel_white = 0;
        unsigned char last_pixel_magenta = 0;
        unsigned char last_pixel_purple = 0;
        unsigned char last_pixel_black = 0;
        unsigned char last_pixel_red = 0;
        unsigned char last_pixel_blue = 0;
        
        for (int length = 0; length < std::max(image_hsv.cols, image_hsv.rows); length++) {
            int x = static_cast<int>(center_x + length * cos(rad));
            int y = static_cast<int>(center_y + length * sin(rad));
            
            if (x >= 0 && x < image_white.cols && y >= 0 && y < image_white.rows) {
                // 检测彩色区域边缘
                unsigned char pixel_white = image_white.at<uchar>(y, x);
                if (length > 0 && last_pixel_white != pixel_white && (last_pixel_white == 255 || pixel_white == 255))
                {
                    // 在 image_show 中画紫色十字
                    cv::line(image_show, Point(x - 5, y), Point(x + 5, y), Scalar(255, 0, 255), 1);
                    cv::line(image_show, Point(x, y - 5), Point(x, y + 5), Scalar(255, 0, 255), 1);
                    cv::circle(image_lines, Point(x,y), 2, cv::Scalar(255, 255, 255), -1);

                    // 畸变矫正
                    double distance_m = distanceLookup.getDistance(length);
                    if(distance_m > min_feature_distance && distance_m < max_feature_distance)
                    {
                        addProjectedPoint(projected_white_points, distance_m, rad);
                        double dist = distance_m * map_pixels_per_meter / camera_to_map_scale;
                        int x2 = static_cast<int>(center_x + dist * cos(rad));
                        int y2 = static_cast<int>(center_y + dist * sin(rad));
                        if (x2 >= 0 && x2 < image_corrected.cols && y2 >= 0 && y2 < image_corrected.rows) {
                            image_corrected.at<uchar>(y2, x2) = 255;
                        }
                    }
                }
                last_pixel_white = pixel_white;

                unsigned char pixel_magenta = image_magenta.at<uchar>(y, x);
                if (length > 0 && last_pixel_magenta != pixel_magenta &&
                    (last_pixel_magenta == 255 || pixel_magenta == 255))
                {
                    double distance_m = distanceLookup.getDistance(length);
                    if(distance_m > min_feature_distance && distance_m < max_feature_distance)
                    {
                        addProjectedPoint(projected_magenta_points, distance_m, rad);
                    }
                }
                last_pixel_magenta = pixel_magenta;

                unsigned char pixel_purple = image_purple.at<uchar>(y, x);
                if (length > 0 && last_pixel_purple != pixel_purple &&
                    (last_pixel_purple == 255 || pixel_purple == 255))
                {
                    double distance_m = distanceLookup.getDistance(length);
                    if(distance_m > min_feature_distance && distance_m < max_feature_distance)
                    {
                        addProjectedPoint(projected_purple_points, distance_m, rad);
                    }
                }
                last_pixel_purple = pixel_purple;

                unsigned char pixel_black = image_black.at<uchar>(y, x);
                if (length > 0 && last_pixel_black != pixel_black &&
                    (last_pixel_black == 255 || pixel_black == 255))
                {
                    double distance_m = distanceLookup.getDistance(length);
                    if(distance_m > min_feature_distance && distance_m < max_feature_distance)
                    {
                        addProjectedPoint(projected_black_points, distance_m, rad);
                    }
                }
                last_pixel_black = pixel_black;

                // 检测红线边缘
                unsigned char pixel_red = image_red.at<uchar>(y, x);
                if (last_pixel_red != 255 && pixel_red == 255)  // 修改这里的条件
                {
                    // 在 image_show 中画黄色十字
                    cv::line(image_show, Point(x - 5, y), Point(x + 5, y), Scalar(0, 255, 255), 1);
                    cv::line(image_show, Point(x, y - 5), Point(x, y + 5), Scalar(0, 255, 255), 1);

                    // 畸变矫正
                    double distance_m = distanceLookup.getDistance(length);
                    if(distance_m > min_feature_distance && distance_m < max_feature_distance)
                    {
                        addProjectedPoint(projected_red_points, distance_m, rad);
                        double dist = distance_m * map_pixels_per_meter / camera_to_map_scale;
                        int x2 = static_cast<int>(center_x + dist * cos(rad));
                        int y2 = static_cast<int>(center_y + dist * sin(rad));
                        if (x2 >= 0 && x2 < image_sideline_red.cols && y2 >= 0 && y2 < image_sideline_red.rows) {
                            image_sideline_red.at<uchar>(y2, x2) = 255;
                        }
                    }
                }
                last_pixel_red = pixel_red;

                // 检测蓝线边缘
                unsigned char pixel_blue = image_blue.at<uchar>(y, x);
                if (last_pixel_blue != 255 && pixel_blue == 255)
                {
                    // 在 image_show 中画浅蓝色十字
                    cv::line(image_show, Point(x - 5, y), Point(x + 5, y), Scalar(255, 255, 0), 1);
                    cv::line(image_show, Point(x, y - 5), Point(x, y + 5), Scalar(255, 255, 0), 1);

                    // 畸变矫正
                    double distance_m = distanceLookup.getDistance(length);
                    if(distance_m > min_feature_distance && distance_m < max_feature_distance)
                    {
                        addProjectedPoint(projected_blue_points, distance_m, rad);
                        double dist = distance_m * map_pixels_per_meter / camera_to_map_scale;
                        int x2 = static_cast<int>(center_x + dist * cos(rad));
                        int y2 = static_cast<int>(center_y + dist * sin(rad));
                        if (x2 >= 0 && x2 < image_sideline_blue.cols && y2 >= 0 && y2 < image_sideline_blue.rows) {
                            image_sideline_blue.at<uchar>(y2, x2) = 255;
                        }
                    }
                }
                last_pixel_blue = pixel_blue;
            }
        }
    }

    // 将 image_corrected 顺时针旋转90度
    cv::Mat rotated_image;
    cv::rotate(image_corrected, rotated_image, cv::ROTATE_90_CLOCKWISE);

    // 计算旋转后图像的中心点
    cv::Point2f img_center(rotated_image.cols/2.0f, rotated_image.rows/2.0f);
    
    // 从原始射线检测结果直接投影到地图坐标，避免中间图像尺寸裁剪远距离特征。
    std::vector<cv::Point2f> white_points = projected_white_points;
    std::vector<cv::Point2f> magenta_points = projected_magenta_points;
    std::vector<cv::Point2f> purple_points = projected_purple_points;
    std::vector<cv::Point2f> black_points = projected_black_points;
    std::vector<cv::Point2f> red_points = projected_red_points;
    std::vector<cv::Point2f> blue_points = projected_blue_points;

    static int debug_frame_count = 0;
    if (++debug_frame_count % 30 == 0) {
        printf(
            "detected points before match: white=%zu magenta=%zu purple=%zu black=%zu red=%zu blue=%zu, matched=%d/%d score=%d status=%s\n",
            white_points.size(),
            magenta_points.size(),
            purple_points.size(),
            black_points.size(),
            red_points.size(),
            blue_points.size(),
            last_tracking_matched,
            last_tracking_features,
            last_tracking_score,
            last_tracking_match_valid ? "tracking" : "hold");
    }

    cv::Point2f center = map_center;
    if (!vision_initialized) {
        tryInitialVisualLocalization(
            white_points, magenta_points, purple_points, black_points, red_points, blue_points, center);
    } else if (!use_odom_for_tracking) {
        trackPoseWithDetectedPoints(white_points, magenta_points, purple_points, black_points, red_points, blue_points, center,
                                    counter_x, counter_y, counter_yaw);
    }
    syncPoseFromOdom();
    
    float rad = (counter_yaw) * CV_PI / 180.0;
    const bool have_valid_pose = vision_initialized;

    // 显示匹配效果
    cv::Mat image_match_result = image_lines_all.clone();
    cv::putText(
        image_match_result,
        cv::format(
            "match %s %d/%d score=%d",
            last_tracking_match_valid ? "tracking" : "hold",
            last_tracking_matched,
            last_tracking_features,
            last_tracking_score),
        cv::Point(8, 24),
        cv::FONT_HERSHEY_SIMPLEX,
        0.6,
        cv::Scalar(0, 0, 0),
        2);
    // cv::Mat image_match_result = image_lines_map.clone();
    // cv::Mat image_match_result = image_red_map.clone();
    // cv::Mat image_match_result = cv::Mat::zeros(image_red_map.size(), CV_8UC1);
    // for(int y = 0; y < image_match_result.rows; y++) {
    //     for(int x = 0; x < image_match_result.cols; x++) {
    //         image_match_result.at<uchar>(y,x) = std::max(image_red_map.at<uchar>(y,x), 
    //                                               image_blue_map.at<uchar>(y,x));
    //     }
    // }
    // cv::cvtColor(image_match_result, image_match_result, cv::COLOR_GRAY2BGR);
    
    // printf("counter_x: %.2f, counter_y: %.2f, counter_yaw: %.2f\n", counter_x, counter_y, counter_yaw);

    // 绘制白点
    for(const auto& point : white_points) {
        float rotated_x = point.x;
        float rotated_y = point.y;
        if (have_valid_pose) {
            float x = point.x - center.x;
            float y = point.y - center.y;
            rotated_x = x * cos(rad) - y * sin(rad) + center.x + counter_x;
            rotated_y = x * sin(rad) + y * cos(rad) + center.y + counter_y;
        }

        if(rotated_x >= 0 && rotated_x < image_match_result.cols &&
           rotated_y >= 0 && rotated_y < image_match_result.rows) {
            cv::circle(image_match_result, cv::Point(rotated_x, rotated_y), 5, cv::Scalar(0,255,0), -1);
        }
    }

    // 绘制红点
    for(const auto& point : red_points) {
        float rotated_x = point.x;
        float rotated_y = point.y;
        if (have_valid_pose) {
            float x = point.x - center.x;
            float y = point.y - center.y;
            rotated_x = x * cos(rad) - y * sin(rad) + center.x + counter_x;
            rotated_y = x * sin(rad) + y * cos(rad) + center.y + counter_y;
        }

        if(rotated_x >= 0 && rotated_x < image_match_result.cols &&
           rotated_y >= 0 && rotated_y < image_match_result.rows) {
            cv::circle(image_match_result, cv::Point(rotated_x, rotated_y), 5, cv::Scalar(0,0,255), -1);
        }
    }

    // 绘制蓝点
    for(const auto& point : blue_points) {
        float rotated_x = point.x;
        float rotated_y = point.y;
        if (have_valid_pose) {
            float x = point.x - center.x;
            float y = point.y - center.y;
            rotated_x = x * cos(rad) - y * sin(rad) + center.x + counter_x;
            rotated_y = x * sin(rad) + y * cos(rad) + center.y + counter_y;
        }

        if(rotated_x >= 0 && rotated_x < image_match_result.cols &&
           rotated_y >= 0 && rotated_y < image_match_result.rows) {
            cv::circle(image_match_result, cv::Point(rotated_x, rotated_y), 5, cv::Scalar(255,0,0), -1);
        }
    }
    
    // 将各种线条标记到 image_scope 上
    // for(int y = 0; y < image_scope.rows; y++) {
    //     for(int x = 0; x < image_scope.cols; x++) {
    //         // if(image_corrected.at<uchar>(y,x) == 255) {
    //         //     // 白色点表示彩色边缘
    //         //     circle(image_scope, Point(x,y), 2, Scalar(255,255,255), -1);
    //         // }
    //         if(image_sideline_red.at<uchar>(y,x) == 255) {
    //             // 红色点表示红线
    //             circle(image_scope, Point(x,y), 2, Scalar(0,0,255), -1);
    //         }
    //         if(image_sideline_blue.at<uchar>(y,x) == 255) {
    //             // 蓝色点表示蓝线
    //             circle(image_scope, Point(x,y), 2, Scalar(255,0,0), -1);
    //         }
    //     }
    // }

    // 在image_scope上绘制筛选后的white_points
    // for(const auto& point : white_points) {
    //     if(point.x >= 0 && point.x < image_scope.cols &&
    //        point.y >= 0 && point.y < image_scope.rows) {
    //         int x = -(point.x - x_offset - img_center.x)/scale_factor_x + img_center.x;
    //         int y = (point.y - y_offset - img_center.y)/scale_factor_y + img_center.y;
    //         circle(image_scope, Point(y, x), 2, Scalar(255,255,255), -1);
    //     }
    // }

    // 创建显示用的图像副本
    field_image.copyTo(monitor_image(cv::Rect(0, MONITOR_INFO_HEIGHT, field_image.cols, field_image.rows)));
    
    // 清除并更新信息区域
    cv::Mat info_area = monitor_image(cv::Rect(0, 0, monitor_image.cols, MONITOR_INFO_HEIGHT));
    info_area.setTo(cv::Scalar(255, 255, 255));  // 清除之前的文本
    
    std::string vision_coordinates = have_valid_pose
        ? cv::format(
            "Vision X: %.2fm Y: %.2fm Yaw: %.1fdeg",
            counter_x / pixelsPerMeterX(),
            -counter_y / pixelsPerMeterY(),
            -counter_yaw)
        : cv::format(
            "Vision  waiting: magenta=%zu purple=%zu black=%zu red=%zu blue=%zu",
            magenta_points.size(),
            purple_points.size(),
            black_points.size(),
            red_points.size(),
            blue_points.size());
    cv::putText(monitor_image, vision_coordinates, cv::Point(8, 34),
                cv::FONT_HERSHEY_SIMPLEX, 0.62, cv::Scalar(0, 0, 0), 2);

    std::string odom_coordinates = odom_ready
        ? cv::format("Odom X: %.2fm Y: %.2fm Yaw: %.1fdeg", odom_x, odom_y, odom_yaw)
        : std::string("Odom    waiting for odom...");
    cv::putText(monitor_image, odom_coordinates, cv::Point(8, 74),
                cv::FONT_HERSHEY_SIMPLEX, 0.62, cv::Scalar(50, 50, 50), 2);
    
    if (have_valid_pose) {
        // 绘制机器人图标
        cv::Point2f bottom_center(field_image.cols/2.0f, field_image.rows/2.0f + MONITOR_INFO_HEIGHT);
        float display_scale_x = static_cast<float>(field_image.cols) / image_lines_map.cols;
        float display_scale_y = static_cast<float>(field_image.rows) / image_lines_map.rows;
        float robot_x = bottom_center.x + counter_x * display_scale_x;
        float robot_y = bottom_center.y + counter_y * display_scale_y;
        cv::Point robot_pos(robot_x, robot_y);

        int robot_radius = 15;
        cv::circle(monitor_image, robot_pos, robot_radius, cv::Scalar(0,0,0), 2);
        cv::circle(monitor_image, robot_pos, robot_radius-2, cv::Scalar(255,0,255), -1);

        float direction_length = 20.0f;
        float direction_rad = counter_yaw * CV_PI / 180.0;
        cv::Point direction_end(
            robot_x + direction_length * cos(direction_rad),
            robot_y + direction_length * sin(direction_rad)
        );
        cv::line(monitor_image, robot_pos, direction_end, cv::Scalar(0,0,0), 2);
    }

    // 显示
    cv::imshow("result", image_show);
    // cv::imshow("scope", image_scope);
    cv::imshow("match_result", image_match_result);
    cv::imshow("定位", monitor_image);
    cv::waitKey(1);

    if (have_valid_pose) {
        publishRobotPose(msg->header.stamp);
    }
}

void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg)
{
    odom_x = msg->pose.pose.position.x;
    odom_y = msg->pose.pose.position.y;

    double roll, pitch, yaw;
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    odom_yaw = yaw * 180.0 / M_PI;
    odom_ready = true;
    if (!odom_origin_ready) {
        odom_zero_x = odom_x;
        odom_zero_y = odom_y;
        odom_zero_yaw = odom_yaw;
        odom_origin_ready = true;
    }
}

int main(int argc, char** argv) {
    setlocale(LC_ALL, "");
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("loc_sidelines");
    loadColorThresholds(node);

    std::string table_file = node->declare_parameter<std::string>("table_file", "distances.txt");
    distanceLookup.init(table_file);

    std::string lines_file = node->declare_parameter<std::string>("lines_file", "lines.jpg");
    image_lines_all = cv::imread(lines_file);
    image_magenta_map = createMagentaMap(image_lines_all);
    image_purple_map = createPurpleMap(image_lines_all);
    image_black_map = createBlackMap(image_lines_all);


    std::string field_file = node->declare_parameter<std::string>("field_file", "field_bg.png");
    cv::Mat resultImage = cv::imread(field_file);
    if(resultImage.empty()) {
        ROS_ERROR("读取场地背景图失败: %s", field_file.c_str());
        return -1;
    }
    

    // 初始化field_image为彩色图像，尺寸为resultImage的一半
    cv::resize(resultImage, field_image, cv::Size(resultImage.cols/2, resultImage.rows/2));
    
    // 初始化monitor_image，大小与field_image相同
    monitor_image = cv::Mat(field_image.rows + MONITOR_INFO_HEIGHT, field_image.cols, CV_8UC3);

    
    // 读取参数
    std::string lines_map_path = node->declare_parameter<std::string>("lines_map_file", "");
    std::string red_map_path = node->declare_parameter<std::string>("red_map_file", "");
    std::string blue_map_path = node->declare_parameter<std::string>("blue_map_file", "");
    use_odom_for_tracking = node->declare_parameter<bool>("use_odom_for_tracking", false);
    use_safety_axis_constraint = node->declare_parameter<bool>("use_safety_axis_constraint", false);
    std::string image_topic = node->declare_parameter<std::string>("image_topic", "/robot/image_raw");
    std::string odom_topic = node->declare_parameter<std::string>("odom_topic", "/robot/odom");
    std::string pose_topic = node->declare_parameter<std::string>("pose_topic", "/robot/pose");
    pose_frame_id = node->declare_parameter<std::string>("pose_frame_id", "map");
    // 读取图像为灰度图
    image_lines_map = cv::imread(lines_map_path, cv::IMREAD_GRAYSCALE);
    image_red_map = cv::imread(red_map_path, cv::IMREAD_GRAYSCALE);
    image_blue_map = cv::imread(blue_map_path, cv::IMREAD_GRAYSCALE);
    red_map_axis = estimateAxisFromMap(image_red_map);
    blue_map_axis = estimateAxisFromMap(image_blue_map);

    // 检查图像是否成功加载
    if(image_lines_map.empty() || image_red_map.empty() || image_blue_map.empty() ||
       image_magenta_map.empty() || image_purple_map.empty() || image_black_map.empty()) {
        ROS_ERROR("场线模板读取失败！");
        return -1;
    }
    if (red_map_axis.valid) {
        ROS_INFO("红色安全区地图主轴: %.1f deg, confidence=%.2f", red_map_axis.angle_deg, red_map_axis.confidence);
    }
    if (blue_map_axis.valid) {
        ROS_INFO("蓝色安全区地图主轴: %.1f deg, confidence=%.2f", blue_map_axis.angle_deg, blue_map_axis.confidence);
    }

    pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic, 10);

    auto sub = node->create_subscription<sensor_msgs::msg::Image>(
        image_topic, rclcpp::SensorDataQoS(), imageCallback);
    auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 10, odomCallback);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}

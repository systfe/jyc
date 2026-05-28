#include "lines_map.h"
#include "ros2_compat.h"

LinesMap::LinesMap() {
}

void LinesMap::loadImage(const std::string& imagePath) {
    lines_image = cv::imread(imagePath);
    if (lines_image.empty()) {
        ROS_ERROR("打不开图片文件: %s", imagePath.c_str());
        return;
    }
    printf("正在初始化 彩色边缘 匹配模板，请稍等 ……\n");
    generateGradientImage();
    printf("正在初始化 蓝色安全区边缘 匹配模板，请稍等 ……\n");
    generateBlueGradientImage();
    printf("正在初始化 红色安全区边缘 匹配模板，请稍等 ……\n");
    generateRedGradientImage();
}

cv::Mat LinesMap::createGradientMask(int size) {
    cv::Mat mask(size, size, CV_8UC1);
    int center = size / 2;
    for (int y = 0; y < size; y++) {
        for (int x = 0; x < size; x++) {
            double distance = std::hypot(x - center, y - center);
            int value = cv::saturate_cast<uchar>(255 * std::max(0.0, 1.0 - distance / center));
            mask.at<uchar>(y, x) = value;
        }
    }
    return mask;
}

cv::Mat LinesMap::createColorMask() const {
    cv::Mat hsv;
    cv::cvtColor(lines_image, hsv, cv::COLOR_BGR2HSV);

    cv::Mat colorMask;
    cv::inRange(hsv, cv::Scalar(0, 35, 30), cv::Scalar(180, 255, 255), colorMask);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(colorMask, colorMask, cv::MORPH_OPEN, kernel);
    return colorMask;
}

cv::Mat LinesMap::createRedMask() const {
    cv::Mat hsv;
    cv::cvtColor(lines_image, hsv, cv::COLOR_BGR2HSV);

    cv::Mat redMask1, redMask2;
    cv::inRange(hsv, cv::Scalar(0, 80, 80), cv::Scalar(12, 255, 255), redMask1);
    cv::inRange(hsv, cv::Scalar(165, 80, 80), cv::Scalar(180, 255, 255), redMask2);
    return redMask1 | redMask2;
}

cv::Mat LinesMap::createBlueMask() const {
    cv::Mat hsv;
    cv::cvtColor(lines_image, hsv, cv::COLOR_BGR2HSV);

    cv::Mat blueMask;
    cv::inRange(hsv, cv::Scalar(90, 45, 45), cv::Scalar(125, 255, 255), blueMask);
    return blueMask;
}

cv::Mat LinesMap::createEdgeMask(const cv::Mat& mask) const {
    cv::Mat edges;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(mask, edges, cv::MORPH_GRADIENT, kernel);
    cv::threshold(edges, edges, 0, 255, cv::THRESH_BINARY);
    return edges;
}

cv::Mat LinesMap::createGradientFromEdges(const cv::Mat& edgeMask, int radius) const {
    cv::Mat invertedEdges;
    cv::threshold(edgeMask, invertedEdges, 0, 255, cv::THRESH_BINARY_INV);

    cv::Mat distances;
    cv::distanceTransform(invertedEdges, distances, cv::DIST_L2, 3);

    cv::Mat gradient = cv::Mat::zeros(edgeMask.size(), CV_8UC1);
    for (int y = 0; y < distances.rows; y++) {
        for (int x = 0; x < distances.cols; x++) {
            float distance = distances.at<float>(y, x);
            if (distance <= radius) {
                gradient.at<uchar>(y, x) = cv::saturate_cast<uchar>(255.0f * (1.0f - distance / radius));
            }
        }
    }
    return gradient;
}

void LinesMap::generateGradientImage() {
    cv::Mat edgeMask = createEdgeMask(createColorMask());
    int radius = std::max(30, std::max(lines_image.cols, lines_image.rows) / 20);
    gradientImage = createGradientFromEdges(edgeMask, radius);
}

void LinesMap::generateBlueGradientImage() {
    cv::Mat edgeMask = createEdgeMask(createBlueMask());
    int radius = std::max(60, std::max(lines_image.cols, lines_image.rows) / 10);
    blueIMap = createGradientFromEdges(edgeMask, radius);
}

void LinesMap::generateRedGradientImage() {
    cv::Mat edgeMask = createEdgeMask(createRedMask());
    int radius = std::max(60, std::max(lines_image.cols, lines_image.rows) / 10);
    redIMap = createGradientFromEdges(edgeMask, radius);
}

cv::Mat LinesMap::getLinesMap() const {
    return gradientImage;
}

cv::Mat LinesMap::getRawImage() const {
    return lines_image.clone();
}

cv::Mat LinesMap::getBlueMap() const {
    return blueIMap;
}

cv::Mat LinesMap::getRedMap() const {
    return redIMap;
}

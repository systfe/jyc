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

    cv::Mat redMask1, blueMask, magentaMask, purpleMask;
    cv::inRange(hsv, cv::Scalar(0, 120, 150), cv::Scalar(8, 255, 255), redMask1);
    cv::inRange(hsv, cv::Scalar(94, 45, 170), cv::Scalar(114, 225, 255), blueMask);
    cv::inRange(hsv, cv::Scalar(130, 45, 60), cv::Scalar(170, 255, 255), magentaMask);
    cv::inRange(hsv, cv::Scalar(94, 90, 30), cv::Scalar(126, 255, 165), purpleMask);

    cv::Mat colorMask = magentaMask | purpleMask;

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(colorMask, colorMask, cv::MORPH_OPEN, kernel);
    return colorMask;
}

cv::Mat LinesMap::createRedMask() const {
    cv::Mat hsv;
    cv::cvtColor(lines_image, hsv, cv::COLOR_BGR2HSV);

    cv::Mat redMask;
    cv::inRange(hsv, cv::Scalar(0, 120, 150), cv::Scalar(8, 255, 255), redMask);
    return redMask;
}

cv::Mat LinesMap::createBlueMask() const {
    cv::Mat hsv;
    cv::cvtColor(lines_image, hsv, cv::COLOR_BGR2HSV);

    cv::Mat blueMask;
    cv::inRange(hsv, cv::Scalar(94, 45, 170), cv::Scalar(114, 225, 255), blueMask);
    return blueMask;
}

cv::Mat LinesMap::createEdgeMask(const cv::Mat& mask) const {
    cv::Mat edges;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(mask, edges, cv::MORPH_GRADIENT, kernel);
    cv::threshold(edges, edges, 0, 255, cv::THRESH_BINARY);
    return edges;
}

cv::Mat LinesMap::createDirectionalGradientFromMask(const cv::Mat& mask, int innerRadius, int outerRadius) const {
    (void)innerRadius;
    cv::Mat binaryMask;
    cv::threshold(mask, binaryMask, 0, 255, cv::THRESH_BINARY);

    cv::Mat invertedMask;
    cv::bitwise_not(binaryMask, invertedMask);

    cv::Mat outsideDistances;
    cv::distanceTransform(invertedMask, outsideDistances, cv::DIST_L2, 3);

    cv::Mat gradient = cv::Mat::zeros(mask.size(), CV_8UC1);
    for (int y = 0; y < gradient.rows; y++) {
        for (int x = 0; x < gradient.cols; x++) {
            if (binaryMask.at<uchar>(y, x) > 0) {
                gradient.at<uchar>(y, x) = 255;
                continue;
            }

            if (outerRadius <= 0) {
                continue;
            }

            const float distance = outsideDistances.at<float>(y, x);
            if (distance <= outerRadius) {
                gradient.at<uchar>(y, x) =
                    cv::saturate_cast<uchar>(255.0f * (1.0f - distance / outerRadius));
            }
        }
    }
    return gradient;
}

void LinesMap::generateGradientImage() {
    cv::Mat colorMask = createColorMask();
    int innerRadius = std::max(6, std::max(lines_image.cols, lines_image.rows) / 100);
    int outerRadius = std::max(18, std::max(lines_image.cols, lines_image.rows) / 35);
    gradientImage = createDirectionalGradientFromMask(colorMask, innerRadius, outerRadius);
}

void LinesMap::generateBlueGradientImage() {
    cv::Mat blueMask = createBlueMask();
    int innerRadius = std::max(36, std::max(lines_image.cols, lines_image.rows) / 18);
    int outerRadius = std::max(12, std::max(lines_image.cols, lines_image.rows) / 65);
    blueIMap = createDirectionalGradientFromMask(blueMask, innerRadius, outerRadius);
}

void LinesMap::generateRedGradientImage() {
    cv::Mat redMask = createRedMask();
    int innerRadius = std::max(36, std::max(lines_image.cols, lines_image.rows) / 18);
    int outerRadius = std::max(12, std::max(lines_image.cols, lines_image.rows) / 65);
    redIMap = createDirectionalGradientFromMask(redMask, innerRadius, outerRadius);
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

#include "lines_map.h"
#include "ros2_compat.h"

namespace {

void createSeparatedBluePurpleMasks(const cv::Mat& bgrImage,
                                    const cv::Mat& hsvImage,
                                    cv::Mat& blueMask,
                                    cv::Mat& purpleMask)
{
    cv::Mat blueHsv;
    cv::Mat purpleHsv;
    cv::inRange(hsvImage, cv::Scalar(96, 40, 175), cv::Scalar(110, 210, 255), blueHsv);
    cv::inRange(hsvImage, cv::Scalar(94, 105, 25), cv::Scalar(126, 255, 255), purpleHsv);

    cv::Mat blueBgrGate;
    cv::Mat purpleBgrGate;
    cv::inRange(bgrImage, cv::Scalar(145, 95, 40), cv::Scalar(255, 255, 170), blueBgrGate);
    cv::inRange(bgrImage, cv::Scalar(60, 20, 0), cv::Scalar(255, 150, 80), purpleBgrGate);

    cv::bitwise_and(blueHsv, blueBgrGate, blueMask);
    cv::bitwise_and(purpleHsv, purpleBgrGate, purpleMask);

    cv::Mat notPurple;
    cv::bitwise_not(purpleMask, notPurple);
    cv::bitwise_and(blueMask, notPurple, blueMask);
}

} // namespace

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
    cv::inRange(hsv, cv::Scalar(130, 45, 60), cv::Scalar(170, 255, 255), magentaMask);
    createSeparatedBluePurpleMasks(lines_image, hsv, blueMask, purpleMask);

    cv::Mat colorMask = magentaMask | purpleMask;

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(colorMask, colorMask, cv::MORPH_OPEN, kernel);
    return colorMask;
}

cv::Mat LinesMap::createMagentaMask() const {
    cv::Mat hsv;
    cv::cvtColor(lines_image, hsv, cv::COLOR_BGR2HSV);

    cv::Mat magentaMask;
    cv::inRange(hsv, cv::Scalar(130, 45, 60), cv::Scalar(170, 255, 255), magentaMask);
    cv::morphologyEx(
        magentaMask,
        magentaMask,
        cv::MORPH_OPEN,
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
    return magentaMask;
}

cv::Mat LinesMap::createPurpleMask() const {
    cv::Mat hsv;
    cv::cvtColor(lines_image, hsv, cv::COLOR_BGR2HSV);

    cv::Mat blueMask;
    cv::Mat purpleMask;
    createSeparatedBluePurpleMasks(lines_image, hsv, blueMask, purpleMask);
    cv::morphologyEx(
        purpleMask,
        purpleMask,
        cv::MORPH_OPEN,
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
    return purpleMask;
}

cv::Mat LinesMap::createFilledSafetyZoneMaskFromPurple(const cv::Mat& purpleMask) const {
    cv::Mat binaryMask;
    cv::threshold(purpleMask, binaryMask, 0, 255, cv::THRESH_BINARY);

    const int closeSize =
        std::max(5, (std::min(binaryMask.cols, binaryMask.rows) / 120) | 1);
    cv::Mat closedMask;
    cv::morphologyEx(
        binaryMask,
        closedMask,
        cv::MORPH_CLOSE,
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(closeSize, closeSize)));

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(closedMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    cv::Mat filledMask = cv::Mat::zeros(purpleMask.size(), CV_8UC1);
    const double minArea =
        static_cast<double>(purpleMask.cols * purpleMask.rows) * 0.001;
    for (size_t i = 0; i < contours.size(); i++) {
        const cv::Rect rect = cv::boundingRect(contours[i]);
        if (cv::contourArea(contours[i]) < minArea ||
            rect.width < purpleMask.cols / 12 ||
            rect.height < purpleMask.rows / 20) {
            continue;
        }
        cv::drawContours(filledMask, contours, static_cast<int>(i), cv::Scalar(255), cv::FILLED);
    }

    return filledMask;
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
    cv::Mat purpleMask;
    createSeparatedBluePurpleMasks(lines_image, hsv, blueMask, purpleMask);
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

    cv::Mat edgeMask;
    cv::morphologyEx(
        binaryMask,
        edgeMask,
        cv::MORPH_GRADIENT,
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
    cv::threshold(edgeMask, edgeMask, 0, 255, cv::THRESH_BINARY);

    cv::Mat invertedEdgeMask;
    cv::bitwise_not(edgeMask, invertedEdgeMask);

    cv::Mat edgeDistances;
    cv::distanceTransform(invertedEdgeMask, edgeDistances, cv::DIST_L2, 3);

    cv::Mat gradient = cv::Mat::zeros(mask.size(), CV_8UC1);
    for (int y = 0; y < gradient.rows; y++) {
        for (int x = 0; x < gradient.cols; x++) {
            if (edgeMask.at<uchar>(y, x) > 0) {
                gradient.at<uchar>(y, x) = 255;
                continue;
            }

            if (outerRadius <= 0) {
                continue;
            }

            const float distance = edgeDistances.at<float>(y, x);
            if (distance <= outerRadius) {
                gradient.at<uchar>(y, x) =
                    cv::saturate_cast<uchar>(255.0f * (1.0f - distance / outerRadius));
            }
        }
    }
    return gradient;
}

cv::Mat LinesMap::createFilledGradientFromMask(const cv::Mat& mask,
                                               const cv::Mat& blockedMask,
                                               int outerRadius) const {
    cv::Mat binaryMask;
    cv::threshold(mask, binaryMask, 0, 255, cv::THRESH_BINARY);

    cv::Mat edgeMask;
    cv::morphologyEx(
        binaryMask,
        edgeMask,
        cv::MORPH_GRADIENT,
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
    cv::threshold(edgeMask, edgeMask, 0, 255, cv::THRESH_BINARY);

    cv::Mat invertedEdgeMask;
    cv::bitwise_not(edgeMask, invertedEdgeMask);

    cv::Mat edgeDistances;
    cv::distanceTransform(invertedEdgeMask, edgeDistances, cv::DIST_L2, 3);

    cv::Mat gradient = cv::Mat::zeros(mask.size(), CV_8UC1);
    for (int y = 0; y < gradient.rows; y++) {
        for (int x = 0; x < gradient.cols; x++) {
            if (binaryMask.at<uchar>(y, x) > 0) {
                gradient.at<uchar>(y, x) = 255;
                continue;
            }

            if (!blockedMask.empty() && blockedMask.at<uchar>(y, x) > 0) {
                continue;
            }

            if (outerRadius <= 0) {
                continue;
            }

            const float distance = edgeDistances.at<float>(y, x);
            if (distance <= outerRadius) {
                gradient.at<uchar>(y, x) =
                    cv::saturate_cast<uchar>(255.0f * (1.0f - distance / outerRadius));
            }
        }
    }
    return gradient;
}

void LinesMap::generateGradientImage() {
    cv::Mat magentaMask = createMagentaMask();
    cv::Mat purpleMask = createPurpleMask();
    cv::Mat safetyZoneMask = createFilledSafetyZoneMaskFromPurple(purpleMask);
    int innerRadius = std::max(6, std::max(lines_image.cols, lines_image.rows) / 100);
    int outerRadius = std::max(18, std::max(lines_image.cols, lines_image.rows) / 35);
    cv::Mat magentaGradient = createDirectionalGradientFromMask(magentaMask, innerRadius, outerRadius);
    cv::Mat purpleGradient = createFilledGradientFromMask(purpleMask, safetyZoneMask, outerRadius);
    cv::max(magentaGradient, purpleGradient, gradientImage);
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

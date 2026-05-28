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
    printf("正在初始化 白色场线 匹配模板，请稍等 ……\n");
    generateGradientImage();
    printf("正在初始化 蓝色边线 匹配模板，请稍等 ……\n");
    generateBlueGradientImage();
    printf("正在初始化 红色边线 匹配模板，请稍等 ……\n");
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

void LinesMap::generateGradientImage() {
    int maskSize = 200;
    cv::Mat mask = createGradientMask(maskSize);
    gradientImage = cv::Mat::zeros(lines_image.size(), CV_8UC1);

    for (int y = 0; y < lines_image.rows; y++) {
        for (int x = 0; x < lines_image.cols; x++) {
            // 获取BGR像素值
            cv::Vec3b pixel = lines_image.at<cv::Vec3b>(y, x);
            // 判断是否为白色像素 (B,G,R 值都接近255)
            if (pixel[0] > 240 && pixel[1] > 240 && pixel[2] > 240) {
                for (int dy = -maskSize/2; dy < maskSize/2; dy++) {
                    for (int dx = -maskSize/2; dx < maskSize/2; dx++) {
                        int newY = y + dy;
                        int newX = x + dx;
                        
                        if (newY >= 0 && newY < gradientImage.rows && 
                            newX >= 0 && newX < gradientImage.cols) {
                            int maskY = dy + maskSize/2;
                            int maskX = dx + maskSize/2;
                            uchar maskValue = mask.at<uchar>(maskY, maskX);
                            
                            if (maskValue > gradientImage.at<uchar>(newY, newX)) {
                                gradientImage.at<uchar>(newY, newX) = maskValue;
                            }
                        }
                    }
                }
            }
        }
    }
}

void LinesMap::generateBlueGradientImage() {
    cv::Mat hsv;
    cv::cvtColor(lines_image, hsv, cv::COLOR_BGR2HSV);
    
    // 定义浅蓝色的HSV范围
    cv::Mat blueMask;
    cv::Scalar lowerBlue(90, 50, 50);   // HSV中的浅蓝色下限
    cv::Scalar upperBlue(120, 255, 255); // HSV中的浅蓝色上限
    cv::inRange(hsv, lowerBlue, upperBlue, blueMask);
    
    // 创建渐变掩码
    int maskSize = 800;
    cv::Mat gradientMask = createGradientMask(maskSize);
    
    // 初始化blueIMap
    blueIMap = cv::Mat::zeros(lines_image.size(), CV_8UC1);
    
    // 对每个蓝色像素应用渐变
    for (int y = 0; y < blueMask.rows; y++) {
        for (int x = 0; x < blueMask.cols; x++) {
            if (blueMask.at<uchar>(y, x) > 0) {
                for (int dy = -maskSize/2; dy < maskSize/2; dy++) {
                    for (int dx = -maskSize/2; dx < maskSize/2; dx++) {
                        int newY = y + dy;
                        int newX = x + dx;
                        
                        if (newY >= 0 && newY < blueIMap.rows && 
                            newX >= 0 && newX < blueIMap.cols) {
                            int maskY = dy + maskSize/2;
                            int maskX = dx + maskSize/2;
                            uchar maskValue = gradientMask.at<uchar>(maskY, maskX);
                            
                            if (maskValue > blueIMap.at<uchar>(newY, newX)) {
                                blueIMap.at<uchar>(newY, newX) = maskValue;
                            }
                        }
                    }
                }
            }
        }
    }
}

void LinesMap::generateRedGradientImage() {
    cv::Mat hsv;
    cv::cvtColor(lines_image, hsv, cv::COLOR_BGR2HSV);
    
    // 定义黄色的HSV范围
    cv::Mat yellowMask;
    cv::Scalar lowerYellow(20, 100, 100);   // HSV中的黄色下限
    cv::Scalar upperYellow(30, 255, 255);    // HSV中的黄色上限
    
    // 检测黄色区域
    cv::inRange(hsv, lowerYellow, upperYellow, yellowMask);
    
    // 创建渐变掩码
    int maskSize = 800;
    cv::Mat gradientMask = createGradientMask(maskSize);
    
    // 初始化redIMap
    redIMap = cv::Mat::zeros(lines_image.size(), CV_8UC1);
    
    // 对每个红色像素应用渐变
    for (int y = 0; y < yellowMask.rows; y++) {
        for (int x = 0; x < yellowMask.cols; x++) {
            if (yellowMask.at<uchar>(y, x) > 0) {
                for (int dy = -maskSize/2; dy < maskSize/2; dy++) {
                    for (int dx = -maskSize/2; dx < maskSize/2; dx++) {
                        int newY = y + dy;
                        int newX = x + dx;
                        
                        if (newY >= 0 && newY < redIMap.rows && 
                            newX >= 0 && newX < redIMap.cols) {
                            int maskY = dy + maskSize/2;
                            int maskX = dx + maskSize/2;
                            uchar maskValue = gradientMask.at<uchar>(maskY, maskX);
                            
                            if (maskValue > redIMap.at<uchar>(newY, newX)) {
                                redIMap.at<uchar>(newY, newX) = maskValue;
                            }
                        }
                    }
                }
            }
        }
    }
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

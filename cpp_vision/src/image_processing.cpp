#include "image_processing.h"

cv::Mat convertToGray(const cv::Mat& img) {
    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    return gray;
}

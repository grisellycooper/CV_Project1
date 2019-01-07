#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//** Apply diferents OpenCV Methods and tecniques for reducing information and removing noise **// 
void preprocessImage(cv::Mat src, cv::Mat output, std::vector<std::vector<cv::Point>>& contours, 
                        std::vector<cv::Vec4i>& hierarchy);

//** Find the rings that belong to pattern **// 
void identifyRings(std::vector<std::vector<cv::Point>>& contours, std::vector<cv::Vec4i>& hierarchy, 
                    std::vector<cv::Point2f>& pointbuf, int patternSize, cv::Mat src);

bool findRingsGrid(cv::Mat src, cv::Size patternSize, std::vector<cv::Point2f>& pointbuf, 
                    std::vector<cv::Point2f>& previousPointbuf, bool prevoiusFound);
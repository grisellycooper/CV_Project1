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

bool verifyOrder(std::vector<cv::Point2f> &tmpPB, int patternHeigh, int patternWidth);
std::vector< std::vector<int> > GenerateCombinations(int n, int r);
void printCombination(std::vector< std::vector<int> >& v, int arr[], int n, int r);
void combinationUtil(std::vector< std::vector<int> >& v, int arr[], std::vector<int> &data, int start, int end, int index, int r);
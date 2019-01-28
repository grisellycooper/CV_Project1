#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//** Apply diferents OpenCV Methods and tecniques for reducing information and removing noise **// 
void preprocessImage(cv::Mat src, cv::Mat output, 
                        std::vector<std::vector<cv::Point>>& contours, 
                        std::vector<cv::Vec4i>& hierarchy);

//** Find the rings that belong to pattern **// 
void identifyRings(std::vector<std::vector<cv::Point>>& contours, 
                    std::vector<cv::Vec4i>& hierarchy, 
                    std::vector<cv::Point2f>& pointbuf, 
                    int patternSize, 
                    cv::Mat src);

//** Find the grid acording to the pattern size **//
bool findRingsGrid(cv::Mat src, 
                    cv::Size patternSize, 
                    std::vector<cv::Point2f>& pointbuf, 
                    std::vector<cv::Point2f>& previousPointbuf, 
                    bool prevoiusFound);

//** Try to find an order among rows and columns acording to the pattern size **//
bool verifyOrder(std::vector<cv::Point2f> &tmpPB, 
                    int patternHeigh, 
                    int patternWidth);

//** Util functions **//
void GenerateCombinations(std::vector<std::vector<int>> &combs, int n, int r);
void getCombination(std::vector< std::vector<int>> &combs_, int arr[], int n, int r);
void recursiveCombination(std::vector< std::vector<int>> &combs_, int arr[], std::vector<int> &data, int start, int end, int index, int r);
bool getPointBufferInOrder(std::vector<cv::Point2f> &pointbuf, std::vector<cv::Point2f> &tmpPB, int patternHeigh, int patternWidth);
void thresholdIntegral(cv::Mat &inputMat, cv::Mat &outputMat);  // Not that util

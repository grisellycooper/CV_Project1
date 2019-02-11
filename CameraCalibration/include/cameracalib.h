#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <vector>

/* Assuming the pattern is parallel to the camera, 
 *this function returns real control points distribution among the pattern 
 */ 
void getControlPointsPositions(cv::Size size, 
                               float ctrlPointsDistance, 
                               std::vector<cv::Point3f>& controlPoints, 
                               int patternType);

/* Compute and average colinearity for every frame */
void getAverageColinearity(std::vector<cv::Point2f>& imagePoints,
                           cv::Size size, 
                           float &avgColiniearity);

/* GG */
void getFrontoParallelPoints(std::vector<cv::Point2f> &frontoPoints,
                             cv::Size imgSize, 
                             cv::Size patternSize,
                             float distanceBP);

bool findPatternFP(cv::Mat inputFrame, 
                   int pointsPerRow, 
                   int pointsPerColum, 
                   std::vector<cv::Point2f> &pointbuffp);

void getAverageWithColinearPoints(std::vector<cv::Point2f>& inpointbuf,
                                  std::vector<cv::Point2f>& outpointbuf,
                                  cv::Mat frame,
                                  cv::Size size,
                                  float &avgColinearity);
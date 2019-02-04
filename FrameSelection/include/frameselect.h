#include <vector>
#include <opencv2/opencv.hpp>

//** The pattern in a good frame should cover almost all frame in a camera-fronto-parallel way **//   
bool isGoodFrame(cv::Mat src,
                 std::vector<cv::Point2f>& pointbuf,
                 std::vector<cv::Point2f>& previousCornersBuf,
                 bool previousCorners,
                 int patternWidth,
                 int patternHeigh,
                 float &minDistControlPoints);

void initializeGrid(int framePerQt);

//** 2nd version xD **//
//** The pattern in a good frame should cover almost all frame in a camera-fronto-parallel way **//   
bool isGoodFrameImp(cv::Mat src,
                    std::vector<cv::Point2f>& pointbuf,
                    std::vector<cv::Point2f>& previousCornersBuf,
                    bool previousCorners,
                    int patternWidth,
                    int patternHeigh,
                    float &minDistControlPoints);
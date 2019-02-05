#include "../include/cameracalib.h"

#define displayCompletePreprocess 0 

enum Pattern
{
    CHESSBOARD,
    CIRCLES_GRID,
    ASYMMETRIC_CIRCLES_GRID,
    RINGS_GRID
};

void 
getControlPointsPositions(cv::Size size, 
                          float ctrlPointsDistance, 
                          std::vector<cv::Point3f>& controlPoints, 
                          int patternType)
{
    controlPoints.clear();
    float x, y, z = 0;
    switch(patternType){
        case CHESSBOARD:
        case CIRCLES_GRID:
        case RINGS_GRID:            
            for(int i = 0 ; i < size.height; i++){
                for(int j = 0; j < size.width; j++){
                    x = j * ctrlPointsDistance;
                    y = i * ctrlPointsDistance;
                    controlPoints.push_back(cv::Point3f(x,y,0) );                    
                    //std::cout<<"x: "<<x<<"y: "<<y <<std::endl;
                }
            }
            //std::cout<<" " <<controlPoints.size() ;
            break;

        case ASYMMETRIC_CIRCLES_GRID:
            for( int i = 0; i < size.height; i++ ){
                for( int j = 0; j < size.width; j++ ){
                    
                    controlPoints.push_back(cv::Point3f(float((2*j + i % 2)*ctrlPointsDistance), float(i*ctrlPointsDistance), 0));
                }
            }
            break;
    }
}

void 
getAverageColinearity(std::vector<cv::Point2f>& pointbuf,
                      cv::Size size,
                      float &avgColinearity)
{
    std::vector<cv::Point2f> tmpPoints(size.width);
    cv::Vec4f tmpLine;
    float sumDistsPerLine = 0.0; 
    float sumDistances = 0.0; 
    cv::Point2f a, b;
    
    for(int i = 0; i < size.height; i++){                
        for(int j = i; j < size.width; j++){
            tmpPoints[j] = pointbuf[i*j];
        }
        
        /// Fit line
        fitLine(tmpPoints,tmpLine,cv::DIST_L2,0,0.01,0.01);
        
        /// tmpLine -> [vx, vy, x0, y0] -> (vx, vy) is a normalized vector collinear to the line and (x0, y0) is a point on the linefloat vx = tmpLine[0],vy = tmpLine[1], x0 = tmpLine[2],y0 = tmpLine[3];
        a = cv::Point2f(tmpLine[2], tmpLine[3]);
        b = cv::Point2f(tmpLine[0], tmpLine[1]);
        
        for(int k = 0; k < size.width; k++)
        {
            float t = ( tmpPoints[k].dot(b) - a.dot(b) ) / (cv::norm(b) * cv::norm(b));
            float dist = cv::norm(tmpPoints[k] - (a + t * b));
            sumDistsPerLine += dist;
        }
        sumDistances += sumDistsPerLine/size.width;        
    }
    avgColinearity = sumDistances/size.height; 
}

void
getFrontoParallelPoints(std::vector<cv::Point2f> &frontoPoints,
                        cv::Size imgSize,
                        cv::Size patternSize,
                        float distanceBP)
{
    /* std::cout<<"distanceBP: "<<distanceBP <<std::endl;
    std::cout<<"patternSize.width: "<<patternSize.width  <<std::endl;
    std::cout<<"patternSize.height: "<<patternSize.height <<std::endl; */
    int width, height;
    width = (patternSize.width+1)*distanceBP*1000;
    height = (patternSize.height+1)*distanceBP*1000;
    cv::Size frameSize(width, height);
    //std::cout<<"Frame Size: "<< frameSize<< std::endl;
    //cv::Mat frontoFrame = cv::Mat::zeros(height, width, CV_8UC3);
    
    float x, y;
    for(int i = 0 ; i < patternSize.height; i++){
        for(int j = 0; j < patternSize.width; j++){
            x = (j * distanceBP * 1000) + distanceBP*1000;
            y = (i * distanceBP * 1000)  + distanceBP*1000;
            //std::cout<<"X, Y -> "<< x <<", " << y << std::endl;
            //cv::circle(frontoFrame, cv::Point2f(x,y), 1, cv::Scalar(0, 0, 255), 4, 8);
            frontoPoints.push_back(cv::Point2f(x,y));                    
                    //std::cout<<"x: "<<x<<"y: "<<y <<std::endl;
        }
    }

    /* cv::namedWindow("Fronto Frame", cv::WINDOW_AUTOSIZE);
    imshow("Fronto Frame", frontoFrame);
     */
}

bool 
findPatternFP(cv::Mat src, 
              int pointsPerRow, 
              int pointsPerColum,
              std::vector<cv::Point2f> &pointbuffp){
    
    cv::Mat gray, bw;
    bool found = false;
    cv::Mat frontoFrame = cv::Mat::zeros(src.rows, src.cols, CV_8UC3);
        
    // Convert to grayscale and apply Gaussian blur
    // Reduce information and noise
    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0, 0);
    
    // Convert image to binary
    bw = cv::Mat::zeros(gray.size(), CV_8UC1);
    //cv::threshold(gray, bw, 100, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    //cv::adaptiveThreshold(gray, bw, 250, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 25, -1);
    cv::adaptiveThreshold(gray, bw, 200, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, 43, 3);
    //thresholdIntegral(gray, bw);
    if (displayCompletePreprocess == 1)
    {
        cv::namedWindow("FP Binary", cv::WINDOW_NORMAL);
        imshow("FP Binary", bw);
    }

    pointbuffp.clear();
    found = cv::findCirclesGrid(bw, cv::Size(pointsPerRow, pointsPerColum), pointbuffp);
    
    if (displayCompletePreprocess == 1)
    {
        for (int i = 0; i < pointbuffp.size(); i++)
        {
            cv::circle(frontoFrame, pointbuffp[i], 1, cv::Scalar(0, 0, 255), 4, 8);    
            cv::putText(frontoFrame, std::to_string(i), pointbuffp[i], cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(250, 0, 0), 2);
        }

        cv::namedWindow("Fronto Frame", cv::WINDOW_AUTOSIZE);
        imshow("Fronto Frame", frontoFrame);    
    }
    
    
    return found;
}
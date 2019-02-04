#include "../include/frameselect.h"

#define displaySelector 1

/// To control pattern distribution among the frame
int framePerQt;               /// One frame per grid space
const int sizeGrid = 7;                 /// This means we'll get 25 frames, one frame per grid space  
int gridValidation[sizeGrid][sizeGrid]; ///

bool 
isGoodFrame(cv::Mat src, 
            std::vector<cv::Point2f>& pointbuf, 
            std::vector<cv::Point2f>& previousCornersBuf, 
            bool previousCorners, 
            int patternWidth, 
            int patternHeigh,
            float &minDistControlPoints)
{
    int qtWidth, qtHeigh;
    int i = 8;
    bool good = true;
    float patternArea, totalArea;
    float totalDistDisplace = 0.0f;
    std::vector<cv::Point2f> patternCorners;
    std::vector<cv::Point2f> frameCorners;
    cv::Mat selector = cv::Mat::zeros(src.rows, src.cols, CV_8UC3);

    patternCorners.push_back(pointbuf[0]);
    patternCorners.push_back(pointbuf[patternWidth-1]);
    patternCorners.push_back(pointbuf[pointbuf.size()-patternWidth]);
    patternCorners.push_back(pointbuf[pointbuf.size()-1]);

    frameCorners.push_back(cv::Point2f(0,0));
    frameCorners.push_back(cv::Point2f(0,src.cols));
    frameCorners.push_back(cv::Point2f(src.rows,0));
    frameCorners.push_back(cv::Point2f(src.rows,src.cols));
    
    qtWidth = src.cols/(i-1);
    qtHeigh = src.rows/(i-1);

    if(displaySelector == 1)
    {
        /// Vertical lines
        line(selector, cv::Point2f(qtWidth,0), cv::Point2f(qtWidth, src.rows), cv::Scalar(0,0,255), 2, 8, 0);
        line(selector, cv::Point2f(src.cols-qtWidth,0), cv::Point2f(src.cols-qtWidth, src.rows), cv::Scalar(0,0,255), 2, 8, 0);
        
        /// Horizontal lines
        line(selector, cv::Point2f(0, qtHeigh), cv::Point2f(src.cols, qtHeigh), cv::Scalar(0,0,255), 2, 8, 0);
        line(selector, cv::Point2f(0, src.rows-qtHeigh), cv::Point2f(src.cols, src.rows-qtHeigh), cv::Scalar(0,0,255), 2, 8, 0);
        
        /// Draw pattern corners
        circle(selector, patternCorners[0], 1, cv::Scalar(255,0,0),6, 8, 0);
        circle(selector, patternCorners[1], 1, cv::Scalar(255,0,0),6, 8, 0);
        circle(selector, patternCorners[2], 1, cv::Scalar(255,0,0),6, 8, 0);
        circle(selector, patternCorners[3], 1, cv::Scalar(255,0,0),6, 8, 0);
    }
    
    /// Check pattern distribution
    if(good && (patternCorners[0].x < qtWidth || patternCorners[0].y < qtHeigh))
        good = false;
    if(good && (patternCorners[1].x > (i-1)*qtWidth || patternCorners[1].y < qtHeigh))
        good = false;
    if(good && (patternCorners[2].x < qtWidth || patternCorners[2].y > (i-1)*qtHeigh))
        good = false;
    if(good && (patternCorners[3].x > (i-1)*qtWidth || patternCorners[3].y > (i-1)*qtHeigh))
        good = false;

    /// Get Area realation
    cv::RotatedRect minRect = cv::minAreaRect(patternCorners);
    cv::RotatedRect minRect_ = cv::minAreaRect(frameCorners);
    
    patternArea = minRect.size.width * minRect.size.height;
    totalArea = minRect_.size.width * minRect_.size.height;
    
    //std::cout<<"T: " <<totalArea <<" -  P: "<< patternArea <<std::endl; 
        
    if(good && (patternArea < totalArea/(i-1))){
        good = false;
    }

    if(displaySelector == 1)
    {
        cv::Point2f rect_points[4]; minRect.points(rect_points);
        for( int j = 0; j < 4; j++ )
            line( selector, rect_points[j], rect_points[(j+1)%4], cv::Scalar(255,0,0), 1, 8 );
            
        cv::namedWindow("Selector", cv::WINDOW_NORMAL);
        imshow("Selector", selector);    
    }
    
    if(good)
    {
        //std::cout<<" - 0" <<std::endl;
        if(previousCorners)
        {
            //std::cout<<" - 1" <<std::endl;
            for( int k = 0; k < previousCornersBuf.size(); k++ ){
                totalDistDisplace += cv::norm(previousCornersBuf[k] - patternCorners[k]);
            }

            if((totalDistDisplace/previousCornersBuf.size()) > (minDistControlPoints/2))
            {
                //std::cout<<" - 3" <<std::endl;
                previousCornersBuf.clear();
                previousCornersBuf = patternCorners;                        
                return true;
            }
        }
        else
        {
            //std::cout<<" - 2" <<std::endl;
            previousCornersBuf = patternCorners;  
            return true;          
        }
    }
        
    return false;
}

void 
initializeGrid(int fpq)
{
    framePerQt = fpq;
    for(int i = 0; i < sizeGrid; i++){
        for(int j = 0; j < sizeGrid; j++){
            gridValidation[i][j] = 0;
        }
    }
}

bool 
isGoodFrameImp( cv::Mat src, 
                std::vector<cv::Point2f>& pointbuf, 
                std::vector<cv::Point2f>& previousCornersBuf, 
                bool previousCorners, 
                int patternWidth, 
                int patternHeigh,
                float &minDistControlPoints)
{
    int magicNumber = 13;
    float a, b, c, d;    
    a = 1.75; b = 2.0; c = 1.75; d = 2.0;
    int x0, y0, x1, y1, x2, y2;
    
    int qtWidth, qtHeigh;
    bool good = true;
    float totalDistDisplace = 0.0f;
    std::vector<cv::Point2f> patternCorners;
    
    cv::Mat selector = cv::Mat::zeros(src.rows, src.cols, CV_8UC3);

    patternCorners.push_back(pointbuf[0]);
    patternCorners.push_back(pointbuf[patternWidth-1]);
    patternCorners.push_back(pointbuf[pointbuf.size()-patternWidth]);
    patternCorners.push_back(pointbuf[pointbuf.size()-1]);
    
    qtWidth = src.cols/magicNumber;
    qtHeigh = src.rows/magicNumber;

    if(displaySelector == 1)
    {
        /// Vertical lines
        line(selector, cv::Point2f(a*qtWidth,0), cv::Point2f(a*qtWidth, src.rows), cv::Scalar(0,0,255), 2, 8, 0);
        line(selector, cv::Point2f(src.cols-(a*qtWidth),0), cv::Point2f(src.cols-(a*qtWidth), src.rows), cv::Scalar(0,0,255), 2, 8, 0);
        
        line(selector, cv::Point2f((a+b)*qtWidth,0), cv::Point2f((a+b)*qtWidth, src.rows), cv::Scalar(0,0,255), 2, 8, 0);
        line(selector, cv::Point2f(src.cols-(a+b)*qtWidth,0), cv::Point2f(src.cols-(a+b)*qtWidth, src.rows), cv::Scalar(0,0,255), 2, 8, 0);
        
        line(selector, cv::Point2f((a+b+c)*qtWidth,0), cv::Point2f((a+b+c)*qtWidth, src.rows), cv::Scalar(0,0,255), 2, 8, 0);
        line(selector, cv::Point2f(src.cols-(a+b+c)*qtWidth,0), cv::Point2f(src.cols-(a+b+c)*qtWidth, src.rows), cv::Scalar(0,0,255), 2, 8, 0);
        
        /// Horizontal lines
        line(selector, cv::Point2f(0, a*qtHeigh), cv::Point2f(src.cols, a*qtHeigh), cv::Scalar(0,0,255), 2, 8, 0);
        line(selector, cv::Point2f(0, src.rows-a*qtHeigh), cv::Point2f(src.cols, src.rows-a*qtHeigh), cv::Scalar(0,0,255), 2, 8, 0);
        
        line(selector, cv::Point2f(0, (a+b)*qtHeigh), cv::Point2f(src.cols, (a+b)*qtHeigh), cv::Scalar(0,0,255), 2, 8, 0);
        line(selector, cv::Point2f(0, src.rows-(a+b)*qtHeigh), cv::Point2f(src.cols, src.rows-(a+b)*qtHeigh), cv::Scalar(0,0,255), 2, 8, 0);
        
        line(selector, cv::Point2f(0, (a+b+c)*qtHeigh), cv::Point2f(src.cols, (a+b+c)*qtHeigh), cv::Scalar(0,0,255), 2, 8, 0);
        line(selector, cv::Point2f(0, src.rows-(a+b+c)*qtHeigh), cv::Point2f(src.cols, src.rows-(a+b+c)*qtHeigh), cv::Scalar(0,0,255), 2, 8, 0);
        
        /// Draw pattern corners
        circle(selector, patternCorners[0], 1, cv::Scalar(255,0,0),6, 8, 0);
        circle(selector, patternCorners[1], 1, cv::Scalar(255,0,0),6, 8, 0);
        circle(selector, patternCorners[2], 1, cv::Scalar(255,0,0),6, 8, 0);
        circle(selector, patternCorners[3], 1, cv::Scalar(255,0,0),6, 8, 0);
    }
    
    if(patternCorners[0].x > 0) x0 = 0;
    if(patternCorners[0].x > a*qtWidth) x0 = 1;
    if(patternCorners[0].x > (a+b)*qtWidth) x0 = 2;
    if(patternCorners[0].x > (a+b+c)*qtWidth) x0 = 3;
    if(patternCorners[0].x > (a+b+c+d)*qtWidth) x0 = 4;
    if(patternCorners[0].x > (2*a+b+c+d)*qtWidth) x0 = 5;
    if(patternCorners[0].x > (2*a+2*b+c+d)*qtWidth) x0 = 6;

    if(patternCorners[0].y > 0) y0 = 0;
    if(patternCorners[0].y > a*qtHeigh) y0 = 1;
    if(patternCorners[0].y > (a+b)*qtHeigh) y0 = 2;
    if(patternCorners[0].y > (a+b+c)*qtHeigh) y0 = 3;
    if(patternCorners[0].y > (a+b+c+d)*qtHeigh) y0 = 4;
    if(patternCorners[0].y > (2*a+b+c+d)*qtHeigh) y0 = 5;
    if(patternCorners[0].y > (2*a+2*b+c+d)*qtHeigh) y0 = 6;

    if(patternCorners[1].x > 0) x1 = 0;
    if(patternCorners[1].x > a*qtWidth) x1 = 1;
    if(patternCorners[1].x > (a+b)*qtWidth) x1 = 2;
    if(patternCorners[1].x > (a+b+c)*qtWidth) x1 = 3;
    if(patternCorners[1].x > (a+b+c+d)*qtWidth) x1 = 4;
    if(patternCorners[1].x > (2*a+b+c+d)*qtWidth) x1 = 5;
    if(patternCorners[1].x > (2*a+2*b+c+d)*qtWidth) x1 = 6;

    if(patternCorners[2].y > 0) y2 = 0;
    if(patternCorners[2].y > a*qtHeigh) y2 = 1;
    if(patternCorners[2].y > (a+b)*qtHeigh) y2 = 2;
    if(patternCorners[2].y > (a+b+c)*qtHeigh) y2 = 3;
    if(patternCorners[2].y > (a+b+c+d)*qtHeigh) y2 = 4;
    if(patternCorners[2].y > (2*a+b+c+d)*qtHeigh) y2 = 5;
    if(patternCorners[2].y > (2*a+2*b+c+d)*qtHeigh) y2 = 6;
    
    /// Check pattern distribution
    if(good && (x1 < 2+x0)) good = false;
    if(good && (y2 < 2+y0)) good = false;
        
    if(displaySelector == 1)
    {
        //cv::Point2f rect_points[4]; minRect.points(rect_points);
        /*for( int j = 0; j < 4; j++ )
            line( selector, rect_points[j], rect_points[(j+1)%4], cv::Scalar(255,0,0), 1, 8 );
          */  
        cv::namedWindow("Selector", cv::WINDOW_NORMAL);
        imshow("Selector", selector);    
    }
    
    if(good && gridValidation[x0][y0] < framePerQt){
        if(gridValidation[x0][y0] == 0){
            gridValidation[x0][y0]++;
            previousCornersBuf = patternCorners;  
            return true;
        }
        else if(previousCorners){                
            totalDistDisplace = cv::norm(previousCornersBuf[0] - patternCorners[0]);
            if(totalDistDisplace > minDistControlPoints/2)
            {
                gridValidation[x0][y0]++;
                previousCornersBuf.clear();
                previousCornersBuf = patternCorners;                                    
                return true;
            }
        }                
    }                     
            
    return false;
}
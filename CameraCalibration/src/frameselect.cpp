#include "../include/frameselect.h"

#define displaySelector 0
#define i 8 

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

bool 
isGoodFrameImp( cv::Mat src, 
                std::vector<cv::Point2f>& pointbuf, 
                std::vector<cv::Point2f>& previousCornersBuf, 
                bool previousCorners, 
                int patternWidth, 
                int patternHeigh,
                float &minDistControlPoints)
{
    int magicNumber = 16;
    float a, b, c;
    a = 2.0; b = 4.5; c = 3.0;

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
        
        /// Horizontal lines
        line(selector, cv::Point2f(0, a*qtHeigh), cv::Point2f(src.cols, a*qtHeigh), cv::Scalar(0,0,255), 2, 8, 0);
        line(selector, cv::Point2f(0, src.rows-a*qtHeigh), cv::Point2f(src.cols, src.rows-a*qtHeigh), cv::Scalar(0,0,255), 2, 8, 0);
        
        line(selector, cv::Point2f(0, (a+b)*qtHeigh), cv::Point2f(src.cols, (a+b)*qtHeigh), cv::Scalar(0,0,255), 2, 8, 0);
        line(selector, cv::Point2f(0, src.rows-(a+b)*qtHeigh), cv::Point2f(src.cols, src.rows-(a+b)*qtHeigh), cv::Scalar(0,0,255), 2, 8, 0);
        
        /// Draw pattern corners
        circle(selector, patternCorners[0], 1, cv::Scalar(255,0,0),6, 8, 0);
        circle(selector, patternCorners[1], 1, cv::Scalar(255,0,0),6, 8, 0);
        circle(selector, patternCorners[2], 1, cv::Scalar(255,0,0),6, 8, 0);
        circle(selector, patternCorners[3], 1, cv::Scalar(255,0,0),6, 8, 0);
    }
    
    /// Check pattern distribution
    if(good && (patternCorners[0].x > (a+b)*qtWidth || patternCorners[0].y > (a+b)*qtHeigh))
        good = false;
    if(good && (patternCorners[1].x < (magicNumber-(a+b))*qtWidth || patternCorners[1].y > (a+b)*qtHeigh))
        good = false;
    if(good && (patternCorners[2].x > (a+b)*qtWidth || patternCorners[2].y < (magicNumber-(a+b))*qtHeigh))
        good = false;
    if(good && (patternCorners[3].x < (magicNumber-(a+b))*qtWidth || patternCorners[3].y < (magicNumber-(a+b))*qtHeigh))
        good = false;

    if(good && (patternCorners[0].x < a*qtWidth || patternCorners[0].y < a*qtHeigh))
        good = false;
    if(good && (patternCorners[1].x > (magicNumber-a)*qtWidth || patternCorners[1].y < a*qtHeigh))
        good = false;
    if(good && (patternCorners[2].x < a*qtWidth || patternCorners[2].y > (magicNumber-a)*qtHeigh))
        good = false;
    if(good && (patternCorners[3].x > (magicNumber-a)*qtWidth || patternCorners[3].y > (magicNumber-a)*qtHeigh))
        good = false;

    /// Get Area realation
    /*cv::RotatedRect minRect = cv::minAreaRect(patternCorners);
    cv::RotatedRect minRect_ = cv::minAreaRect(frameCorners);
    
    patternArea = minRect.size.width * minRect.size.height;
    totalArea = minRect_.size.width * minRect_.size.height;
    */
    //std::cout<<"T: " <<totalArea <<" -  P: "<< patternArea <<std::endl; 
        
    /*if(good && (patternArea < totalArea/(i-1))){
        good = false;
    }*/

    if(displaySelector == 1)
    {
        //cv::Point2f rect_points[4]; minRect.points(rect_points);
        /*for( int j = 0; j < 4; j++ )
            line( selector, rect_points[j], rect_points[(j+1)%4], cv::Scalar(255,0,0), 1, 8 );
          */  
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

            if((totalDistDisplace/previousCornersBuf.size()) > minDistControlPoints/2)
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
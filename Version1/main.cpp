#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "include/patrecog.h"

#define display 0
#define displayTest 0
#define displayContinuosly 0
#define printFrameCount 1
#define printTime 1

//#define video_path "../../../videos/PadronAnillos_01.avi"  // 30 Anillos
#define video_path "../../../videos/padron2.avi"  // 20 Anillos
//#define video_path "../../../videos/padron1.avi"  // 12 Anillos

enum Pattern { CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID, RINGS_GRID };
cv::Size patternSizes[] = { cv::Size(9,6), cv::Size(4,5), cv::Size(4,11), cv::Size(6,5)}; // Accoring to pattern

int main()
{
    Pattern pattern = RINGS_GRID;
    int patternSize = patternSizes[pattern].width*patternSizes[pattern].height;
    cv::Mat frame, view;
            
    bool found = false;
    bool previousFound = false;
    
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    std::vector<cv::Point2f> pointbuf;
    std::vector<cv::Point2f> prevoiusPointbuf;
   
    /// Testing variables
    int frameCount = 0;
    int frameCountLess = 0;
    int frameCountMore = 0;
    int frameCountFound = 0;
    int frameCountNotFound = 0;
    float percentage = 0.0;
    
    /// Time algorithm
    clock_t start, end;
    float sumTime = 0.0f;

    cv::VideoCapture capture(video_path);

    for (;;)
    {
        capture >> frame;
        if (frame.empty())
            break;

        frameCount++; 
        start = clock();       

        ///**** PREPROCESSING IMAGE ****///
        preprocessImage(frame, view, contours, hierarchy);
        //std::cout<<"Contours: "<<contours.size()<<std::endl;
        //std::cout<<"Hierarchy: "<<hierarchy.size()<<std::endl;

        ///*** IDENTIFY RINGS ***///
        identifyRings(contours, hierarchy, pointbuf, patternSize, frame);
        //std::cout<<"PointBufferSize: "<<pointbuf.size()<<std::endl;
        
        ///*** FIND RINGS GRID ***///
        if(pointbuf.size() == patternSize)
        {
            found = findRingsGrid(frame, patternSizes[pattern], pointbuf, prevoiusPointbuf, previousFound);            
            if(found){
                previousFound = true;
                prevoiusPointbuf = pointbuf;
                cv::drawChessboardCorners(frame, cv::Size(patternSizes[pattern].width,patternSizes[pattern].height), pointbuf,found);
                frameCountFound++;
            }
            else{
                //std::cout<<frameCount<<" - "<<found<<std::endl;                
                previousFound = false;
                frameCountNotFound++;
            }
        }
        else
        {
            if(pointbuf.size() > patternSize) frameCountMore++;
            if(pointbuf.size() < patternSize) frameCountLess++;
            previousFound = false;
        }
        
        /*switch(pattern)
        {
            case CHESSBOARD:
                found = cv::findChessboardCorners( view, patternSizes[pattern], pointbuf );
                break;
            case CIRCLES_GRID:
                found = cv::findCirclesGrid( view, patternSizes[pattern], pointbuf );
                break;
            case ASYMMETRIC_CIRCLES_GRID:
                found = findCirclesGrid( view, patternSizes[pattern], pointbuf );
                break;
            case RINGS_GRID:
                found = findRingsGrid( view, patternSizes[pattern], pointbuf );
            default:
                found = false;
        }*/

        if(display == 1)
        {
            cv::namedWindow("Video Display", cv::WINDOW_NORMAL);
            imshow("Video Display", frame);
            if(displayContinuosly == 1)
                cv::waitKey(20);
            else
            {
                if( cv::waitKey() == 27 )
                    cv::waitKey(100);        
            }
        }

        if(printTime == 1){
            end = clock();
            sumTime += (end - start)/(double)( CLOCKS_PER_SEC / 1000 );

            /*if (frameCount % 20 == 0)
            {
                sumTime = sumTime/20.0;
                //std::cout << "Frame: " << frameCount << " Time: " << (end - start)/(double)( CLOCKS_PER_SEC / 1000 ) << " ms." <<std::endl;
                std::cout << (end - start)/(double)( CLOCKS_PER_SEC / 1000 ) <<std::endl;
                //std::cout << sumTime <<std::endl;
                sumTime = 0.0;
            }*/
        }

        contours.clear();
        hierarchy.clear();
        pointbuf.clear();
    }

    if(printFrameCount == 1){
        std::cout<<"Complete rings were detected in "<<frameCountFound <<" out of " <<frameCount<< " frames"<<std::endl;
        std::cout<<"--> "<<(frameCountFound * 100)/frameCount <<"% frames"<<std::endl;
        std::cout<<"Average time pattern detection "<<sumTime/frameCount <<std::endl;
        std::cout<<"-----------------------------------"<<std::endl;
        std::cout<<"Less than pattern size "<<frameCountLess <<std::endl;
        std::cout<<"More than pattern size "<<frameCountMore <<std::endl;            
    }
}
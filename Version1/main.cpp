#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "include/patrecog.h"

#define display 1
#define displayTest 0
#define displayContinuosly 0

#define video_path "../../../videos/PadronAnillos_01.avi"  // 30 Anillos
//#define video_path "../../../videos/padron2.avi"  // 20 Anillos
//#define video_path "../../../videos/padron1.avi"  // 12 Anillos



enum Pattern { CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID, RINGS_GRID };
cv::Size patternSizes[] = { cv::Size(9,6), cv::Size(4,5), cv::Size(4,11), cv::Size(4,3)}; // Accoring to pattern

int main()
{
    Pattern pattern = RINGS_GRID;
    int patternSize = patternSizes[pattern].width*patternSizes[pattern].height;
    cv::Mat frame, view;
            
    bool found;
    bool previousFound = false;


    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    std::vector<cv::Point2f> pointbuf;
    std::vector<cv::Point2f> prevoiusPointbuf;
    
    
    //Testing variables
    int frameCount = 0;
    int frameCountLess = 0;
    int frameCountMore = 0;
    


    cv::VideoCapture capture(video_path);

    for (;;)
    {
        capture >> frame;
        if (frame.empty())
            break;

        frameCount++;        

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
            found = findRingsGrid(view, patternSizes[pattern], pointbuf, prevoiusPointbuf, previousFound);
            if(found){
                previousFound = true;
                prevoiusPointbuf = pointbuf;
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

        contours.clear();
        hierarchy.clear();
        pointbuf.clear();
    }
}
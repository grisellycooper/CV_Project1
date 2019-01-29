#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "include/patrecog.h"

#define display 1
#define displayContinuosly 1
#define printFrameCount 1
#define printTime 0

//#define video_path "../../../videos/PadronCirculos_02.avi" // 4 x 11 circulos

//#define video_path "../../../videos/PadronAnillos_01.avi"  // 30 Anillos
//#define video_path "../../../videos/padron2.avi" // 20 Anillos
//#define video_path "../../../videos/padron1.avi"  // 12 Anillos
#define video_path "../../../videos/cam1_anillos.mp4"  // 12 Anillos
//#define video_path "../../../videos/cam2_anillos.avi"  // 12 Anillos

/// Test cam1
//#define video_path "../../../videos/cam1/anillos.mp4"  // 5 x 4
//#define video_path "../../../videos/cam1/asymcir.mp4"  // 4 x 11
//#define video_path "../../../videos/cam1/chess2.mp4"  // 8 x 6

/// Test cam2
//#define video_path "../../../videos/cam2/anillos.avi"  // 5 x 4
//#define video_path "../../../videos/cam2/circulos.avi"  // 4 x 11
//#define video_path "../../../videos/cam2/chess.avi"  // 8 x 6

enum Pattern
{
    CHESSBOARD,
    CIRCLES_GRID,
    ASYMMETRIC_CIRCLES_GRID,
    RINGS_GRID
};
cv::Size patternSizes[] = {cv::Size(8, 6), cv::Size(5, 5), cv::Size(4, 11), cv::Size(5, 4)}; // Accoring to pattern

int 
main()
{
    Pattern pattern = RINGS_GRID;
    int patternSize = patternSizes[pattern].width * patternSizes[pattern].height;
    cv::Mat frame, view;
    float frWidth, frHeight, frFPS;

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

    int test = 0;

    cv::VideoCapture capture(video_path);
    
    for (;;)
    {
        //std::string filename = "../data/InnerCircles" + std::to_string(i) + ".jpg";
        //frame = cv::imread(filename, cv::IMREAD_COLOR);
        
        capture >> frame;                
        if (frame.empty())
            break;        

        /// Getting frame features
        frWidth = capture.get(cv::CAP_PROP_FRAME_WIDTH );
        frHeight = capture.get(cv::CAP_PROP_FRAME_HEIGHT );
        frFPS =  capture.get(cv::CAP_PROP_FPS);
        
        /*frWidth = frame.size().width;
        frHeight = frame.size().height;
        frFPS =  capture.get(cv::CAP_PROP_FPS);        */

        //std::cout<<"Frame: " << frWidth <<" x " <<frHeight <<" - "<<frFPS<<std::endl;

        frameCount++;
        found = false;
        start = clock();
        
        //std::cout<<"Frame: "<<frameCount<<" -----------------------------------------------------------" <<std::endl;

        switch (pattern)
        {
        case CHESSBOARD:
            found = cv::findChessboardCorners(frame, patternSizes[pattern], pointbuf, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
            break;
        case CIRCLES_GRID:
            found = cv::findCirclesGrid(frame, patternSizes[pattern], pointbuf);
            break;
        case ASYMMETRIC_CIRCLES_GRID:
            found = cv::findCirclesGrid(frame, patternSizes[pattern], pointbuf, cv::CALIB_CB_ASYMMETRIC_GRID);
            break;
        case RINGS_GRID:
            ///**** PREPROCESSING IMAGE ****///
            preprocessImage(frame, view, contours, hierarchy);
            //std::cout<<"Contours: "<<contours.size()<<std::endl;
            //std::cout<<"Hierarchy: "<<hierarchy.size()<<std::endl;

            ///*** IDENTIFY RINGS ***///
            identifyRings(contours, hierarchy, pointbuf, patternSize, frame);
            //std::cout<<"PointBufferSize: "<<pointbuf.size()<<"  ";
            
            ///*** FIND RINGS GRID ***///
            //std::cout<<"pointbufSize: " <<pointbuf.size() <<std::endl;
            if (pointbuf.size() == patternSize)
            {   
                found = findRingsGrid(frame, patternSizes[pattern], pointbuf, prevoiusPointbuf, previousFound);                   
                /*if(found)
                    std::cout<<"patternSizeFound!!!" <<std::endl;*/
            }
            else
            {
                if (pointbuf.size() > patternSize)
                    frameCountMore++;
                if (pointbuf.size() < patternSize)
                    frameCountLess++;
                previousFound = false;
            }   
            //std::cout<<frameCount<<"  -  "<<found <<std::endl;         
            break;
        default:
            found = false;
        }

        if (found)
        {   
            if(pattern == CHESSBOARD){
                cv::TermCriteria criteria(cv::TermCriteria::COUNT + cv::TermCriteria::MAX_ITER, 30, 0.001);
                cv::Mat gray;
                cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
                cv::cornerSubPix(gray, pointbuf, cv::Size(11,11), cv::Size(-1,-1),criteria);
		    }

            frameCountFound++;
            cv::drawChessboardCorners(frame, patternSizes[pattern], pointbuf, found);
            
            end = clock();
            sumTime += (end - start) / (double)(CLOCKS_PER_SEC / 1000);
            
            previousFound = true;
            prevoiusPointbuf = pointbuf;              
        }
        else
        {
            //std::cout<<" NotFound "<<std::endl;
            previousFound = false;
            frameCountNotFound++;
        }

        if (display == 1)
        {
            cv::namedWindow("Video Display", cv::WINDOW_NORMAL);
            imshow("Video Display", frame);
            
            if (displayContinuosly == 1)
                cv::waitKey(20);
            else
            {
                if (cv::waitKey() == 27)
                    cv::waitKey(100);
            }
        }

        if (printTime == 1)
        {
            //std::cout<<" Time:  ";
            
            if (frameCount % 20 == 0)
            {
                sumTime = sumTime / 20.0;
                //std::cout << "Frame: " << frameCount << " Time: " << (end - start)/(double)( CLOCKS_PER_SEC / 1000 ) << " ms." <<std::endl;
                //std::cout << (end - start) / (double)(CLOCKS_PER_SEC / 1000) << std::endl;
                std::cout << sumTime <<std::endl;
                sumTime = 0.0;
            }
        }

        contours.clear();
        hierarchy.clear();
        pointbuf.clear();
    }
    
    if (printFrameCount == 1)
        {            
            std::cout << "Complete rings were detected in " << frameCountFound << " out of " << frameCount << " frames" << std::endl;
            std::cout << "--> " << (frameCountFound * 100) / frameCount << "% frames" << std::endl;
            std::cout << "Average time pattern detection " << sumTime / frameCount << std::endl;
            std::cout << "-----------------------------------" << std::endl;
            /*std::cout << "Less than pattern size " << frameCountLess << std::endl;
            std::cout << "More than pattern size " << frameCountMore << std::endl;*/
        }
}

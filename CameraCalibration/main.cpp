#include <stdio.h>
#include <iostream>
#include <cstring>
#include <dirent.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "include/patrecog.h"
#include "include/frameselect.h"

#define display 1
#define displayContinuosly 0
#define printFrameCount 1
#define printTime 0

enum Pattern
{
    CHESSBOARD,
    CIRCLES_GRID,
    ASYMMETRIC_CIRCLES_GRID,
    RINGS_GRID
};
cv::Size patternSizes[] = {cv::Size(8, 6), cv::Size(5, 5), cv::Size(4, 11), cv::Size(5, 4)}; // Accoring to pattern

void 
getFiles (std::string dir, std::vector<std::string> &files)
{
    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(dir.c_str())) == NULL) 
        std::cout << "Error(" << errno << ") opening " << dir << std::endl;
    
    while ((dirp = readdir(dp)) != NULL) 
    {
        if(strcmp(dirp->d_name, ".") != 0 && strcmp(dirp->d_name, "..") != 0)
            files.push_back(std::string(dirp->d_name));
    }
    closedir(dp);
}

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

    /// Frame selection
    int countGood = 0;
    std::vector<cv::Point2f> previousCornersBuf;
    bool previousCorners = false;
    float minDistControlPoints = 0.0f;                   /// MinDistance between two control points among all pattern

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

    std::string dir = "../data/20_CP/";
    std::vector<std::string> files = std::vector<std::string>();

    getFiles(dir,files);

    for (int i = 0;i < files.size();i++) 
    {
        //std::cout << files[i] << std::endl;
        frame = cv::imread(dir+files[i], cv::IMREAD_COLOR);
        if( frame.empty() )
        {
            std::cout << "Couldn't load " << files[i] << std::endl;
            continue;
        }        

        view = frame.clone();
                
        frWidth = frame.size().width;
        frHeight = frame.size().height;
        /*frFPS =  capture.get(cv::CAP_PROP_FPS);        */

        //std::cout<<"Frame: " << frWidth <<" x " <<frHeight <<" - "<<frFPS<<std::endl;

        frameCount++;
        found = false;
        start = clock();
        
        std::cout<<"Frame: "<<frameCount<<" -----------------------------------------------------------" <<std::endl;

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
                found = findRingsGrid(frame, patternSizes[pattern], pointbuf, prevoiusPointbuf, false, minDistControlPoints);                   
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
            
            /*end = clock();
            sumTime += (end - start) / (double)(CLOCKS_PER_SEC / 1000);*/
                       
                           
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
    }
}

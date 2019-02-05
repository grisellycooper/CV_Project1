#include <stdio.h>
#include <iostream>
#include <cstring>
#include <dirent.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "include/patrecog.h"
#include "include/frameselect.h"
#include "include/cameracalib.h"

#define display 1
#define displayContinuosly 1
#define printFrameCount 1
#define printTime 0

#define cam "cam1"
#define frameSample 25

enum Pattern
{
    CHESSBOARD,
    CIRCLES_GRID,
    ASYMMETRIC_CIRCLES_GRID,
    RINGS_GRID
};
cv::Size patternSizes[] = {cv::Size(8, 6), cv::Size(5, 5), cv::Size(4, 11), cv::Size(5, 4)}; // Accoring to pattern
float ctrlPointDistances[] = {0.026, 0.0, 0.0375, 0.0455}; // Meters real distances

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
    std::string camera(cam); 

    /// Camera calibration
    std::vector<std::vector<cv::Point2f>> imagePoints;
    float avgColinearityPerFrame = 0.0;
    float finalAvgColinearity = 0.0;
    double rms;

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
    
    std::string dir = "../data/"+ camera + "/" +std::to_string(frameSample)+"/";		        
    std::vector<std::string> files = std::vector<std::string>();

    getFiles(dir,files);

    ///******* TEST ****///


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
            
            getAverageColinearity(pointbuf, patternSizes[pattern], avgColinearityPerFrame);
            finalAvgColinearity += avgColinearityPerFrame; 
            
            imagePoints.push_back(pointbuf);
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
                    cv::waitKey(50);
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

    /// Average colinearity before calibration
    std::cout<<"Average Colinearity Before Undistortion: "<< finalAvgColinearity/frameCount <<std::endl;

    ///** Parallel thing **//
    cv::Mat temp, imgWarp, imgWarp_inv;
    cv::Mat H;              /// Homography
    std::vector<cv::Point2f> undistpointbuf;
    std::vector<cv::Point2f> pointbuffp;
    std::vector<cv::Point2f> pointbuffp_;
    std::vector<cv::Point2f> corrected_points;
    std::vector<std::vector<cv::Point2f>> imagePointsFP;
    std::vector<std::vector<cv::Point2f>> tmpimagepoints;
    
    std::vector<cv::Point2f> frontopoints;
    getFrontoParallelPoints(frontopoints, cv::Size(frWidth, frHeight), patternSizes[pattern], ctrlPointDistances[pattern]);
    
    ///** Initial Camera Calibration **///        
    std::vector<std::vector<cv::Point3f>> objectPoints(1);                    
    getControlPointsPositions(patternSizes[pattern], ctrlPointDistances[pattern], objectPoints[0], pattern);
    objectPoints.resize(imagePoints.size(),objectPoints[0]);

    cv::Mat cameraMatrix = cv::Mat::eye(3,3,CV_64F);
    cv::Mat distCoeffs = cv::Mat::zeros(8, 1,CV_64F);    
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;

    frameCount = 0;
    bool flag = true;
    
    for (int i = 0; i < 6; i++)
    {
        std::cout<<"-- imagePoints: "<<imagePoints.size()<<std::endl;
                
        rms = calibrateCamera(objectPoints, imagePoints, cv::Size(frWidth,frHeight), cameraMatrix, distCoeffs, rvecs, tvecs);

        bool ok = (checkRange(cameraMatrix) && checkRange(distCoeffs));
        if(!ok)
            std::cout<<"Camera was not calibrated" <<std::endl;
                        
        std::cout <<std::endl;
        std::cout << "RMS error reported by calibrateCamera: " << rms << std::endl;                
        std::cout << "Intrinsic camera matrix" << std::endl << cameraMatrix << std::endl;
        std::cout << "Distortion coefficients" << std::endl << distCoeffs << std::endl;
        std::cout << "----------------------------" << std::endl;        
        
        if(flag)
        {       
            imagePointsFP.clear();
            bool foundFP;
            finalAvgColinearity = 0; 

            int width = (patternSizes[pattern].width+1)*ctrlPointDistances[pattern]*1000;
            int height = (patternSizes[pattern].height+1)*ctrlPointDistances[pattern]*1000;

            for (int i = 0;i < files.size();i++) 
            {
                //std::cout << files[i] << std::endl;
                frame = cv::imread(dir+files[i], cv::IMREAD_COLOR);
                if( frame.empty() )
                {
                    std::cout << "Couldn't load " << files[i] << std::endl;
                    continue;
                } 

                frameCount++;
                
                /*frWidth = frame.size().width;
                frHeight = frame.size().height;
                std::cout<<"Frame: " << frWidth <<" x " <<frHeight <<std::endl;*/
                //std::cout<<"Frame: "<<++frameCount<<" -----------------------------------------------------------" <<std::endl;
                
                temp = frame.clone();
                cv::Mat OptimalMatrix = cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, cv::Size(frWidth,frHeight), 1.0);
                cv::undistort(temp,frame,cameraMatrix,distCoeffs,OptimalMatrix);                    
                cv::undistortPoints(imagePoints[i], undistpointbuf, cameraMatrix, distCoeffs, cv::noArray(), OptimalMatrix);

                cv::namedWindow("Undistort frame", cv::WINDOW_NORMAL);
                imshow("Undistort frame", frame);

                getAverageColinearity(undistpointbuf, patternSizes[pattern], avgColinearityPerFrame);
                finalAvgColinearity += avgColinearityPerFrame; 
                    
                /*getAverageColinearity(imagePoints[i], patternSizes[pattern], avgColinearityPerFrame);
                std::cout<<"Old avgColinearity: "<<avgColinearityPerFrame <<std::endl;

                getAverageColinearity(pointbufpar, patternSizes[pattern], avgColinearityPerFrame);
                std::cout<<"New avgColinearity: " <<avgColinearityPerFrame <<std::endl;; */
                                
                /*std::cout<<"imagePoints - new pointbuffer" <<std::endl;
                for(int l = 0; l < pointbufpar.size(); l++){
                    std::cout<<" " <<imagePoints[i][l].x <<", " <<imagePoints[i][l].y <<"  /-/  " <<pointbufpar[l].x <<", " <<pointbufpar[l].y<<std::endl;            
                }*/

                H = cv::findHomography(undistpointbuf,frontopoints);	        
                cv::warpPerspective(temp,imgWarp,H,cv::Size(width, height));
                    
                cv::namedWindow("ImgWarp", cv::WINDOW_AUTOSIZE); 
                imshow("ImgWarp", imgWarp);       

                ///*** Find pattern ***///
                foundFP = false;
                pointbuffp.clear();		
                
                foundFP = findPatternFP(imgWarp, patternSizes[pattern].width, patternSizes[pattern].height, pointbuffp);
                            
                if(foundFP){
                    cv::warpPerspective(imgWarp,imgWarp_inv,H.inv(),temp.size());                
                    cv::perspectiveTransform( pointbuffp, pointbuffp_, H.inv() );
                    cv::undistortPoints(pointbuffp_,corrected_points,OptimalMatrix,-distCoeffs,cv::noArray(),cameraMatrix);
                        
                    imagePointsFP.push_back(corrected_points);
                    //std::cout<<"imagePointsFP.size " <<imagePointsFP.size() <<std::endl;

                    cv::drawChessboardCorners(imgWarp_inv, patternSizes[pattern], corrected_points, foundFP);
                        
                    cv::namedWindow("ImgWarp Inv", cv::WINDOW_NORMAL); 
                    imshow("ImgWarp Inv", imgWarp_inv);
                }
                else{
                    std::cout << "Pattern NOT found in the parallel frame!\n";
                }
                
                if (cv::waitKey() == 27)
                    cv::waitKey(100);
                        
            }

            /// Replace old image points
            //imagePoints.clear();
            //imagePoints = imagePointsFP;
            
            /// Get the average
            tmpimagepoints.clear();
            tmpimagepoints = imagePoints;

            std::cout<<"imagePoints: "<<imagePoints.size()<<std::endl;
            std::cout<<"imagePointsFP: "<<imagePointsFP.size()<<std::endl;
            std::cout<<"tmpimagepoints: "<<tmpimagepoints.size()<<std::endl;

            /* std::cout<<"tmpimagepoints ------------: "<<std::endl;

            for(int ip = 0; ip < patternSize; ip++){
                std::cout<<ip <<" -> "<<tmpimagepoints[24][ip] <<std::endl;
            } */

            for (int f = 0; f < frameCount; f++){
                for(int ip = 0; ip < tmpimagepoints.size(); ip++){
                    imagePoints[f][ip].x = (tmpimagepoints[f][ip].x + imagePointsFP[f][ip].x)/2;
                    imagePoints[f][ip].y = (tmpimagepoints[f][ip].y + imagePointsFP[f][ip].y)/2;
                }
            }        

            /* std::cout<<"ImagePoints ------------: "<<std::endl;
            for(int ip = 0; ip < patternSize; ip++){
                std::cout<<ip <<" -> "<<imagePoints[24][ip] <<std::endl;
            } */

            std::cout<<"imagePoints: "<<imagePoints.size()<<std::endl;
            std::cout<<"Average Colinearity After Undistorion: "<< finalAvgColinearity/frameCount <<std::endl;

            //flag = false;
        }
    }    
}
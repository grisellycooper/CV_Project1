#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "include/patrecog.h"
#include "include/camcalib.h"

#define display 1
#define displayTest 0
#define displayContinuosly 1
#define printFrameCount 0
#define printTime 0

//#define video_path "../../../videos/PadronCirculos_02.avi" // 4 x 11 circulos

//#define video_path "../../../videos/PadronAnillos_01.avi"  // 30 Anillos
//#define video_path "../../../videos/padron2.avi" // 20 Anillos
//#define video_path "../../../videos/padron1.avi"  // 12 Anillos

/// Test cam1
#define video_path "../../../videos/cam1/anillos.mp4"  // 5 x 4
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
float ctrlPointDistances[] = {0.026, 0.0, 0.0375, 0.0455}; // Meters real distances

static double computeReprojectionErrors( const std::vector<std::vector<cv::Point3f> >& objectPoints,
                                         const std::vector<std::vector<cv::Point2f> >& imagePoints,
                                         const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
                                         const cv::Mat& cameraMatrix , const cv::Mat& distCoeffs,
                                         std::vector<float>& perViewErrors);
int main()
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

    /// Camera Calibration variables
    int nrSkip = 20;
    int nrSamples = 25;

    std::vector<std::vector<cv::Point3f>> objectPoints;
    std::vector<std::vector<cv::Point2f>> imagePoints;
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    std::vector<float> perViewError;
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
        //start = clock();
        
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
            //std::cout<<"PointBufferSize: "<<pointbuf.size()<<std::endl;
            
            ///*** FIND RINGS GRID ***///
            if (pointbuf.size() == patternSize)
            {   
                found = findRingsGrid(frame, patternSizes[pattern], pointbuf, prevoiusPointbuf, previousFound);                   
            }
            else
            {
                if (pointbuf.size() > patternSize)
                    frameCountMore++;
                if (pointbuf.size() < patternSize)
                    frameCountLess++;
                previousFound = false;

            }            
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

            /*end = clock();
            sumTime += (end - start) / (double)(CLOCKS_PER_SEC / 1000);*/
            
            previousFound = true;
            prevoiusPointbuf = pointbuf;
            cv::drawChessboardCorners(frame, patternSizes[pattern], pointbuf, found);
            frameCountFound++;
            //std::cout<<"frame: "<<frameCount <<" - found!"<<std::endl;

            //Add pointBuffer to imagePoint
            if(frameCountFound % nrSkip == 0){
                imagePoints.push_back(pointbuf);
                //std::cout<<"frame capturado: " <<imagePoints.size()<<std::endl; 
                //std::cout << imagePoints.size() <<" ";
            }
            
            /// When it riches predefine nro of Sample, calibrate camera
            if(imagePoints.size() == nrSamples){  
                //std::cout<<imagePoints              //Calibrate camera
                std::vector<std::vector<cv::Point3f>> objectPoints(1);                
                getControlPointsPositions(patternSizes[pattern], ctrlPointDistances[pattern], objectPoints[0], pattern);
                objectPoints.resize(imagePoints.size(),objectPoints[0]);

                /*std::cout << " image Points" <<std::endl;
                for(int i = 0; i < imagePoints.size(); i++){

                    std::cout <<"xy: " <<imagePoints[i] << std::endl;  
                }*/

                std::cout<<objectPoints.size() <<" - "<<imagePoints.size()<<" - "<<frWidth << " - " <<frHeight <<std::endl;
                rms = calibrateCamera(objectPoints, imagePoints, cv::Size(frWidth,frHeight), cameraMatrix, distCoeffs, rvecs, tvecs);
    			std::cout <<std::endl;
                std::cout << "RMS error reported by calibrateCamera: " << rms << std::endl;                
                std::cout << "Intrinsic camera matrix" << std::endl << cameraMatrix << std::endl;
                std::cout << "Distortion coefficients" << std::endl << distCoeffs << std::endl;
                std::cout << "----------------------------" << std::endl;
                
                std::vector<float> reprojErrs;
                bool ok = (checkRange(cameraMatrix) && checkRange(distCoeffs));
                if(!ok)
                    std::cout<<"Camera was not calibrated" <<std::endl;
                
                double totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints, rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);
                std::cout<<"Average reprojection error: " << totalAvgErr <<std::endl;

                imagePoints.clear();                
            }   
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
            
            /*if (frameCount % 20 == 0)
            {
                sumTime = sumTime / 20.0;
                //std::cout << "Frame: " << frameCount << " Time: " << (end - start)/(double)( CLOCKS_PER_SEC / 1000 ) << " ms." <<std::endl;
                std::cout << (end - start) / (double)(CLOCKS_PER_SEC / 1000) << std::endl;
                //std::cout << sumTime <<std::endl;
                sumTime = 0.0;
            }*/
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

        contours.clear();
        hierarchy.clear();
        pointbuf.clear();
        
    }

    /*if (printFrameCount == 1)
    {
        std::cout << "Complete rings were detected in " << frameCountFound << " out of " << frameCount << " frames" << std::endl;
        std::cout << "--> " << (frameCountFound * 100) / frameCount << "% frames" << std::endl;
        std::cout << "Average time pattern detection " << sumTime / frameCount << std::endl;
        std::cout << "-----------------------------------" << std::endl;
        std::cout << "Less than pattern size " << frameCountLess << std::endl;
        std::cout << "More than pattern size " << frameCountMore << std::endl;
    }*/
}

static double computeReprojectionErrors( const std::vector<std::vector<cv::Point3f> >& objectPoints,
                                         const std::vector<std::vector<cv::Point2f> >& imagePoints,
                                         const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
                                         const cv::Mat& cameraMatrix , const cv::Mat& distCoeffs,
                                         std::vector<float>& perViewErrors)
{
    std::vector<cv::Point2f> imagePoints2;
    int i, totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for( i = 0; i < (int)objectPoints.size(); ++i )
    {
        projectPoints( cv::Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,
                       distCoeffs, imagePoints2);
        err = cv::norm(cv::Mat(imagePoints[i]) - cv::Mat(imagePoints2));

        int n = (int)objectPoints[i].size();
        perViewErrors[i] = (float) std::sqrt(err*err/n);
        totalErr        += err*err;
        totalPoints     += n;
    }

    return std::sqrt(totalErr/totalPoints);
}
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "include/patrecog.h"
#include "include/camcalib.h"

#define display 1
#define displayTest 0
#define displayContinuosly 1
#define printFrameCount 1
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

//** helping functions **//
std::vector<cv::Point2f> getFrontoParallelCorners(cv::Size imgSize, cv::Size patternSize);
std::vector<cv::Point2f> getPatternCorners(std::vector<cv::Point2f>& v, cv::Size size);

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
    int nrSkip = 35;
    int nrSamples = 25;
    bool undistort = false;

    /// Fronto Parallel
    std::vector<cv::Mat> selectedFrames;
        

    //std::vector<std::vector<cv::Point3f>> objectPoints;
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
            //std::cout<<"PointBufferSize: "<<pointbuf.size()<<"  ";
            
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

            /*end = clock();
            sumTime += (end - start) / (double)(CLOCKS_PER_SEC / 1000);*/
            
            previousFound = true;
            prevoiusPointbuf = pointbuf;
            //std::cout<<"frame: "<<frameCount <<" - found!"<<std::endl;

            //Add pointBuffer to imagePoint
            if(frameCountFound % nrSkip == 0){
                imagePoints.push_back(pointbuf);
                selectedFrames.push_back(frame.clone());
                std::cout<<"frame capturado: " <<imagePoints.size()<<std::endl; 
                //std::cout << imagePoints.size() <<" ";
            }
            
            cv::drawChessboardCorners(frame, patternSizes[pattern], pointbuf, found);
            frameCountFound++;
            
            /// When it riches predefine nro of Sample, calibrate camera
            if(imagePoints.size() == nrSamples)
            {  
                //std::cout<<imagePoints              //Calibrate camera
                std::vector<std::vector<cv::Point3f>> objectPoints(1);                
                getControlPointsPositions(patternSizes[pattern], ctrlPointDistances[pattern], objectPoints[0], pattern);
                objectPoints.resize(imagePoints.size(),objectPoints[0]);

                /*std::cout << " image Points" <<std::endl;
                for(int i = 0; i < imagePoints.size(); i++){

                    std::cout <<"xy: " <<imagePoints[i] << std::endl;  
                }

                std::cout << " objectPoints" <<std::endl;
                for(int i = 0; i < objectPoints.size(); i++){

                    std::cout <<"xy: " <<objectPoints[i] << std::endl;  
                }*/

                //std::cout<<objectPoints.size() <<" - "<<imagePoints.size()<<" - "<<frWidth << " - " <<frHeight <<std::endl;
                rms = calibrateCamera(objectPoints, imagePoints, cv::Size(frWidth,frHeight), cameraMatrix, distCoeffs, rvecs, tvecs);
                
                bool ok = (checkRange(cameraMatrix) && checkRange(distCoeffs));
                if(!ok)
                    std::cout<<"Camera was not calibrated" <<std::endl;
                
                std::cout <<std::endl;
                std::cout << "RMS error reported by calibrateCamera: " << rms << std::endl;                
                std::cout << "Intrinsic camera matrix" << std::endl << cameraMatrix << std::endl;
                std::cout << "Distortion coefficients" << std::endl << distCoeffs << std::endl;
                std::cout << "----------------------------" << std::endl;

                //undistort = true; 
                //test++;
                //if(test == 2)
                break;    
                //else
                //    imagePoints.clear();                                 
            }  
        }
        else
        {
            //std::cout<<" NotFound "<<std::endl;
            previousFound = false;
            frameCountNotFound++;
        }

        if(undistort){
            ///*** Undistort image ***///
            cv::Mat temp1 = frame.clone();
            cv::Mat OptimalMatrix = cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, cv::Size(frWidth,frHeight), 1.0);
            cv::undistort(temp1,frame,cameraMatrix,distCoeffs,OptimalMatrix);     
            break;
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

        contours.clear();
        hierarchy.clear();
        pointbuf.clear();
    }

    ///*** Fronto-Parallel Calibration ***///

    std::vector<std::vector<cv::Point2f>> imagePoints2;    
    std::vector<cv::Point2f> pointbuf2;

    
    
    cv::Mat imgWarp_inv;
    cv::Mat imgWarp;
    std::vector<cv::Point2f> corrected_points;
    std::vector<cv::Point2f> points_buffer2;

    //std::vector<cv::Point2f> corners;  
        
    std::vector<cv::Point2f> fronto_corners = getFrontoParallelCorners(cv::Size(frWidth, frHeight),patternSizes[pattern]);

    cv::Mat temp;    
    bool found2;        
    bool flag = false;
        
    for(int iter = 0; iter < 5; iter++){
        std::vector<cv::Mat> rvecs2;
        std::vector<cv::Mat> tvecs2;
        cv::Mat cameraMatrix2;
        cv::Mat distCoeffs2;

        std::vector<std::vector<cv::Point3f>> objectPoints(1);                
        getControlPointsPositions(patternSizes[pattern], ctrlPointDistances[pattern], objectPoints[0], pattern);
        objectPoints.resize(imagePoints.size(),objectPoints[0]);
    
        std::cout<<"------ Fronto-Parallel Calibration" <<std::endl;
        
        rms = calibrateCamera(objectPoints, imagePoints, cv::Size(frWidth,frHeight), cameraMatrix2, distCoeffs2, rvecs2, tvecs2);
        std::cout << "RMS error reported by calibrateCamera: " << rms << std::endl;   
        std::cout << "Intrinsic camera matrix" << std::endl << cameraMatrix2 << std::endl;
        std::cout << "Distortion coefficients" << std::endl << distCoeffs2 << std::endl;
        std::cout << "----------------------------" << std::endl;

        //***** Iterative Refinement Method
        /// Undistort and Unproject
        /// find grid
        /// ProjectControlPoints
        /// Calibrate camera                    
             
        for(int i=0; i<selectedFrames.size(); i++){
            
            ///*** Undistort image ***///
            cv::Mat frame2 = selectedFrames[i]; 
            temp = frame2.clone();
            cv::Mat OptimalMatrix = cv::getOptimalNewCameraMatrix(cameraMatrix2, distCoeffs2, cv::Size(frWidth,frHeight), 1.0);
            cv::undistort(temp,frame2,cameraMatrix2,distCoeffs2,OptimalMatrix);                    
            cv::undistortPoints(imagePoints[i], pointbuf2, cameraMatrix2, distCoeffs2, cv::noArray(), OptimalMatrix);
            
            std::cout<<"imagePoints - new pointbuffer" <<std::endl;
            for(int l = 0; l < pointbuf2.size(); l++){
                std::cout<<" " <<imagePoints[i][l].x <<", " <<imagePoints[i][l].y <<"  /-/  " <<pointbuf2[l].x <<", " <<pointbuf2[l].y<<std::endl;            
            }

            cv::namedWindow("Video Display 2", cv::WINDOW_NORMAL);
            imshow("Video Display 2", frame2);

            std::cout<<"buf: " <<pointbuf2.size() <<std::endl;
            
            std::vector<cv::Point2f> corners = getPatternCorners(pointbuf2,patternSizes[pattern]);
            
            std::cout<<"corners: " <<corners.size() <<std::endl;
            for(int j = 0; j < corners.size(); j++){
                std::cout<<"x,y " <<corners[j].x <<", " <<corners[j].y <<std::endl;            
            }

            for(int k = 0; k < fronto_corners.size(); k++){
                std::cout<<"x,y " <<fronto_corners[k].x <<", " <<fronto_corners[k].y <<std::endl;            
            }

            cv::Mat H = cv::findHomography(corners,fronto_corners);	        
	        cv::warpPerspective(frame2,imgWarp,H,frame2.size());
	        
            cv::namedWindow("ImgWarp", cv::WINDOW_NORMAL); 
            imshow("ImgWarp", imgWarp);            

            ///*** Find pattern ***///
            pointbuf2.clear();		
            contours.clear();
            hierarchy.clear();
            found2 = false;

            preprocessImage(imgWarp, view, contours, hierarchy);
            identifyRings2(contours, hierarchy, pointbuf2, patternSize, imgWarp);
            std::cout<<"PointBufferSize: "<<pointbuf2.size()<<"  ";
            
            if(flag){
                if (pointbuf2.size() == patternSize)
                {   
                    found2 = findRingsGrid(imgWarp, patternSizes[pattern], pointbuf2, pointbuf2, false);                                   
                }

                if(found2){
                    cv::warpPerspective(imgWarp,imgWarp_inv,H.inv(),imgWarp.size());                
                    cv::perspectiveTransform( pointbuf2, points_buffer2, H.inv() );
                    cv::undistortPoints(points_buffer2,corrected_points,OptimalMatrix,-distCoeffs2,cv::noArray(),cameraMatrix2);
                
                    imagePoints2.push_back(corrected_points);

                    cv::drawChessboardCorners(imgWarp_inv, patternSizes[pattern], corrected_points, found);
                    std::cout<<"corrected_points" <<std::endl;
                    for(int m = 0; m < corrected_points.size(); m++){
                        std::cout<<" " <<corrected_points[m].x <<", " <<corrected_points[m].y <<std::endl;            
                    }
                }
                else{
                    std::cout << "Not found in the parallel frame!\n";
                }

                
                /*std::cout<<"points_buffer2: "<<points_buffer2.size()<<"  ";                
                std::cout<<"corrected_points: "<<corrected_points.size()<<"  ";                */
                    //cv::drawChessboardCorners(imgWarp_inv, patternSizes[pattern], corrected_points, true);
                
                cv::namedWindow("ImgWarp Inv", cv::WINDOW_NORMAL); 
                imshow("ImgWarp Inv", imgWarp_inv);

                 
            }

            if (cv::waitKey() == 27)
                cv::waitKey(100);

                
        }
        imagePoints.clear();
        imagePoints = imagePoints2;
        std::cout<<"imagePoints: "<<imagePoints.size()<<"  ";

        flag = false;
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


std::vector<cv::Point2f> getPatternCorners(std::vector<cv::Point2f>& v, cv::Size size){
    std::vector<cv::Point2f> corners;
    corners.push_back(v[0]);
    corners.push_back(v[size.width - 1]);
    corners.push_back(v[v.size() - size.width]);
    corners.push_back(v[v.size()-1]);
    return corners;
}

std::vector<cv::Point2f> getFrontoParallelCorners(cv::Size imgSize, cv::Size patternSize){
    float tx = 80.0f, ty = 60.0f;
    float dim = 90.0f;
    
    std::vector<cv::Point2f> corners;
    corners.push_back(cv::Point2f(tx,ty + patternSize.height * dim));
    corners.push_back(cv::Point2f(tx + patternSize.width * dim, ty + patternSize.height * dim));
    corners.push_back(cv::Point2f(tx,ty));
    corners.push_back(cv::Point2f(tx + patternSize.width * dim,ty));
    return corners;

}


            /*corners.push_back(pointbuf2[0]);
            corners.push_back(pointbuf2[(patternSizes[pattern].width) - 1]);
            corners.push_back(pointbuf2[pointbuf2.size() - (patternSizes[pattern].width)]);
            corners.push_back(pointbuf2[pointbuf2.size()-1]);*/
            
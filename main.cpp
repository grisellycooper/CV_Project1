#include <iostream>
#include <cstring>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>


//#define video_path "../../Test/videos/PadronAnillos_01.avi"
#define video_path "../../videos/padron2.avi"
#define amountRingsInPattern 20

/* Try to detect circles in a video using HoughCircles function */
int main(int argc, char **argv)
{

    /*std::string filename = "../../../videos/PadronAnillos_01.avi";
    cv::VideoCapture capture(filename);*/

#ifdef video_path
    cv::VideoCapture capture(video_path);
#else
    cv::VideoCapture capture(0);   // --> For video Capture
    capture.set(cv::CAP_PROP_FPS, 60); // ---> Cantidad de FPS caputrados por la camara
    capture.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
#endif

    if (!capture.isOpened())
        throw "Error when reading steam_avi"; 

    /// Auxilaries
    cv::Mat frame, gray, bw, img;
    int frameCount = 0;
    int frameCount18 = 0;
    int frameCount19 = 0;
    int frameCount20 = 0;
    int frameCount21 = 0;
    int frameCount22 = 0;
    int frameCountRingsCompl = 0;    
    float threshold = 0.65f;
    float sumX, sumY,cpX, cpY;
    int radialZone;
    int nearestCount = 0;
    
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    std::vector<cv::RotatedRect> minEllipse;
    std::vector<cv::RotatedRect> minEllipseSelected;
    std::vector<cv::Point2f> centers;
    std::vector<cv::Point2f> tmpCenters;

    //cv::namedWindow("Video Display", cv::WINDOW_NORMAL);

    float w, h, c_x, c_y, child_c_x, child_c_y, distance;
    int child_index;
    int color = 50;

    /// Time algorithm
    clock_t start, end;

    for (;;)
    {
        capture >> frame;
        if (frame.empty())
            break;

        frameCount++;        
        /// Restart variables
        sumX = 0.0;
        sumY = 0.0;

        start = clock();

        // Convert to grayscale and apply Gaussian blur
        // Reduce information and noise
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0, 0);
        /*cv::namedWindow( "Grayscale Gaussian Blur", cv::WINDOW_NORMAL);
        imshow("Grayscale Gaussian Blur", gray);*/

        // Convert image to binary
        //cv::threshold(gray, bw, 100, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        cv::adaptiveThreshold(gray, bw, 200, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 21, 10);
        /*cv::namedWindow( "Binary", cv::WINDOW_NORMAL);
        imshow("Binary", bw);*/


        // Find all the contours in the thresholded image        
        cv::findContours(bw, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE, cv::Point(0, 0));
        //int idx = 0;
        for (int i = 0; i < contours.size(); i++)
        {
            cv::drawContours(bw, contours, static_cast<int>(i), cv::Scalar(0, 0, 255), 2);
        }

        //Find the minimum bounding ellipse
        minEllipse.resize(contours.size());
        minEllipseSelected.resize(contours.size());
        img = cv::Mat::zeros(frame.rows, frame.cols, CV_8UC3);        
        for (int i = 0; i < contours.size(); i++)
        {
            if (contours[i].size() >= 5)
            {
                minEllipse[i] = fitEllipse(contours[i]);
                //ellipse( src, minEllipse[i], Scalar(0,0,255), 1, 8 );
            }
        }

        for (int i = 0; i < contours.size(); i++)
        {            
            child_index = hierarchy[i][2];
            if (child_index != -1 && hierarchy[child_index][2] == -1) //Check out for child but not grandchild
            {
                //Check distances between parent's center and child's center
                //Should be less than threshold
                c_x = minEllipse[i].center.x;
                c_y = minEllipse[i].center.y;
                child_c_x = minEllipse[child_index].center.x;
                child_c_y = minEllipse[child_index].center.y;
                distance = sqrt(pow((c_x - child_c_x), 2) + pow((c_y - child_c_y), 2));
                //std::cout<<"dist: " <<distance <<std::endl;
                if (distance < threshold)
                {
                    minEllipseSelected.push_back(minEllipse[i]);
                    minEllipseSelected.push_back(minEllipse[hierarchy[i][2]]);
                    //ellipse(frame, minEllipse[i], Scalar(0,0,255), 2, 8 );
                    //ellipse(frame, minEllipse[hierarchy[i][2]], Scalar(0,0,255), 1, 8 );
                    centers.push_back(cv::Point2f((child_c_x + c_x) / 2, (child_c_y + c_y) / 2));                                     
                }
            }
        }

        for(int i = 0; i < centers.size(); i++){
            sumX += centers[i].x;
            sumY += centers[i].y; 
        }

        /// Finding an average Central Point         
        cpX = sumX/centers.size();
        cpY = sumY/centers.size();
        circle(frame, cv::Point2f(sumX/centers.size(), sumY/centers.size()), 1, cv::Scalar(255, 0, 0), 4, 8);

        
        for(radialZone = 1; radialZone < 200; radialZone++){
            
            for(int i = 0; i < centers.size(); i++){
                if(sqrt(pow((cpX - centers[i].x), 2) + pow((cpY - centers[i].y), 2)) < radialZone)
                    nearestCount++;
            }
            if(nearestCount >= amountRingsInPattern){                
                break;
            }   
        }   
        //std::cout<<"Radial zone: "<<radialZone<<std::endl;     
        /// Display circles in the radial zone     
        for(int i = 0; i < centers.size(); i++){
            if(sqrt(pow((cpX - centers[i].x), 2) + pow((cpY - centers[i].y), 2)) < radialZone + 20){
                //tmpCenters.push_back(centers[i]);
                circle(frame, centers[i], 1, cv::Scalar(0, 0, 255), 4, 8);    
            }
                
        }
        
        /*if(centers.size() > amountRingsInPattern)
            std::cout << "I: "<<centers.size() <<" F:"<< tmpCenters.size() <<std::endl;*/

        /*centers.clear();
        centers = tmpCenters;

        for(int i = 0; i < centers.size();i++){
            //circle(frame,centers[i],5,cv::Scalar(255,0,0),3,8);
            circle(frame, centers[i], 1, cv::Scalar(0, 0, 255), 4, 8);
            //cv::putText(frame, std::to_string(i), centers[i],cv::FONT_HERSHEY_DUPLEX,0.5, cv::Scalar(250, 0, 0),2);
        }*/

        //std::cout<<"Frame: "<<frameCount <<" CP: "<<centers.size()<<std::endl;

        /*if(centers.size() == 18)
            frameCount18++;

        if(centers.size() == 19)
            frameCount19++;

        if(centers.size() == 20)
            frameCount20++;
        
        if(centers.size() == 21)
            frameCount21++;

        if(centers.size() == 22)
            frameCount22++;*/
        
        /*cv::namedWindow("Video Display", cv::WINDOW_NORMAL);
        imshow("Video Display", frame);
        cv::waitKey(20);*/
                
        if (frameCount % 20 == 0)
        {
            end = clock();
            std::cout<<"Frame: "<<frameCount <<" Time: " << (end - start)/(double)( CLOCKS_PER_SEC / 1000 ) << " ms." <<std::endl;
        }

        contours.clear();
        hierarchy.clear();
        minEllipse.clear();   
        minEllipseSelected.clear();
        centers.clear();
        tmpCenters.clear();
    }
    
    /*std::cout<<"Complete rings were detected in "<<frameCount20 <<" out of " <<frameCount<< " frames"<<std::endl;
    std::cout<<"18 "<<frameCount18 <<std::endl;
    std::cout<<"19 "<<frameCount19 <<std::endl;
    std::cout<<"20 "<<frameCount20 <<std::endl;
    std::cout<<"21 "<<frameCount21 <<std::endl;
    std::cout<<"22 "<<frameCount22 <<std::endl;*/
    //cv::waitKey(0); // key press to close window
    // releases and window destroy are automatic in C++ interface

}


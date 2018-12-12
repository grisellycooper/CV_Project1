#include <iostream>
#include <cstring>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define video_path "../../Test/videos/PadronAnillos_01.avi"

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

    std::cout<<"0 ";    

    /// Auxilaries
    cv::Mat frame, gray, bw, img;
    int frameCount = 0;
    //int frameCountRingsCompl = 0;
    //int ellipseCount = 0;
    float threshold = 0.4f;

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    std::vector<cv::RotatedRect> minEllipse;
    std::vector<cv::RotatedRect> minEllipseSelected;
    std::vector<cv::Point2f> centers;

    //cv::namedWindow("Video Display", cv::WINDOW_NORMAL);

    /// To try a couple of different algorithms
    /*int icase = 1;
    if( argc > 1)
    {
        icase = atoi(argv[1]);            
    } */
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
        //ellipseCount = 0;
        
        start = clock();

        // Convert to grayscale and apply Gaussian blur
        // Reduce information and noise
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0, 0);
        //cv::namedWindow( "Grayscale Gaussian Blur", cv::WINDOW_NORMAL);
        //imshow("Grayscale Gaussian Blur", gray);

        // Convert image to binary
        cv::threshold(gray, bw, 100, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

        // Find all the contours in the thresholded image
        
        cv::findContours(bw, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE, cv::Point(0, 0));
        //int idx = 0;
        /*for (int i = 0; i < contours.size(); i++)
        {
            cv::drawContours(bw, contours, static_cast<int>(i), cv::Scalar(0, 0, 255), 2);
        }*/

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
            //cout<<i<<": ";
            /*for (int j = 0; j< 4; j++ ){  
                        cout<<"  "<<hierarchy[i][j]; 
                    } */
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
                    //ellipseCount++;
                }
            }
        }

        //std::cout << "Frame: " << frameCount << " Ellipses found: " << ellipseCount << std::endl;

        for (int i = 0; i < centers.size(); i++)
        {
            circle(frame, centers[i], 1, cv::Scalar(0, 0, 255), 4, 8);
            cv::putText(frame, std::to_string(i), centers[i],cv::FONT_HERSHEY_DUPLEX,0.5, cv::Scalar(255, 0, 0),2);
        }

        /*cv::namedWindow("Video Display", cv::WINDOW_NORMAL);
        imshow("Video Display", frame);
        cv::waitKey(40);*/

        /*if(centers.size() >= 30)
            frameCountRingsCompl++;
        */
        if (frameCount % 10 == 0)
        {
            end = clock();
            std::cout << "Time: " << (end - start)/(double)( CLOCKS_PER_SEC / 1000 ) << " milliseconds." << std::endl;
        }

        /*if(centers.size() == 30){
            // Draw center points
            for(int i = 0; i < centers.size();i++)
                circle(frame, centers[i],1,Scalar(0,0,255),4,8);

            namedWindow("Video Display", WINDOW_NORMAL);
            imshow("Video Display", frame);
            waitKey(20); // waits to display frame            
        } */

        contours.clear();
        hierarchy.clear();
        minEllipse.clear();   
        minEllipseSelected.clear();
        centers.clear();
    }
    //std::cout<<"Complete rings were detected in "<<frameCountRingsCompl << " frames"<<std::endl;
    //cv::waitKey(0); // key press to close window
    // releases and window destroy are automatic in C++ interface

}

/*switch(icase)
        {
            case 0:
                std::cout<<"0"<<std::endl;
                float diff; //Difference between width and high ellipse
        
                for(size_t i = 0; i< contours.size(); i++ ){
                    w = minEllipse[i].size.width;
                    h = minEllipse[i].size.height;
                    diff = w - h;                        
                
                    if(abs(diff) < 40){
                        child_index = hierarchy[i][2];    
                        if(child_index != -1 && hierarchy[child_index][2] == -1) //Check out for child but not grandchild
                        {
                            minEllipseSelected.push_back(minEllipse[i]);            
                            minEllipseSelected.push_back(minEllipse[hierarchy[i][2]]);                            
                            ellipse(img, minEllipse[i], cv::Scalar(0,0,255), 1, 8 );            
                            ellipse(img, minEllipse[hierarchy[i][2]], cv::Scalar(0,0,255), 1, 8 );            
                        }
                    }                                                                        
                }        
        
                break;

        */
#include <iostream>
#include <cstring>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

/* Try to detect circles in a video using HoughCircles function */
int main(int argc, char** argv){

    std::string filename = "../../../videos/PadronAnillos_01.avi";
    VideoCapture capture(filename);
    Mat frame;

    if( !capture.isOpened() )
        throw "Error when reading steam_avi";

    namedWindow("Video Display", WINDOW_NORMAL);

    for( ; ; )
    {
        capture >> frame;
        if(frame.empty())
            break;

        //Mat src = imread(filename, IMREAD_COLOR);
        /*if( src.empty() )
        {
            cout << "Couldn't load " << filename << endl;
                //continue;
        }*/

        // Convert to grayscale and apply Gaussian blur
        Mat gray;
        cvtColor(frame, gray, COLOR_BGR2GRAY);
        GaussianBlur(gray, gray, Size(5,5),0, 0);
        namedWindow( "Grayscale Gaussian Blur", WINDOW_NORMAL );
        imshow("Grayscale Gaussian Blur", gray);
        
        // Convert image to binary
        Mat bw;
        threshold(gray, bw, 100, 255, THRESH_BINARY | THRESH_OTSU);
            
        // Find all the contours in the thresholded image
        vector<vector<Point> > contours;    
        vector<Vec4i> hierarchy;
        findContours(bw, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE, Point(0,0));
        //int idx = 0;
        for (size_t i = 0; i < contours.size(); i++)
        {        
            drawContours(bw, contours, static_cast<int>(i), Scalar(0, 0, 255), 2);
        }
            
        //Find minimum bounding rectangle &  minimum bounding ellipse    
        vector<RotatedRect>minEllipse(contours.size());
        vector<RotatedRect>minEllipseSelected(contours.size());
        
        Mat img = Mat::zeros(frame.rows,frame.cols,CV_8UC3);
        vector<Point2f> centers;
        RotatedRect box;
        for (size_t i = 0; i < contours.size(); i++)    
        {
            if(contours[i].size() >= 5){
                minEllipse[i] = fitEllipse(contours[i]);
                //ellipse( src, minEllipse[i], Scalar(0,0,255), 1, 8 ); 
            }             
        }

        int icase = 1;
        if( argc > 1)
        {
            icase = atoi(argv[1]);            
        }       
        float w, h, c_x, c_y, child_c_x, child_c_y, distance;
        int child_index;
        switch(icase)
        {
            case 0:
                cout<<"0"<<endl;
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
                            ellipse(img, minEllipse[i], Scalar(0,0,255), 1, 8 );            
                            ellipse(img, minEllipse[hierarchy[i][2]], Scalar(0,0,255), 1, 8 );            
                        }
                    }                                                                        
                }        
        
                break;

            case 1:
                float threshold = 0.4f;
                for(size_t i = 0; i< contours.size(); i++ ){
                    cout<<i<<": ";                            
                    for (size_t j = 0; j< 4; j++ ){  
                        cout<<"  "<<hierarchy[i][j]; 
                    }            
                    child_index = hierarchy[i][2];    
                    if(child_index != -1 && hierarchy[child_index][2] == -1) //Check out for child but not grandchild
                    {
                        //Check distances between parent's center and child's center
                        //Should be less than threshold                                       
                        c_x = minEllipse[i].center.x;
                        c_y = minEllipse[i].center.y;
                        child_c_x = minEllipse[child_index].center.x;
                        child_c_y = minEllipse[child_index].center.y;
                        distance = sqrt(pow((c_x-child_c_x),2) + pow((c_y-child_c_y),2));
                        cout<<"dist: " <<distance <<endl;
                        if(distance < threshold){
                            minEllipseSelected.push_back(minEllipse[i]);            
                            minEllipseSelected.push_back(minEllipse[hierarchy[i][2]]);                            
                            //ellipse(frame, minEllipse[i], Scalar(0,0,255), 2, 8 );            
                            //ellipse(frame, minEllipse[hierarchy[i][2]], Scalar(0,0,255), 1, 8 );  
                            centers.push_back(Point2f(child_c_x,child_c_y));          
                        }                                                        
                    }
                    cout<<endl;
                }
                
                break;        
        }
        // The size grid must be "parametrizado"
        if(centers.size() == 30){
            // Draw center points
            for(int i = 0; i < centers.size();i++)
                circle(frame, centers[i],1,Scalar(0,0,255),4,8);

            namedWindow("Video Display", WINDOW_NORMAL);
            imshow("Video Display", frame);
            waitKey(20); // waits to display frame            
        }        
    }
    waitKey(0); // key press to close window
    // releases and window destroy are automatic in C++ interface
}
#include <iostream>
#include <cstring>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// richard

//#define video_path "../../Test/videos/PadronAnillos_01.avi"
//#define video_path "../../videos/padron1.avi"  // 12 Anillos
//#define video_path "../../videos/padron2.avi"  // 20 Anillos
//#define video_path "../../videos/PadronAnillos_01.avi"
#define patternWidth 5
#define patternHeigh 4

/* Try to detect circles in a video using HoughCircles function */
int main(int argc, char **argv)
{
    int patterSize = patternHeigh * patternWidth;
    /*std::string filename = "../../../videos/PadronAnillos_01.avi";
    cv::VideoCapture capture(filename);*/

/*#ifdef video_path
    cv::VideoCapture capture(video_path);
#else
    cv::VideoCapture capture(0);   // --> For video Capture
    capture.set(cv::CAP_PROP_FPS, 60); // ---> Cantidad de FPS caputrados por la camara
    capture.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
#endif*/

    /*if (!capture.isOpened())
        throw "Error when reading steam_avi"; */
    
    //cv::Mat frame;
    /// Auxilaries
    cv::Mat gray, bw, cont, img;
    int frameCount = 0;
    int frameCount18 = 0;
    int frameCount19 = 0;
    int frameCount20 = 0;
    int frameCount21 = 0;
    int frameCount22 = 0;
    int frameCountRingsCompl = 0;    
    float threshold = 0.75f;
    float sumX, sumY,cpX, cpY;
    int radialZone;
    float sumTime = 0.0f;
    
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    std::vector<cv::RotatedRect> minEllipse;
    std::vector<cv::RotatedRect> minEllipseSelected;
    std::vector<cv::Point2f> centers;
    std::vector<cv::Point2f> tmpCenters;
    std::vector<cv::Point2f> corners;

    //cv::namedWindow("Video Display", cv::WINDOW_NORMAL);

    float w, h, c_x, c_y, child_c_x, child_c_y, distance;
    int child_index;
    int color = 50;

    /// Time algorithm
    clock_t start, end;

    for( int o = 1; o< 6; o++ )
    {
        //string filename = samples::findFile(names[i]);
        std::string filename = "../../img/" + std::to_string(o) + ".png";
        cv::Mat frame = cv::imread(filename, cv::IMREAD_COLOR);
    //for (;;)
    //{
        //capture >> frame;
        //if (frame.empty())
            //break;

        //frameCount++;        
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

        cont = frame.clone();
        // Find all the contours in the thresholded image        
        cv::findContours(bw, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE, cv::Point(0, 0));
        //int idx = 0;
        /*for (int i = 0; i < contours.size(); i++)
        {
            cv::drawContours(cont, contours, static_cast<int>(i), cv::Scalar(0, 0, 255), 2);
        }*/
        /*cv::namedWindow( "Contour", cv::WINDOW_NORMAL);
        imshow("Contour", cont);*/

        //Find the minimum bounding ellipse
        minEllipse.resize(contours.size());
        minEllipseSelected.resize(contours.size());
        //img = cv::Mat::zeros(frame.rows, frame.cols, CV_8UC3);        
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
                /*c_x = minEllipse[i].center.x;
                c_y = minEllipse[i].center.y;
                child_c_x = minEllipse[child_index].center.x;
                child_c_y = minEllipse[child_index].center.y;*/
                distance = cv::norm(minEllipse[i].center - minEllipse[child_index].center); 
                //distance = sqrt(pow((c_x - child_c_x), 2) + pow((c_y - child_c_y), 2));
                //std::cout<<"dist: " <<distance <<std::endl;
                if (distance < threshold)
                {
                    minEllipseSelected.push_back(minEllipse[i]);
                    minEllipseSelected.push_back(minEllipse[hierarchy[i][2]]);
                    //ellipse(cont, minEllipse[i], cv::Scalar(0,0,255), 2, 8 );
                    //ellipse(cont, minEllipse[hierarchy[i][2]], cv::Scalar(0,0,255), 1, 8 );
                    centers.push_back(cv::Point2f((minEllipse[i].center.x + minEllipse[child_index].center.x) / 2, (minEllipse[i].center.y + minEllipse[child_index].center.y) / 2));                                     
                }
            }
        }
        /*cv::namedWindow( "Ellipse Fitting", cv::WINDOW_NORMAL);
        imshow("Ellipse Fitting", cont);*/

        for(int i = 0; i < centers.size(); i++){
            sumX += centers[i].x;
            sumY += centers[i].y; 
        }

        /// Finding an average Central Point         
        cv::Point2f avg(sumX/centers.size(), sumY/centers.size());
        /*cpX = sumX/centers.size();
        cpY = sumY/centers.size();*/
        circle(frame, avg, 1, cv::Scalar(255, 0, 0), 4, 8);
        
        for(radialZone = 1; radialZone < 200; radialZone++){
            int count = 0;
            for(int i = 0; i < centers.size(); i++){
                if(cv::norm(avg - centers[i]) < radialZone)
                    count++;
            }
            if(count >= patterSize){
                break;
            }   
        }
        
        /// Display circles in the radial zone     
        for(int i = 0; i < centers.size(); i++){
            if(cv::norm(avg - centers[i]) < radialZone + 20)
                tmpCenters.push_back(centers[i]);
        }

        /*if(centers.size() > amountRingsInPattern)
            std::cout << "I: "<<centers.size() <<" F:"<< tmpCenters.size() <<std::endl;*/

        centers.clear();
        centers = tmpCenters;

        for(int i = 0; i < centers.size();i++){
            //circle(frame,centers[i],5,cv::Scalar(255,0,0),3,8);
            circle(frame, centers[i], 1, cv::Scalar(0, 0, 255), 4, 8);
            //cv::putText(frame, std::to_string(i), centers[i],cv::FONT_HERSHEY_DUPLEX,0.5, cv::Scalar(250, 0, 0),2);
        }

        tmpCenters.clear();
        tmpCenters.resize(patterSize);

        int i, j;
        float distMax = 0.0, distTmp = 0.0;
        int ii, jj;

        /*std::cout<<"Print points before... "<<centers.size()<<std::endl; 
        std::cout<<"Print points before... "<<tmpCenters.size()<<std::endl;
        for(int x=0; x<centers.size();x++){
            std::cout<<"("<<centers[x].x<<", "<<centers[x].y<<")"<<std::endl;
        }*/
        
        /// Compute Distance between all centers and get couple of points that are further away feo
        for(i = 0; i < centers.size(); i++){
            for(j = 0; j < centers.size(); j++){
                if(i == j)
                    continue;
                distTmp = cv::norm(centers[j] - centers[i]);
                if(distTmp > distMax){
                    distMax = distTmp;
                    ii = i;
                    jj = j;
                }                           
            }   
        }
        
        /*std::cout<<"i: ("<<centers[ii].x<<", "<<centers[ii].y<<")"<<std::endl;
        std::cout<<"j: ("<<centers[jj].x<<", "<<centers[jj].y<<")"<<std::endl;*/
        
        /// Add to corners vector
        corners.push_back(centers[ii]);
        corners.push_back(centers[jj]);
        
        /// Erase them from list
        centers.erase(centers.begin() + ii);
        centers.erase(centers.begin() + jj);

        
        /*std::cout<<"Print points after... "<<centers.size()<<std::endl; 
        std::cout<<"Print points after... "<<tmpCenters.size()<<std::endl; 
        for(int x=0; x<centers.size();x++){
            std::cout<<"("<<centers[x].x<<", "<<centers[x].y<<")"<<std::endl;
        }*/

        distMax = 0.0;
        distTmp = 0.0;        

        for(i = 0; i < centers.size(); i++){
            for(j = 0; j < centers.size(); j++){
                if(i == j)
                    continue;
                distTmp = cv::norm(centers[j] - centers[i]);
                if(distTmp > distMax){
                    distMax = distTmp;
                    ii = i;
                    jj = j;
                }                    
            }   
        }

        /// Add to corners vector
        corners.push_back(centers[ii]);
        corners.push_back(centers[jj]);
        
        /// Erase
        centers.erase(centers.begin() + ii);
        centers.erase(centers.begin() + jj);
        
        
        std::cout<<"Corners "<<corners.size()<<std::endl;
        for(int i = 0; i < corners.size(); i++){
            std::cout<<"("<<corners[i].x<<", "<<corners[i].y<<")"<<std::endl;
        }

        std::sort(corners.begin(), corners.end(), [](cv::Point2f const& f, cv::Point2f const& s){ return f.x < s.x; });
        std::sort(centers.begin(), centers.end(), [](cv::Point2f const& f, cv::Point2f const& s){ return f.x < s.x; });

        std::cout<<"X "<<corners.size()<<std::endl;
        for(int i = 0; i < corners.size(); i++){
            std::cout<<"("<<corners[i].x<<", "<<corners[i].y<<")"<<std::endl;
        }


        if(corners[0].y > corners[1].y){            
            tmpCenters[patterSize-patternWidth] = corners[0];
            tmpCenters[0] = corners[1];
            std::swap(corners[0], corners[1]);
        }
        else{
            tmpCenters[0] = corners[0];
            tmpCenters[patterSize-patternWidth] = corners[1];            
        }

        if(corners[2].y > corners[3].y){
            tmpCenters[patterSize-1] = corners[2];
            tmpCenters[patternWidth-1] = corners[3];            
        }
        else{            
            tmpCenters[patternWidth-1] = corners[2];
            tmpCenters[patterSize-1] = corners[3];
            std::swap(corners[2], corners[3]);
        }

        /*std::cout<<"--- "<<corners.size()<<std::endl;
        for(int i = 0; i < corners.size(); i++){
            std::cout<<"("<<corners[i].x<<", "<<corners[i].y<<")"<<std::endl;
        }*/
            
        circle(frame, tmpCenters[0], 1, cv::Scalar(0, 255, 0), 4, 8);
        circle(frame, tmpCenters[patternWidth-1], 1, cv::Scalar(0, 255, 0), 4, 8);
        circle(frame, tmpCenters[patterSize-patternWidth], 1, cv::Scalar(0, 255, 0), 4, 8);
        circle(frame, tmpCenters[patterSize -1], 1, cv::Scalar(0, 255, 0), 4, 8);

        cv::putText(frame, std::to_string(0), tmpCenters[0],cv::FONT_HERSHEY_DUPLEX,0.5, cv::Scalar(250, 0, 0),2);
        cv::putText(frame, std::to_string(1), tmpCenters[patternWidth-1],cv::FONT_HERSHEY_DUPLEX,0.5, cv::Scalar(250, 0, 0),2);
        cv::putText(frame, std::to_string(2), tmpCenters[patterSize-patternWidth],cv::FONT_HERSHEY_DUPLEX,0.5, cv::Scalar(250, 0, 0),2);
        cv::putText(frame, std::to_string(3), tmpCenters[patterSize -1],cv::FONT_HERSHEY_DUPLEX,0.5, cv::Scalar(250, 0, 0),2);


        ///***** get slope ****///
        /*double slope, length;
        slope  = (tmpCenters[0].y - tmpCenters[patterSize-patternWidth].y) / (tmpCenters[0].x - tmpCenters[patterSize-patternWidth].x)
        length = norm(tmpCenters[0] - tmpCenters[0])*/

        /*circle(frame,cv::Point2f(50,50), 1, cv::Scalar(0, 255, 255), 4, 8);
        circle(frame,cv::Point2f(250,250), 1, cv::Scalar(0, 255, 255), 4, 8);*/
        
        /// Draw a rectagle
        /*cv::Rect rect = cv::boundingRect(centers);
        cv::rectangle(frame, rect, cv::Scalar(0, 255, 0));
        std::cout<<rect.height << " " <<rect.width <<std::endl;*/



        //std::cout<<"Frame: "<<frameCount <<" CP: "<<centers.size()<<std::endl;

        /*if(centers.size() == 10)
            frameCount18++;

        if(centers.size() == 11)
            frameCount19++;

        if(centers.size() == 12)
            frameCount20++;
        
        if(centers.size() == 13)
            frameCount21++;

        if(centers.size() == 14)
            frameCount22++;*/
        
        cv::namedWindow("Video Display", cv::WINDOW_NORMAL);
        imshow("Video Display", frame);
        //cv::waitKey();

        end = clock();
        sumTime += (end - start)/(double)( CLOCKS_PER_SEC / 1000 );

        /*if (frameCount % 20 == 0)
        {
            sumTime = sumTime/20.0;
            std::cout << "Frame: " << frameCount << " Time: " << (end - start)/(double)( CLOCKS_PER_SEC / 1000 ) << " ms." <<std::endl;
            //std::cout << sumTime <<std::endl;
            sumTime = 0.0;
        }*/

        contours.clear();
        hierarchy.clear();
        minEllipse.clear();   
        minEllipseSelected.clear();
        centers.clear();
        tmpCenters.clear();
        corners.clear();
    
        int c = cv::waitKey();
        if( c == 27 )
            break;
    }
    /*std::cout<<"Complete rings were detected in "<<frameCount20 <<" out of " <<frameCount<< " frames"<<std::endl;
    std::cout<<"10 "<<frameCount18 <<std::endl;
    std::cout<<"11 "<<frameCount19 <<std::endl;
    std::cout<<"12 "<<frameCount20 <<std::endl;
    std::cout<<"13 "<<frameCount21 <<std::endl;
    std::cout<<"14 "<<frameCount22 <<std::endl;*/
    //cv::waitKey(0); // key press to close window
    // releases and window destroy are automatic in C++ interface

}


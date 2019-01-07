#include <iostream>
#include <cstring>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//#define video_path "../../Test/videos/PadronAnillos_01.avi"
#define video_path "../../../videos/padron1.avi"  // 12 Anillos
//#define video_path "../../../videos/padron2.avi"  // 20 Anillos
//#define video_path "../../../videos/PadronAnillos_01.avi"  // 30 Anillos

#define patternWidth 4
#define patternHeigh 3

#define thresholdDist 35

#define display 1
#define displayTest 1
#define displayCompleteProcess 0
#define displayContinuosly 0
#define printTime 0
#define printFrameCount 1

bool verifyOrder(std::vector<cv::Point2f> &tmpCenters, int heigh, int width);
std::vector< std::vector<int>> GenerateCombinations(int n, int r);
void printCombination(std::vector< std::vector<int> >& v, int arr[], int n, int r);
void combinationUtil(std::vector< std::vector<int> >& v, int arr[], std::vector<int> &data, int start, int end, int index, int r);
float StandarDesviation(const std::vector<float> & values);
bool cmpx(cv::Point2f a, cv::Point2f b);

void findCorners(std::vector<cv::Point2f> &centers, std::vector<cv::Point2f> &corners); //Version1
void getProperOrder(std::vector<cv::Point2f> &centers, std::vector<cv::Point2f> &corners); //Version1

int main(int argc, char **argv)
{
    int tracking = 0;
    int patternSize = patternHeigh * patternWidth;
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
    cv::Mat frame;    
    cv::Mat gray, bw, cont, test1, test2;
    int frameCount = 0;
    int frameCountCorrect = 0;
    int frameCountIncorrect = 0;
    int frameCountLess = 0;
    int frameCountMore = 0;
    float threshold = 0.75f;        /// Between parent and child ellipse
    float sumX, sumY,cpX, cpY;
    int radialZone;
    float sumTime = 0.0f;
    
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    std::vector<cv::RotatedRect> minEllipse;
    std::vector<cv::RotatedRect> minEllipseSelected;
    std::vector<cv::Point2f> centers;
    std::vector<cv::Point2f> tmpCenters;
    std::vector<cv::Point2f> trackedPoints;
    std::vector<cv::Point2f> oldTrackedPoints;
    std::vector<cv::Point2f> corners;

    //cv::namedWindow("Video Display", cv::WINDOW_NORMAL);

    float w, h, c_x, c_y, child_c_x, child_c_y, distance;
    int child_index;
    
    float dist;    
    bool orderFound;    

    /// Time algorithm
    clock_t start, end;

    /*for( int o = 1; o< 6; o++ )
    {
        //string filename = samples::findFile(names[i]);
        std::string filename = "../../img/" + std::to_string(o) + ".png";
        cv::Mat frame = cv::imread(filename, cv::IMREAD_COLOR);
    */
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

        ///**** PREPROCESSING IMAGE ****///

        // Convert to grayscale and apply Gaussian blur
        // Reduce information and noise
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0, 0);
        
        if(displayCompleteProcess == 1){
            cv::namedWindow( "Grayscale Gaussian Blur", cv::WINDOW_NORMAL);
            imshow("Grayscale Gaussian Blur", gray);
        }
        
        // Convert image to binary
        //cv::threshold(gray, bw, 100, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        cv::adaptiveThreshold(gray, bw, 200, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 21, 10);
        if(displayCompleteProcess == 1){
            cv::namedWindow( "Binary", cv::WINDOW_NORMAL);
            imshow("Binary", bw);
        }

        cont = frame.clone();
        // Find all the contours in the thresholded image        
        cv::findContours(bw, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE, cv::Point(0, 0));
        //int idx = 0;
        /*for (int i = 0; i < contours.size(); i++)
        {
            cv::drawContours(cont, contours, static_cast<int>(i), cv::Scalar(0, 0, 255), 2);
        }
        cv::namedWindow( "Contour", cv::WINDOW_NORMAL);
        imshow("Contour", cont);*/

        ///*** IDENTIFY CIRCLES ***///
        
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
                distance = cv::norm(minEllipse[i].center - minEllipse[child_index].center); 
                //std::cout<<"dist: " <<distance <<std::endl;
                if (distance < threshold)
                {
                    minEllipseSelected.push_back(minEllipse[i]);
                    minEllipseSelected.push_back(minEllipse[hierarchy[i][2]]);
                    if(displayCompleteProcess == 1){        
                        ellipse(cont, minEllipse[i], cv::Scalar(0,0,255), 2, 8 );
                        ellipse(cont, minEllipse[hierarchy[i][2]], cv::Scalar(0,0,255), 1, 8 );
                    }
                    centers.push_back(cv::Point2f((minEllipse[i].center.x + minEllipse[child_index].center.x) / 2, (minEllipse[i].center.y + minEllipse[child_index].center.y) / 2));                                     
                }
            }
        }

        for(int i = 0; i < centers.size(); i++){
            sumX += centers[i].x;
            sumY += centers[i].y; 
        }

        /// Finding an average Central Point         
        cv::Point2f avg(sumX/centers.size(), sumY/centers.size());
        circle(frame, avg, 1, cv::Scalar(255, 0, 0), 4, 8);
        
        for(radialZone = 1; radialZone < 200; radialZone++){
            int count = 0;
            for(int i = 0; i < centers.size(); i++){
                if(cv::norm(avg - centers[i]) < radialZone)
                    count++;
            }
            if(count >= patternSize){
                break;
            }   
        }

        /// Add circles in the radial zone         
        for(int i = 0; i < centers.size(); i++){
            if(cv::norm(avg - centers[i]) < radialZone + 20)
                tmpCenters.push_back(centers[i]);
        }

        test1 = cv::Mat::zeros(frame.rows, frame.cols, CV_8UC3); 
        for(int i = 0; i < tmpCenters.size();i++){            
            circle(test1, tmpCenters[i], 1, cv::Scalar(0, 0, 255), 4, 8);
            cv::putText(test1, std::to_string(i), tmpCenters[i],cv::FONT_HERSHEY_DUPLEX,0.5, cv::Scalar(250, 0, 0),2);
        }
    
        ///*** PUTTING ELEMENTS IN ORDER ***///
        test2 = cv::Mat::zeros(frame.rows, frame.cols, CV_8UC3); 
        int c = 0;
        if(tmpCenters.size() == patternSize)
        { 
            if(tracking == 1)
            {                            
                trackedPoints.resize(patternSize);
                std::vector<float> distances;
                bool isCorrect = true;
                for(int k = 0; k < patternSize;k++){
                    cv::Point2f tmp = oldTrackedPoints[k]; 
                    float min = 100000.0f;
                    int index = 0;
                    for(int i = 0; i < tmpCenters.size(); i++){
                        dist = cv::norm(oldTrackedPoints[k] - tmpCenters[i]);
                        if( min > dist){
                            min = dist;
                            index = i;
                        }
                    }
                    distances.push_back(cv::norm(oldTrackedPoints[k]-tmpCenters[index]));
                    if(min < 3.0 ){
                        c++;  
                        trackedPoints[k] = tmpCenters[index]; // Actualizamos la posicion de los puntos                        
                    }
                    else{
                        //std::cout<<"mindist: " << min <<std::endl;
                    }
                }
                                
                if(c == patternSize){
                    for(int i = 0; i < patternSize; i++){
                        circle(test2, trackedPoints[i], 1, cv::Scalar(0, 0, 255), 4, 8);
                        cv::putText(test2, std::to_string(i), trackedPoints[i],cv::FONT_HERSHEY_DUPLEX,0.5, cv::Scalar(250, 0, 0),2);
                    }
                    oldTrackedPoints = trackedPoints;
                    cv::drawChessboardCorners(frame, cv::Size(patternWidth,patternHeigh), trackedPoints,orderFound);
                }
                else{
                    //std::cout <<frameCount << ": Couldnt keep tracking\n";
                    tracking = 0;
                    frameCountIncorrect++;
                }

            }
            else{
                    
                orderFound = verifyOrder(tmpCenters, patternHeigh, patternWidth);
                
                if(orderFound){
                    centers.clear();                    
                    for(int i = 0; i < patternSize; i++){
                        centers.push_back(tmpCenters[i]);
                        circle(test2, tmpCenters[i], 1, cv::Scalar(0, 0, 255), 4, 8);
                        cv::putText(test2, std::to_string(i), tmpCenters[i],cv::FONT_HERSHEY_DUPLEX,0.5, cv::Scalar(250, 0, 0),2);
                    }
                    
                    //frameCountCorrect++;
                    tracking = 1;
                    oldTrackedPoints = centers;
                    cv::drawChessboardCorners(frame, cv::Size(patternWidth,patternHeigh), centers,orderFound);

                }   
            }
        }
        else{
            if(tmpCenters.size() > patternSize) frameCountMore++;
            if(tmpCenters.size() < patternSize) frameCountLess++;
            tracking = 0;
        }        

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

        if(displayCompleteProcess == 1){
            cv::namedWindow( "Ellipse Fitting", cv::WINDOW_NORMAL);
            imshow("Ellipse Fitting", cont);
        }

        if(displayTest == 1)
        {
            cv::namedWindow("1", cv::WINDOW_NORMAL);
            imshow("1", test1);
            cv::namedWindow("2", cv::WINDOW_NORMAL);
            imshow("2", test2);
            if(displayContinuosly == 1)
                cv::waitKey(20);
            else
            {
                if( cv::waitKey() == 27 )
                    cv::waitKey(100);        
            }
        }
                
        if(printTime == 1){
            end = clock();
            sumTime += (end - start)/(double)( CLOCKS_PER_SEC / 1000 );

            if (frameCount % 20 == 0)
            {
                sumTime = sumTime/20.0;
                //std::cout << "Frame: " << frameCount << " Time: " << (end - start)/(double)( CLOCKS_PER_SEC / 1000 ) << " ms." <<std::endl;
                std::cout << (end - start)/(double)( CLOCKS_PER_SEC / 1000 ) <<std::endl;
                //std::cout << sumTime <<std::endl;
                sumTime = 0.0;
            }
        }

        contours.clear();
        hierarchy.clear();
        minEllipse.clear();   
        minEllipseSelected.clear();
        centers.clear();
        tmpCenters.clear();
        corners.clear();
    }

    if(printFrameCount == 1){
        //std::cout<<"Complete rings were detected in "<<frameCountCorrect <<" out of " <<frameCount<< " frames"<<std::endl;
        //std::cout<<"Less than pattern size "<<frameCountLess <<std::endl;
        //std::cout<<"More than pattern size "<<frameCountMore <<std::endl;    
        std::cout<<"Incorrect "<<frameCountIncorrect <<" out of " <<frameCount<< " frames"<<std::endl;
    }
}

void findCorners(std::vector<cv::Point2f> &centers, std::vector<cv::Point2f> &corners)
{
    int i, j;
    float distMax = 0.0, distTmp = 0.0;
    int ii, jj;

    float Ax, Ay, Bx, By, Cx, Cy;
    float A,B,C,side;
    float areaMax = 0.0, areaTmp = 0.0;
    
    std::vector<cv::Point2f> tmpCorners;

    float dist1 = 0.0, dist2 = 0.0;

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

    /// Get Two points from Triangle
    Ax = corners[0].x;
    Ay = corners[0].y;

    Bx = corners[1].x;
    By = corners[1].y;
        
    for(i=0;i<centers.size();i++)
    {
        Cx  = centers[i].x;
        Cy  = centers[i].y;
        A   = sqrt(((Bx-Ax)*(Bx-Ax))+((By-Ay)*(By-Ay)));
        B   = sqrt(((Cx-Bx)*(Cx-Bx))+((Cy-By)*(Cy-By)));
        C   = sqrt(((Ax-Cx)*(Ax-Cx))+((Ay-Cy)*(Ay-Cy)));
        side= ((A+B+C)/2);
        areaTmp=sqrt(side*(side-A)*(side-B)*(side-C));

        if(areaTmp>areaMax)
        {
            areaMax = areaTmp;            
            ii = i; 
        }
    }
    
    cv::Point2f np(centers[ii]);
    
    /// Add new corner
    corners.push_back(np);
    
    /// Erase from centers
    centers.erase(centers.begin() + ii);
    
    distTmp = 0.0;
    distMax = 0.0;

    for(int i=0;i<centers.size();i++)
    {
        distTmp = cv::norm(centers[i]-np);
        if(distTmp > distMax){
            distMax = distTmp;
            ii = i;
        }
    }

    /// Adding the final corner
    corners.push_back(centers[ii]);

    /// Erase from centers
    centers.erase(centers.begin() + ii);

    /// Put them in order
    /*std::cout<<"Corners "<<corners.size()<<std::endl;
    for(int i = 0; i < corners.size(); i++){
        std::cout<<"("<<corners[i].x<<", "<<corners[i].y<<")"<<std::endl;
    }*/

    std::sort(corners.begin(), corners.end(), [](cv::Point2f const& f, cv::Point2f const& s){ return f.x < s.x; });

    tmpCorners.resize(4);

    if(corners[0].y > corners[1].y){            
        tmpCorners[2] = corners[0];
        tmpCorners[0] = corners[1];        
    }
    else{
        tmpCorners[0] = corners[0];
        tmpCorners[2] = corners[1];            
    }

    if(corners[2].y > corners[3].y){
        tmpCorners[3] = corners[2];
        tmpCorners[1] = corners[3];            
    }
    else{            
        tmpCorners[1] = corners[2];
        tmpCorners[3] = corners[3];        
    }

    corners.clear();
    corners = tmpCorners;

    /*std::cout<<"Print points after... "<<centers.size()<<std::endl; 
    std::cout<<"Print points after... "<<tmpCenters.size()<<std::endl; 
    for(int x=0; x<centers.size();x++){
        std::cout<<"("<<centers[x].x<<", "<<centers[x].y<<")"<<std::endl;
    }*/    
}


void getProperOrder(std::vector<cv::Point2f> &centers, std::vector<cv::Point2f> &corners){
    ///***** get slope ****///
    /*double slope, length;
    slope  = (tmpCenters[0].y - tmpCenters[patternSize-patternWidth].y) / (tmpCenters[0].x - tmpCenters[patternSize-patternWidth].x)
    length = norm(tmpCenters[0] - tmpCenters[0])*/

    /*circle(frame,cv::Point2f(50,50), 1, cv::Scalar(0, 255, 255), 4, 8);
    circle(frame,cv::Point2f(250,250), 1, cv::Scalar(0, 255, 255), 4, 8);*/   
             
}

bool verifyOrder(std::vector<cv::Point2f> &probableCPs, int num_rows, int num_cols){
    int n = probableCPs.size();
    std::vector<cv::Vec4f> lines;
    std::vector< std::vector<int> > combinations = GenerateCombinations(probableCPs.size(),num_cols);
    std::vector<cv::Vec4f> preSelectedLines;
    std::vector<std::vector<int> > combination_preSelectedLines;
    for(int i = 0; i < combinations.size();i++){
        std::vector<cv::Point2f> tmpPoints(num_cols);
        cv::Vec4f tmpLine;
        for(int j = 0; j < num_cols; j++){
            tmpPoints[j] = probableCPs[ combinations[i][j] ];
        }
        fitLine(tmpPoints,tmpLine,cv::DIST_L2,0,0.01,0.01);
        //Le damos forma a los valores vectoriales que nos devuelve fitline
        // r = a + r*b --> p0 punto de paso, v vector director normalizado
        float vx = tmpLine[0],vy = tmpLine[1], x0 = tmpLine[2],y0 = tmpLine[3];
        cv::Point2f a = cv::Point2f(x0,y0), b = cv::Point2f(vx,vy);

        std::vector<float> distances;
        for(int k = 0; k < num_cols; k++){
            //Calculamos la distancia del punto a la recta y almacenamos para el calculo de la desviacion
            float t = ( tmpPoints[k].dot(b) - a.dot(b) ) / (cv::norm(b) * cv::norm(b));
            float dist = cv::norm(tmpPoints[k] - (a + t * b));
            distances.push_back(dist);
        }

        float stddev = StandarDesviation(distances);

        //Si el error de la linea no es mucho. Seleccionamos la linea
        if(stddev < 0.5f){
            preSelectedLines.push_back(tmpLine);
            //Guardamos la Combinacion
            combination_preSelectedLines.push_back(combinations[i]);
        }

    }

    // Apply some filters here to verify line selection
    // Then Order Points and Store in CPs(Hard verification of only 20 Ordered and Aligned Control Points)
    // Acordemonos que ya seleccionamos solo lineas con 5 puntos
    if(preSelectedLines.size() == num_rows){
        //Tenemos que ordenar las lineas. (Recordemos que son lineas paralelas)
        //Primero verificamos la pendiente

        //LINE ORDERING
            //Recordemos la grilla que presenta openCV 
            // -------> x+
            // |
            // |
            // y+

            cv::Vec4f Line = preSelectedLines[0];
            float vx = Line[0],vy = Line[1], x0 = Line[2],y0 = Line[3];
            //Pendiente
            float slope = vy/vx;
            if(abs(slope) < 5.0f){ //Evaluamos las pendientes de casi 80 grados (Revisar esta funcion)
                std::vector<float> y_intersection(num_rows);
                //Calcular el punto de interseccion por el eje y
                for(int i = 0; i < num_rows; i++){
                    cv::Vec4f tmpLine = preSelectedLines[0];
                    float vx = tmpLine[0],vy = tmpLine[1], x0 = tmpLine[2],y0 = tmpLine[3];

                    float t = -x0 / vx;
                    float y = y0 + t*vy;

                    y_intersection[i] = y;
                }

                //Realizamos un bubble sort en base a las intersecciones con el eje y
                //ordenamiento por burbuja
                bool swapp = true;
                while(swapp)
                {
                    swapp = false;
                    for (int i = 0; i < preSelectedLines.size()-1; i++)
                    {
                        if (y_intersection[i] > y_intersection[i+1] ){
                            //Cambiamos en todos nuestros vectores
                            std::swap(y_intersection[i],y_intersection[i+1]);
                            std::swap(preSelectedLines[i],preSelectedLines[i+1]);
                            std::swap(combination_preSelectedLines[i],combination_preSelectedLines[i+1]);
                            swapp = true;
                        }
                    }
                }// Fin del ordenamiento

                // Para Cada Linea obtener los CP segun la combinacion y ordenarlos por el eje X
                // Obtenemos los puntos desde el CP

                std::vector<cv::Point2f> tmpCPs;
                for(int i = 0; i < num_rows; i++){
                    std::vector<cv::Point2f> tmpCenters(num_cols);
                    for(int j = 0; j < num_cols; j++){
                        tmpCenters[j] = probableCPs[ combination_preSelectedLines[i][j] ];
                    }
                    sort(tmpCenters.begin(), tmpCenters.end(),cmpx);
                    for(int j = 0; j < num_cols; j++){
                        tmpCPs.push_back(tmpCenters[j]);
                    }
                }

                probableCPs.clear();
                probableCPs = tmpCPs;

                return true;
            }      
    }
    return false;
}

std::vector< std::vector<int> > GenerateCombinations(int n, int r){
    std::vector< std::vector<int> > v;

    int arr[n];
    for(int i = 0; i < n; i++)
        arr[i] = i;

    printCombination(v, arr, n, r);

    return v;
}

void printCombination(std::vector< std::vector<int> >& v, int arr[], int n, int r)
{
    std::vector<int> data(r);

    combinationUtil(v, arr, data, 0, n-1, 0, r);
}
 
void combinationUtil(std::vector< std::vector<int> >& v, int arr[], std::vector<int> &data, int start, int end,
                     int index, int r)
{
    if (index == r)
    {
        v.push_back(data);
        return;
    }
 
    for (int i=start; i<=end && end-i+1 >= r-index; i++)
    {
        data[index] = arr[i];
        combinationUtil(v, arr, data, i+1, end, index+1, r);
    }
}

float StandarDesviation(const std::vector<float> & values ){
	int n = values.size();
    float dmean = 0.0;
    float dstddev = 0.0;

    // Mean standard algorithm
    for (int i = 0; i < n; ++i)
    {
       dmean += values[i];
    }
    dmean /= (float)n;

    // Standard deviation standard algorithm
    std::vector<float> var(n);

    for (int i = 0; i < n; ++i){
        var[i] = (dmean - values[i]) * (dmean - values[i]);
    }

    for (int i = 0; i < n; ++i){
        dstddev += var[i];
    }
    dstddev = sqrt(dstddev / (float)n);
    //std::cout << "Mean: " << dmean << "   StdDev: " << dstddev << std::endl;

    return dstddev;
}

bool cmpx(cv::Point2f a, cv::Point2f b){
    return a.x < b.x;
}

/*centers.clear();
                    centers = tmpCenters;

                    for(int i = 0; i < centers.size();i++){            
                        circle(frame, centers[i], 1, cv::Scalar(0, 0, 255), 4, 8);
                        //cv::putText(frame, std::to_string(i), centers[i],cv::FONT_HERSHEY_DUPLEX,0.5, cv::Scalar(250, 0, 0),2);
                    }*/

                    //findCorners(centers, corners);
                    /*for(int i=0; i<corners.size(); i++){
                        circle(frame, corners[i], 1, cv::Scalar(255, 0, 0), 4, 8);
                        //cv::putText(frame, std::to_string(i), corners[i],cv::FONT_HERSHEY_DUPLEX,0.5, cv::Scalar(130, 0, 0), 1);        
                    }*/

                    /// Checking similar distances between corners
                    /// 0 - 1 should be similar 2 - 3
                    //std::cout<<"0-1: " << cv::norm(corners[0] - corners[1]) << " 2-3: " <<cv::norm(corners[2] - corners[3]) <<std::endl;

                    /// 0 - 2 should be similar 1 - 3
                    //std::cout<<"0-2: " << cv::norm(corners[0] - corners[2]) << " 1-3: " <<cv::norm(corners[1] - corners[3]) <<std::endl;

                    /*if(abs(cv::norm(corners[0] - corners[1]) - cv::norm(corners[2] - corners[3])) < thresholdDist && abs(cv::norm(corners[0] - corners[2]) - cv::norm(corners[1] - corners[3])) < thresholdDist){
                        frameCountCorrect++;
                        line(frame, corners[0], corners[1], cv::Scalar(200, 80, 80), 1.5, cv::LINE_8, 0);
                        line(frame, corners[1], corners[2], cv::Scalar(200, 80, 80), 1.5, cv::LINE_8, 0);
                        line(frame, corners[2], corners[3], cv::Scalar(200, 80, 80), 1.5, cv::LINE_8, 0);
                    }*/
            
                    /// Put the rest in order
                    /*tmpCenters.clear();
                    tmpCenters.resize(patternSize);
                            
                    std::sort(centers.begin(), centers.end(), [](cv::Point2f const& f, cv::Point2f const& s){ return f.x < s.x; });*/

                    //getProperOrder(centers, corners);

                    /*for(int i=0; i<centers.size(); i++){
                        circle(frame, centers[i], 1, cv::Scalar(0, 0, 255), 4, 8);
                        //cv::putText(frame, std::to_string(i), centers[i],cv::FONT_HERSHEY_DUPLEX,0.5, cv::Scalar(0, 0, 255),1);        
                    }*/                            
                
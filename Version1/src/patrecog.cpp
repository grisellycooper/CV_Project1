#include "../include/patrecog.h"

#define threshold 0.75f
#define displayCompletePreprocess 0
#define displayCompleteFilter1 1
#define displayCompleteFilter2 0

int found;

void preprocessImage(cv::Mat src, cv::Mat output,
                     std::vector<std::vector<cv::Point>> &contours,
                     std::vector<cv::Vec4i> &hierarchy)
{
    cv::Mat gray, bw, cont;

    // Convert to grayscale and apply Gaussian blur
    // Reduce information and noise
    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0, 0);
    if (displayCompletePreprocess == 1)
    {
        cv::namedWindow("Grayscale Gaussian Blur", cv::WINDOW_NORMAL);
        imshow("Grayscale Gaussian Blur", gray);
    }

    // Convert image to binary
    //cv::threshold(gray, bw, 100, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    cv::adaptiveThreshold(gray, bw, 200, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 21, 10);
    if (displayCompletePreprocess == 1)
    {
        cv::namedWindow("Binary", cv::WINDOW_NORMAL);
        imshow("Binary", bw);
    }

    cont = cv::Mat::zeros(src.rows, src.cols, CV_8UC3);
    // Find all the contours in the thresholded image
    cv::findContours(bw, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE, cv::Point(0, 0));
    if (displayCompletePreprocess == 1)
    {
        for (int i = 0; i < contours.size(); i++)
        {
            cv::drawContours(cont, contours, static_cast<int>(i), cv::Scalar(0, 0, 255), 2);
        }
        cv::namedWindow("Contour", cv::WINDOW_NORMAL);
        imshow("Contour", cont);
    }
    return;
}

void identifyRings(std::vector<std::vector<cv::Point>> &contours,
                   std::vector<cv::Vec4i> &hierarchy,
                   std::vector<cv::Point2f> &pointbuf,
                   int patternSize,
                   cv::Mat src)
{
    std::vector<cv::RotatedRect> minEllipse;
    std::vector<cv::RotatedRect> minEllipseSelected;
    std::vector<cv::Point2f> tmpCenters;

    //float w, h, c_x, c_y, child_c_x, child_c_y, distance;
    //float sumX, sumY,cpX, cpY;
    float distance;
    float sumX, sumY;
    int child_index, radialZone;

    sumX = sumY = 0.0;

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
                /*if (displayCompleteProcess == 1)
                {
                    ellipse(cont, minEllipse[i], cv::Scalar(0, 0, 255), 2, 8);
                    ellipse(cont, minEllipse[hierarchy[i][2]], cv::Scalar(0, 0, 255), 1, 8);
                }*/
                tmpCenters.push_back(cv::Point2f((minEllipse[i].center.x + minEllipse[child_index].center.x) / 2, (minEllipse[i].center.y + minEllipse[child_index].center.y) / 2));
            }
        }
    }

    for (int i = 0; i < tmpCenters.size(); i++)
    {
        sumX += tmpCenters[i].x;
        sumY += tmpCenters[i].y;
    }

    /// Finding an average Central Point
    cv::Point2f avg(sumX / tmpCenters.size(), sumY / tmpCenters.size());

    for (radialZone = 1; radialZone < 200; radialZone++)
    {
        int count = 0;
        for (int i = 0; i < tmpCenters.size(); i++)
        {
            if (cv::norm(avg - tmpCenters[i]) < radialZone)
                count++;
        }
        if (count >= patternSize)
        {
            break;
        }
    }

    /// Add circles in the radial zone
    for (int i = 0; i < tmpCenters.size(); i++)
    {
        if (cv::norm(avg - tmpCenters[i]) < radialZone + 20)
            pointbuf.push_back(tmpCenters[i]);
    }

    if (displayCompleteFilter1 == 1)
    {
        cv::Mat test1 = cv::Mat::zeros(src.rows, src.cols, CV_8UC3);
        for (int i = 0; i < tmpCenters.size(); i++)
        {
            circle(test1, avg, 1, cv::Scalar(255, 0, 0), 4, 8);
            circle(test1, tmpCenters[i], 1, cv::Scalar(0, 0, 255), 4, 8);
            //cv::putText(test1, std::to_string(i), tmpCenters[i], cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(250, 0, 0), 2);
        }

        cv::namedWindow("Identify Rings", cv::WINDOW_NORMAL);
        imshow("Identify Rings", test1);
    }
}

bool findRingsGrid(cv::Mat src, cv::Size patternSize, std::vector<cv::Point2f>& pointbuf,
                   std::vector<cv::Point2f>& previousPointbuf, bool prevoiusFound)
{
    int iPatternSize = patternSize.width * patternSize.height;
    std::vector<cv::Point2f> tmpCenters;
    bool orderFound;
    cv::Mat test2;

    tmpCenters = pointbuf;

    if (prevoiusFound)
    {
        float currDist, minDist;
        int minIndex, c;
        for (int k = 0; k < iPatternSize; k++)
        {
            cv::Point2f tmp = previousPointbuf[k];
            minDist = 1000.0;
            minIndex = 0;
            for (int i = 0; i < tmpCenters.size(); i++)
            {
                currDist = cv::norm(previousPointbuf[k] - tmpCenters[i]);
                if (minDist > currDist)
                {
                    minDist = currDist;
                    minIndex = i;
                }
            }
            
            /// Verify the displacement between posible same points, the displacement shouldnt be large
            if (minDist < 3.0)
            {
                c++;    /// count points which verify min displacement
                pointbuf[k] = tmpCenters[minIndex];
            }
            else
            {
                //std::cout<<"mindist: " << minDist <<std::endl;
            }
        }

        /// Points that verify min displacment should be the same size as the patternSize 
        if (c == iPatternSize)
        {
            if(displayCompleteFilter2)
            {
                for (int i = 0; i < iPatternSize; i++)
                {
                    circle(test2, pointbuf[i], 1, cv::Scalar(0, 0, 255), 4, 8);
                    cv::putText(test2, std::to_string(i), pointbuf[i], cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(250, 0, 0), 2);
                }
            }            
            return true;
        }        
    }
    else
    {
        orderFound = verifyOrder(tmpCenters, patternSize.height, patternSize.width);

        if (orderFound)
        {
            pointbuf.clear();
            for (int i = 0; i < iPatternSize; i++)
            {
                pointbuf.push_back(tmpCenters[i]);
                if(displayCompleteFilter2)
                {
                    circle(test2, tmpCenters[i], 1, cv::Scalar(0, 0, 255), 4, 8);
                    cv::putText(test2, std::to_string(i), tmpCenters[i], cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(250, 0, 0), 2);
                }                
            }
            return true;
        }
    }
    return false;
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

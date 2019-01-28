#include "../include/patrecog.h"

#define thresholdDistCircles 1.0f
#define thresholdDistColinearity 5.0f  /// To prevent high distortion  
#define displayCompletePreprocess 0
#define displayCompleteFilter1 0
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
    bw = cv::Mat::zeros(gray.size(), CV_8UC1);
    //cv::threshold(gray, bw, 100, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    //cv::adaptiveThreshold(gray, bw, 250, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 25, -1);
    cv::adaptiveThreshold(gray, bw, 200, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 43, 3);
    //thresholdIntegral(gray, bw);
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
    
    cv::Mat test1 = cv::Mat::zeros(src.rows, src.cols, CV_8UC3);

    //float w, h, c_x, c_y, child_c_x, child_c_y, distance;
    //float sumX, sumY,cpX, cpY;
    float distance;
    float sumX, sumY;
    int child_index, radialZone, tmpMinDistTwoCenters;
    float minDistTwoCenters = 1000.0f;

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
        //std::cout<<"*  " <<std::endl;
        if (child_index != -1 && hierarchy[child_index][2] == -1) //Check out for child but not grandchild
        {
            //Check distances between parent's center and child's center
            //Should be less than threshold
            distance = cv::norm(minEllipse[i].center - minEllipse[child_index].center);
            /*if(distance < 5.0)
                std::cout<<"dist Parent-Child: " <<distance <<std::endl;*/
            if (distance < thresholdDistCircles)
            {
                minEllipseSelected.push_back(minEllipse[i]);
                minEllipseSelected.push_back(minEllipse[hierarchy[i][2]]);
                if (displayCompleteFilter1 == 1)
                {
                    ellipse(test1, minEllipse[i], cv::Scalar(0, 0, 255), 2, 8);
                    ellipse(test1, minEllipse[hierarchy[i][2]], cv::Scalar(0, 0, 255), 1, 8);
                }
                tmpCenters.push_back(cv::Point2f((minEllipse[i].center.x + minEllipse[child_index].center.x) / 2, (minEllipse[i].center.y + minEllipse[child_index].center.y) / 2));

                if(tmpCenters.size() > 1){
                    tmpMinDistTwoCenters = cv::norm(tmpCenters[tmpCenters.size()-1] - tmpCenters[tmpCenters.size()-2]);
                    if(tmpMinDistTwoCenters < minDistTwoCenters)
                        minDistTwoCenters = tmpMinDistTwoCenters;
                }
            }
        }
    }

    /*std::cout<<"tmpCenterSize: " <<tmpCenters.size() <<std::endl;
    std::cout<<"minDistTwoCenters: " <<minDistTwoCenters <<std::endl;*/

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
        if (cv::norm(avg - tmpCenters[i]) < radialZone + ceil(minDistTwoCenters))
            pointbuf.push_back(tmpCenters[i]);
    } 

    if (displayCompleteFilter1 == 1)
    {        
        circle(test1, avg, 1, cv::Scalar(255, 0, 0), 4, 8);            
        for (int i = 0; i < pointbuf.size(); i++)
        {
            circle(test1, pointbuf[i], 1, cv::Scalar(0, 0, 255), 4, 8);
            //cv::putText(test1, std::to_string(i), tmpCenters[i], cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(250, 0, 0), 2);
        }

        cv::namedWindow("Identify Rings", cv::WINDOW_NORMAL);
        imshow("Identify Rings", test1);
    }
}

void preprocessImage2(cv::Mat src, cv::Mat output,
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
    bw = cv::Mat::zeros(gray.size(), CV_8UC1);
    //cv::threshold(gray, bw, 100, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    //cv::adaptiveThreshold(gray, bw, 250, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 25, -1);
    //cv::adaptiveThreshold(gray, bw, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 39, 7);
    thresholdIntegral(gray, bw);
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

void identifyRings2(std::vector<std::vector<cv::Point>> &contours,
                   std::vector<cv::Vec4i> &hierarchy,
                   std::vector<cv::Point2f> &pointbuf,
                   int patternSize,
                   cv::Mat src)
{
    std::vector<cv::RotatedRect> minEllipse;
    std::vector<cv::RotatedRect> minEllipseSelected;
    std::vector<cv::Point2f> tmpCenters;

    cv::Mat test1 = cv::Mat::zeros(src.rows, src.cols, CV_8UC3);

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
        //std::cout<<"*  " <<std::endl;
        if (child_index != -1 && hierarchy[child_index][2] == -1) //Check out for child but not grandchild
        {
            //Check distances between parent's center and child's center
            //Should be less than threshold
            distance = cv::norm(minEllipse[i].center - minEllipse[child_index].center);
            //std::cout<<"dist: " <<distance <<std::endl;
            if (distance < thresholdDistCircles)
            {
                minEllipseSelected.push_back(minEllipse[i]);
                minEllipseSelected.push_back(minEllipse[hierarchy[i][2]]);
                if (displayCompletePreprocess == 1)
                {
                    ellipse(test1, minEllipse[i], cv::Scalar(0, 0, 255), 2, 8);
                    ellipse(test1, minEllipse[hierarchy[i][2]], cv::Scalar(0, 0, 255), 1, 8);
                }
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
    /*cv::Point2f avg(sumX / tmpCenters.size(), sumY / tmpCenters.size());

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
*/
    for (int i = 0; i < tmpCenters.size(); i++)
    {
        pointbuf.push_back(tmpCenters[i]);
    }

    if (displayCompleteFilter1 == 1)
    {
        
        //circle(test1, avg, 1, cv::Scalar(255, 0, 0), 4, 8);            
        for (int i = 0; i < tmpCenters.size(); i++)
        {
            //pointbuf.push_back(tmpCenters[i]);
            circle(test1, tmpCenters[i], 1, cv::Scalar(0, 0, 255), 4, 8);
            //cv::putText(test1, std::to_string(i), tmpCenters[i], cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(250, 0, 0), 2);
        }

        cv::namedWindow("Identify Rings", cv::WINDOW_NORMAL);
        imshow("Identify Rings", test1);
    }
}

bool findRingsGrid(cv::Mat src, cv::Size patternSize, std::vector<cv::Point2f> &pointbuf,
                   std::vector<cv::Point2f> &previousPointbuf, bool prevoiusFound)
{
    int iPatternSize = patternSize.width * patternSize.height;
    std::vector<cv::Point2f> tmpPointBuf;
    bool orderFound;
    cv::Mat test2 = cv::Mat::zeros(src.rows, src.cols, CV_8UC3);

    tmpPointBuf = pointbuf;  
    //prevoiusFound = false;  
    if (prevoiusFound)
    {
        //std::cout<<" -- 1: " ;
        float currDist, minDist, stdv;
        int minIndex, c = 0;
        std::vector<float> distances;
        for (int k = 0; k < iPatternSize; k++)
        {
            //std::cout<<"*1 "<<std::endl;
            cv::Point2f tmp = previousPointbuf[k];
            minDist = 1000.0;
            minIndex = 0;
            for (int i = 0; i < tmpPointBuf.size(); i++)
            {
                currDist = cv::norm(previousPointbuf[k] - tmpPointBuf[i]);
                if (minDist > currDist)
                {
                    minDist = currDist;
                    minIndex = i;
                }
            }
            distances.push_back(cv::norm(previousPointbuf[k] - tmpPointBuf[minIndex]));
            pointbuf[k] = tmpPointBuf[minIndex];
            //std::cout<<"mindist: "<< minDist <<std::endl;
            /// Verify the displacement between posible same points, the displacement shouldnt be large
            /*if (minDist < 3.0)
            {
                c++; /// count points which verify min displacement
                pointbuf[k] = tmpPointBuf[minIndex];
            }
            else
            { /* std::cout<<"mindist: " << minDist <<std::endl; */
            //}
        }
        stdv = StandarDesviation(distances);
        //std::cout<<"std: " << stdv <<std::endl;

        /// Points that verify min displacment should be the same size as the patternSize
        /*if (c == iPatternSize)
        {*/
        if(stdv < 1.5)
        {
            std::cout<<" -- 1: " <<std::endl;            
            //std::cout<<" found! "<<std::endl;
            //pointbuf[k] = tmpPointBuf[minIndex];
            if (displayCompleteFilter2 == 1)
            {
                for (int i = 0; i < iPatternSize; i++)
                {
                    circle(test2, pointbuf[i], 1, cv::Scalar(0, 0, 255), 4, 8);
                    cv::putText(test2, std::to_string(i), pointbuf[i], cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(250, 0, 0), 2);
                }

                cv::namedWindow("Find Grid", cv::WINDOW_NORMAL);
                imshow("Find Grid", test2);
            }
            return true;
        }
        else{
            std::cout<<" -- 2: " <<std::endl;
            orderFound = verifyOrder(tmpPointBuf, patternSize.height, patternSize.width);
            
            if (orderFound)
            {
                pointbuf.clear();
                for (int i = 0; i < iPatternSize; i++)
                {
                    pointbuf.push_back(tmpPointBuf[i]);
                }

                if (displayCompleteFilter2 == 1)
                {
                    for (int i = 0; i < iPatternSize; i++)
                    {
                        circle(test2, pointbuf[i], 1, cv::Scalar(0, 0, 255), 4, 8);
                        cv::putText(test2, std::to_string(i), pointbuf[i], cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(250, 0, 0), 2);
                    }

                    cv::namedWindow("Find Grid", cv::WINDOW_NORMAL);
                    imshow("Find Grid", test2);
                }

                return true;
            }    
        }
    }
    else
    {
        std::cout<<" -- 0: " <<std::endl;            
        //std::cout<<" -- 2: " ;
        orderFound = verifyOrder(tmpPointBuf, patternSize.height, patternSize.width);
        
        if (orderFound)
        {
            //std::cout<<" found! " <<std::endl;
            pointbuf.clear();
            for (int i = 0; i < iPatternSize; i++)
            {
                pointbuf.push_back(tmpPointBuf[i]);
            }

            if (displayCompleteFilter2 == 1)
            {
                for (int i = 0; i < iPatternSize; i++)
                {
                    circle(test2, pointbuf[i], 1, cv::Scalar(0, 0, 255), 4, 8);
                    cv::putText(test2, std::to_string(i), pointbuf[i], cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(250, 0, 0), 2);
                }

                cv::namedWindow("Find Grid", cv::WINDOW_NORMAL);
                imshow("Find Grid", test2);
            }

            return true;
        }        
    }
    return false;
}

bool verifyOrder(std::vector<cv::Point2f> &tmpPB, int patternHeigh, int patternWidth)
{
    int n = tmpPB.size();
    std::vector<cv::Vec4f> lines;
    std::vector<std::vector<int>> combinations = GenerateCombinations(tmpPB.size(), patternWidth); /// Generate all posible combinations of patternWidth among points
    std::vector<cv::Vec4f> preSelectedLines;
    std::vector<std::vector<int>> preSelectedLinesCombinations;

    std::vector<cv::Point2f> tmpLinePoints(patternWidth);
    cv::Vec4f tmpLine;
    float dist, maxDist, t;

    for (int i = 0; i < combinations.size(); i++)
    {
        /// Get the points that belong to each line
        for (int j = 0; j < patternWidth; j++)
        {
            tmpLinePoints[j] = tmpPB[combinations[i][j]];
        }

        /// Detect lines
        ///tmpLine -> [vx, vy, x0, y0] -> (vx, vy) is a normalized vector collinear to the line and (x0, y0) is a point on the line
        fitLine(tmpLinePoints, tmpLine, cv::DIST_L2, 0, 0.01, 0.01);
        cv::Point2f a = cv::Point2f(tmpLine[2], tmpLine[3]), b = cv::Point2f(tmpLine[0], tmpLine[1]);

        maxDist = 0.0;
        for (int k = 0; k < patternWidth; k++)
        {
            /// Get the distance of the farest away point to the line
            t = (tmpLinePoints[k].dot(b) - a.dot(b)) / (cv::norm(b) * cv::norm(b));
            dist = cv::norm(tmpLinePoints[k] - (a + t * b));
            if (maxDist < dist)
                maxDist = dist;
        }
        /// The distance should be less that a threashold, if it fits, its selected
        if (maxDist < thresholdDistColinearity)
        {
            //std::cout<<"Dist: "<<maxDist <<std::endl;                    
            preSelectedLines.push_back(tmpLine);
            preSelectedLinesCombinations.push_back(combinations[i]);
        }
    }
    //std::cout<<"PreSelecSize: "<<preSelectedLines.size() <<std::endl;

    /// Validate line selection, shoud be same size as patternHeigh
    if (preSelectedLines.size() == patternHeigh)
    {
        std::vector<float> y_intersections(patternHeigh);
        float y;
        for (int i = 0; i < patternHeigh; i++)
        {
            tmpLine = preSelectedLines[i];
            t = -tmpLine[2] / tmpLine[0];
            y = tmpLine[3] + t * tmpLine[1];
            y_intersections[i] = y;
        }

        /// Get lines in order by Y-axes
        bool swapp = true;
        while (swapp)
        {
            swapp = false;
            for (int i = 0; i < preSelectedLines.size() - 1; i++)
            {
                if (y_intersections[i] > y_intersections[i + 1])
                {
                    std::swap(y_intersections[i], y_intersections[i + 1]);
                    std::swap(preSelectedLines[i], preSelectedLines[i + 1]);
                    std::swap(preSelectedLinesCombinations[i], preSelectedLinesCombinations[i + 1]);
                    swapp = true;
                }
            }
        }

        /// Get points in order by X-axes
        std::vector<cv::Point2f> tmpBuff;
        for (int i = 0; i < patternHeigh; i++)
        {
            std::vector<cv::Point2f> tmpCenters(patternWidth);
            for (int j = 0; j < patternWidth; j++)
            {
                tmpCenters[j] = tmpPB[preSelectedLinesCombinations[i][j]];
            }
            std::sort(tmpCenters.begin(), tmpCenters.end(), [](cv::Point2f const &f, cv::Point2f const &s) { return f.x < s.x; });
            for (int j = 0; j < patternWidth; j++)
            {
                tmpBuff.push_back(tmpCenters[j]);
            }
        }

        tmpPB.clear();
        tmpPB = tmpBuff;

        return true;
    }
    return false;
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

std::vector<std::vector<int>> GenerateCombinations(int n, int r)
{
    std::vector<std::vector<int>> v;

    int arr[n];
    for (int i = 0; i < n; i++)
        arr[i] = i;

    printCombination(v, arr, n, r);

    return v;
}

void printCombination(std::vector<std::vector<int>> &v, int arr[], int n, int r)
{
    std::vector<int> data(r);

    combinationUtil(v, arr, data, 0, n - 1, 0, r);
}

void combinationUtil(std::vector<std::vector<int>> &v, int arr[], std::vector<int> &data, int start, int end,
                     int index, int r)
{
    if (index == r)
    {
        v.push_back(data);
        return;
    }

    for (int i = start; i <= end && end - i + 1 >= r - index; i++)
    {
        data[index] = arr[i];
        combinationUtil(v, arr, data, i + 1, end, index + 1, r);
    }
}

void thresholdIntegral(cv::Mat &inputMat, cv::Mat &outputMat)
{
    // accept only char type matrices
    CV_Assert(!inputMat.empty());
    CV_Assert(inputMat.depth() == CV_8U);
    CV_Assert(inputMat.channels() == 1);
    CV_Assert(!outputMat.empty());
    CV_Assert(outputMat.depth() == CV_8U);
    CV_Assert(outputMat.channels() == 1);

    // rows -> height -> y
    int nRows = inputMat.rows;
    // cols -> width -> x
    int nCols = inputMat.cols;

    // create the integral image
    cv::Mat sumMat;
    cv::integral(inputMat, sumMat);

    CV_Assert(sumMat.depth() == CV_32S);
    CV_Assert(sizeof(int) == 4);

    int S = MAX(nRows, nCols)/8;
    double T = 0.15;

    // perform thresholding
    int s2 = S/2;
    int x1, y1, x2, y2, count, sum;

    // CV_Assert(sizeof(int) == 4);
    int *p_y1, *p_y2;
    uchar *p_inputMat, *p_outputMat;

    for( int i = 0; i < nRows; ++i)
    {
        y1 = i-s2;
        y2 = i+s2;

        if (y1 < 0){
            y1 = 0;
        }
        if (y2 >= nRows) {
            y2 = nRows-1;
        }

        p_y1 = sumMat.ptr<int>(y1);
        p_y2 = sumMat.ptr<int>(y2);
        p_inputMat = inputMat.ptr<uchar>(i);
        p_outputMat = outputMat.ptr<uchar>(i);

        for ( int j = 0; j < nCols; ++j)
        {
            // set the SxS region
            x1 = j-s2;
            x2 = j+s2;

            if (x1 < 0) {
                x1 = 0;
            }
            if (x2 >= nCols) {
                x2 = nCols-1;
            }

            count = (x2-x1)*(y2-y1);

            // I(x,y)=s(x2,y2)-s(x1,y2)-s(x2,y1)+s(x1,x1)
            sum = p_y2[x2] - p_y1[x2] - p_y2[x1] + p_y1[x1];

            if ((int)(p_inputMat[j] * count) < (int)(sum*(1.0-T)))
                p_outputMat[j] = 255;
            else
                p_outputMat[j] = 0;
        }
    }
}
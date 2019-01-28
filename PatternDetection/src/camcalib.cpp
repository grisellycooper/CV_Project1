#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include "../include/camcalib.h"

enum Pattern
{
    CHESSBOARD,
    CIRCLES_GRID,
    ASYMMETRIC_CIRCLES_GRID,
    RINGS_GRID
};

void getControlPointsPositions(cv::Size size, float ctrlPointsDistance, std::vector<cv::Point3f>& controlPoints, int patternType){
    controlPoints.clear();
    float x, y, z = 0;
    switch(patternType){
        case CHESSBOARD:
        case CIRCLES_GRID:
        case RINGS_GRID:            
            for(int i = 0 ; i < size.height; i++){
                for(int j = 0; j < size.width; j++){
                    x = j * ctrlPointsDistance;
                    y = i * ctrlPointsDistance;
                    controlPoints.push_back(cv::Point3f(x,y,0) );                    
                    //std::cout<<"x: "<<x<<"y: "<<y <<std::endl;
                }
            }
            //std::cout<<" " <<controlPoints.size() ;
            break;

        case ASYMMETRIC_CIRCLES_GRID:
            for( int i = 0; i < size.height; i++ ){
                for( int j = 0; j < size.width; j++ ){
                    
                    controlPoints.push_back(cv::Point3f(float((2*j + i % 2)*ctrlPointsDistance), float(i*ctrlPointsDistance), 0));
                }
            }
            break;
    }
}
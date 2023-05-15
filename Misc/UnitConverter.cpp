/* CUSTOM INCLUDES ----------------------------------------------------------*/
#include "UnitConverter.hpp"

/* METHODS ------------------------------------------------------------------*/
float UnitConverter::calcPXToCM(std::vector<cv::Point2f> arucoCorners)
{
    if(arucoCorners.size() != 4){
        return 0.f;
    }

    cv::Point2f firstCorner = arucoCorners[0];
    cv::Point2f secondCorner = arucoCorners[1];

    cv::Vec2f sizeVector = cv::Vec2f(firstCorner.x - secondCorner.x,
            firstCorner.y - secondCorner.y);
    float sizeInPx = std::sqrt(std::pow(sizeVector[0], 2.f) + 
            std::pow(sizeVector[1], 2.f));

    /* We need to divide know centimeters with pixels */
    return (float) (ARUCO_SIZE_CM / sizeInPx);
}


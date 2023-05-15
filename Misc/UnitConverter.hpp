#pragma once

/* LIBRARY INCLUDES ---------------------------------------------------------*/
#include <vector>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>

/* CUSTOM INCLUDES ----------------------------------------------------------*/
#include "../config.hpp"

/* CLASSES ------------------------------------------------------------------*/
class UnitConverter
{
    public:
        float calcPXToCM(std::vector<cv::Point2f> arucoCorners);
};

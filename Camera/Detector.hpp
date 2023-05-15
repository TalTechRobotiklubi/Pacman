#pragma once

/* LIBRARY INCLUDES ---------------------------------------------------------*/
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/aruco.hpp>

/* CUSTOM INCLUDES ----------------------------------------------------------*/
#include "../config.hpp"
#include "../Misc/Time.hpp"

/* CLASSES ------------------------------------------------------------------*/
class Detector
{
    public:
        Detector();
        cv::Mat detectArucos(const cv::Mat frame, const int drawMarkers);
        std::vector<int> getIds();
        std::vector<std::vector<cv::Point2f>> getCorners();
    protected:
        cv::Ptr<cv::aruco::Dictionary> arucoDict;
        cv::Ptr<cv::aruco::DetectorParameters> detectorParameters;
        std::vector<int> newIds;
        std::vector<std::vector<cv::Point2f>> newCorners;
};

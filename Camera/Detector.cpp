/* CUSTOM INCLUDES ----------------------------------------------------------*/
#include "Detector.hpp"

/* Construct a new Detector object. The camera uses OpenCV VideoCapture or
 * xiAPIplusCameraOcv object and ArUco codes for the detection process.
 *
 * Class variables:
 *      arucoDict - cv::Ptr<cv::aruco::Dictionary>, protected, The ArUco
 *                  dictionary used for ArUco detection (there are multiple
 *                  ones, but we are using 6x6 1000) (do not touch this unless
 *                  you know exactly what are you doing)
 *      detectorParameters - cv::Ptr<cv::aruco::DetectorParameters>, protected,
 *                           OpenCV detector parameters (again, you really
 *                           shuld not touch this)
 *      newIds - std::vector<int>, protected, The IDs that were detected on the
 *               last frame
 *      newCorners - std::vector<std::vector<cv::Point2f>>, protected, The ArUco
 *                   corners that were detected on the last frame
 *
 */
/* METHODS ------------------------------------------------------------------*/
Detector::Detector()
{
    this->arucoDict =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);

    this->detectorParameters = cv::aruco::DetectorParameters::create();
}

/**
 * Detect the ArUco codes (robots) from the provided frame.
 *
 * Parameters:
 *      frame - cv::Mat, The frame where the detection will take place. See
 *              RobotManager::getFrame()
 *      drawMarkekrs - int, boolean value, if we should draw markers on the 
                       returned frame (can be useful for later displaying 
                       the detected frame)
 *
 * Returns: cv::Mat, Frame where the detection took place (with markers if
 *          drawMarkers was true)
 */
cv::Mat Detector::detectArucos(const cv::Mat frame, const int drawMarkers)
{
    if(!frame.empty()){
        cv::aruco::detectMarkers(frame, this->arucoDict, this->newCorners,
            this->newIds, this->detectorParameters);

        if(this->newIds.size() > 0 && drawMarkers){
            cv::aruco::drawDetectedMarkers(frame, this->newCorners,
                this->newIds);
        }

        return frame;
    }

    this->newIds.clear();
    return frame;
}

std::vector<int> Detector::getIds()
{
    return this->newIds;
}

std::vector<std::vector<cv::Point2f>> Detector::getCorners()
{
    return this->newCorners;
}

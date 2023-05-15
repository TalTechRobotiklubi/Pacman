#pragma once

/* LIBRARY INCLUDES ---------------------------------------------------------*/
#include <vector>
#include <opencv2/core/types.hpp>

/* CUSTOM INCLUDES ----------------------------------------------------------*/

/* CLASSES ------------------------------------------------------------------*/
class Line
{
    public:
        Line(cv::Point2f p1, cv::Point2f p2);
        /*int intersects(Line l);*/
        int intersects(Line l, std::pair<int, int> nodeIndex);
        int lineSegmentTouchesOrCrosses(Line l);
        int hasPoint(cv::Point2f p);
        int isPointRightOfLine(cv::Point2f p);
        int boundingBoxIntersects(std::vector<cv::Point2f> boundingBox);
        
        void setFirstPoint(cv::Point2f p1), setSecondPoint(cv::Point2f p2);
        cv::Point2f getFirstPoint(), getSecondPoint();
        std::vector<cv::Point2f> getBoundingBox();

    protected:
        void calcBoundingBox();

        cv::Point2f p1, p2;
        std::vector<cv::Point2f> boundingBox;
};

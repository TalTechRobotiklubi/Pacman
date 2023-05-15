#pragma once

/* LIBRARY INCLUDES ---------------------------------------------------------*/
#include <opencv2/core/types.hpp>
#include <utility>
#include <vector>

/* CUSTOM INCLUDES ----------------------------------------------------------*/
#include "Line.hpp"

/* CLASSES ------------------------------------------------------------------*/
class Node
{
    public:
        Node(int x1, int y1, int x2, int y2, std::pair<int, int> index);
        int tempCheckAruco(std::vector<int> arucoIds,
                std::vector<std::vector<cv::Point2f>> arucoCorners);
        void checkAruco(std::vector<int> arucoIds,
                     std::vector<std::vector<cv::Point2f>> arucoCorners);
        void checkWall(Line wallLine);
        std::vector<cv::Point2f> getCorners();
        std::vector<Line> getLines();
        cv::Point2f getCenter();
        std::pair<int, int> getIndex();
        
        std::pair<int, int> parentIndex;
        unsigned int clearance = 1;
        int hasWall = 0, arucoId = -1, g = 0, h = 0, f = 0;

    protected:
        std::vector<cv::Point2f> corners;
        std::vector<Line> lines;
        cv::Point2f center;
        std::pair<int, int> index;
};

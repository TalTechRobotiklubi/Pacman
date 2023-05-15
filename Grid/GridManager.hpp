#pragma once

/* LIBRARY INCLUDES ---------------------------------------------------------*/
#include <utility>
#include <vector>
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"

/* CUSTOM INCLUDES ----------------------------------------------------------*/
#include "Node.hpp"
#include "../Camera/Camera.hpp"

/* CLASSES ------------------------------------------------------------------*/
class GridManager
{
    public:
        std::vector<std::vector<Node>> createGrid(frame_t *frame);
        std::vector<std::vector<Node>> detectWalls(frame_t *frame,
                std::vector<std::vector<Node>> grid, int drawLines);
        std::vector<std::vector<Node>> addClearance(
                std::vector<std::vector<Node>> grid);
        std::vector<std::vector<Node>> fastCheckArucos(
                std::vector<std::vector<Node>> grid, std::vector<int> arucoIds, 
                std::vector<std::vector<cv::Point2f>> arucoCorners);
        std::vector<std::vector<Node>> checkArucos(
                std::vector<std::vector<Node>> grid, std::vector<int> arucoIds, 
                std::vector<std::vector<cv::Point2f>> arucoCorners);

        int gridColumnCount = 0, gridRowCount = 0;
};

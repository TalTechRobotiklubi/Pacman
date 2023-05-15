#pragma once

/* LIBRARY INCLUDES ---------------------------------------------------------*/
#include <algorithm>
#include <vector>

/* CUSTOM INCLUDES ----------------------------------------------------------*/
#include "Node.hpp"

/* CONSTANTS ----------------------------------------------------------------*/
const std::vector<std::pair<int, int>> STEPS = {
    std::pair<int, int>(-1, -1),
    std::pair<int, int>(0, -1),
    std::pair<int, int>(1, -1),
    std::pair<int, int>(-1, 0),
    std::pair<int, int>(1, 0),
    std::pair<int, int>(-1, 1),
    std::pair<int, int>(0, 1),
    std::pair<int, int>(1, 1)
};
/* CLASSES ------------------------------------------------------------------*/
class PathFinder
{
    public:
        std::vector<Node> astar(std::vector<std::vector<Node>> grid,
                std::pair<int, int> startIndex,std::pair<int, int> targetIndex,
                int isArucoObstacle, int currentId, int targetId,
                std::vector<int> arucoIds,
                std::vector<std::vector<cv::Point2f>> arucoCorners);
        int findNodeInVec(Node *n, std::vector<Node*> vec);

        unsigned int clearanceLevel = 1;
        float PX_TO_CM = 0.f;
        
    protected:
};

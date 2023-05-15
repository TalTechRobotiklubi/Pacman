#pragma once

/* LIBRARY INCLUDES ---------------------------------------------------------*/
#include <iostream>
#include <map>
#include <set>

/* CUSTOM INCLUDES ----------------------------------------------------------*/
#include "../config.hpp"
#include "../Robot/Robot.hpp"
#include "../Misc/Time.hpp"

/* STRUCTS ------------------------------------------------------------------*/
typedef struct rob_manager_result_struct{
    std::map<int, Robot> detectedRobots = {};
    std::map<int, Robot> removedRobots = {};
} rob_manager_result_t;

/* CLASSES ------------------------------------------------------------------*/
class RobotManager
{
    public:
        RobotManager();
        void manageRobots();
        rob_manager_result_t update(std::vector<int>newIds, 
                std::vector<std::vector<cv::Point2f>> newCorners);

    protected:
        std::vector<int> newIds;
        std::vector<std::vector<cv::Point2f>> newCorners;
        std::map<int, Robot> detectedRobots;
        std::map<int, Robot> removedRobots;
};

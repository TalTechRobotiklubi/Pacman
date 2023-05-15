#pragma once

/* LIBRARY INCLUDES ---------------------------------------------------------*/
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <utility>

/* CUSTOM INCLUDES ----------------------------------------------------------*/
#include "../config.hpp"
#include "../Robot/Robot.hpp"

/* CLASSES ------------------------------------------------------------------*/
class CommandGenerator
{
    public:
        CommandGenerator();
        std::string generate(const int id, const int cmdType,
                const std::vector<int> data);
        std::map<int, Robot> getChaseCommands(
         std::map<std::string, std::map<int, Robot>> cameraResult);
        int findChecksum(const char *cmd);
};

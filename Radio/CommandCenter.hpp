/* LIBRARY INCLUDES ---------------------------------------------------------*/
#include <cstdlib>
#include <cmath>
#include <iostream>
#include "TimeoutSerial.hpp"
#include <utility>

/* CUSTOM INCLUDES ----------------------------------------------------------*/
#include "../config.hpp"
#include "CommandGenerator.hpp"
#include "../Misc/Time.hpp"

/* STRUCTS ------------------------------------------------------------------*/
typedef struct cmd_struct{
    std::string cmd;
    int cmdType;
    cv::Point2f endPoint;
    float endAngle;
    std::pair<int, int> motorPowers;
    unsigned long endTimestamp;
} cmd_t;

/* CLASSES ------------------------------------------------------------------*/
class CommandCenter : protected TimeoutSerial
{
    public:
        CommandCenter(const std::string deviceName,
                const unsigned int baudRate) :
                        TimeoutSerial(deviceName, baudRate){};
        void sendCmds(std::map<int, Robot> robotsWithCmd,
                std::string playerCmd);
        void stopAll();
        int isCmdEligible(cmd_t *activeCmd, Robot robot, int newCmdType);
        void updateActiveCmd(cmd_t *activeCmd, Robot robot, int newCmdType);
        int getCmdType(std::string cmd);
        void log(std::string msg);
        void closeSerial();

    protected:
        std::string lastPlayerCmd = "";
        std::map<int, cmd_t> activeCmds;
        unsigned long lastSendTime = 0;
        unsigned long lastLogTime = 0;
        int stopAllCounter = 0;
};

#pragma once

/* LIBRARY INCLUDES ---------------------------------------------------------*/
#include <map>
#include <mutex>

/* CUSTOM INCLUDES ----------------------------------------------------------*/
#include "CameraThread.hpp"
#include "InputThread.hpp"
#include "Thread.hpp"
#include "../Robot/Robot.hpp"
#include "../Radio/CommandGenerator.hpp"
#include "../Radio/CommandCenter.hpp"

/* STRUCTS ------------------------------------------------------------------*/
typedef struct radio_msg_struct{
    std::map<int, Robot> robotsWithCmd = {};
    std::string playerCmd = "";
    unsigned long time = 0;
} radio_msg_t;

/* CLASSES ------------------------------------------------------------------*/
class RadioThread : public Thread
{
    public:
        RadioThread(const std::string threadName,
            const std::string deviceName, const unsigned int baudRate);
        ~RadioThread();
        void setMsg(radio_msg_t newMsg);
         
    private:
        void run() override;
        void close() override;
        
        CommandCenter *cmdCenter;
        radio_msg_t msg;
        std::mutex mutex;
        unsigned long lastRadioMsgTime = 0;
};

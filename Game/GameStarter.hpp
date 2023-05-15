#pragma once

/* LIBRARY INCLUDES ---------------------------------------------------------*/
#include <string>

/* CUSTOM INCLUDES ----------------------------------------------------------*/
#include "Games/ChaseGame.hpp"
#include "Games/PacmanGame.hpp"
#include "../Threads/Thread.hpp"
#include "../Threads/CameraThread.hpp"
#include "../Threads/InputThread.hpp"
#include "../Threads/RadioThread.hpp"

/* CLASSES ------------------------------------------------------------------*/
class GameStarter
{
    public:
        GameStarter(const std::string cameraSource, const int cameraApi,
             const std::string radioSource, const int baudRate,
             const std::string windowTitle);
        void init();
        void start();
        void stop();
        
    protected:
        void startThreads();
        void stopThreads();
        void calcPxToCmConst();

        CameraThread *cameraThread;
        InputThread *inputThread;
        RadioThread *radioThread;
        std::vector<Thread*> threads;
};

#pragma once

/* LIBRARY INCLUDES ---------------------------------------------------------*/
#include <atomic>
#include <string>
#include <thread>

/* CLASSES ------------------------------------------------------------------*/
class Thread
{
    public:
        Thread(const std::string threadName);
        void start();
        void stop();
        int isRunning();
        std::string getThreadName();
        
    protected:
        virtual void run() = 0;
        virtual void close() = 0;
        
        std::thread thread;
        std::string threadName;
        std::atomic<int> running;
};

#pragma once

/* LIBRARY INCLUDES ---------------------------------------------------------*/
#include <chrono>
#include <ctime>

using durationTemplate = std::chrono::duration<unsigned long, std::milli>;

/* CLASSES ------------------------------------------------------------------*/
class Time{
    private:
        static std::chrono::steady_clock::time_point start;
    public:
        static unsigned long time();
        static std::time_t epoch();
};

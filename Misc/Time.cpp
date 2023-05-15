/* CUSTOM INCLUDES ----------------------------------------------------------*/
#include "Time.hpp"

/* STATIC VARIABLES ---------------------------------------------------------*/
std::chrono::steady_clock::time_point Time::start = 
    std::chrono::steady_clock::now();

/* METHODS ------------------------------------------------------------------*/
/**
 * Get time in milliseconds since the start of the program
 *
 * Returns: unsigned long, Time in milliseconds since the start of the program
 */
unsigned long Time::time()
{
    auto now = std::chrono::steady_clock::now();
    auto diff = now - Time::start;
    return std::chrono::duration_cast<durationTemplate> (diff).count();
}

std::time_t Time::epoch()
{
    return std::time(nullptr);
}

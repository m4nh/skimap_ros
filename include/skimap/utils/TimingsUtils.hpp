#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <iomanip>
#include <cstdint>
#include <queue>
#include <chrono>

/**
 */
struct Timings
{
    typedef std::chrono::high_resolution_clock Time;
    typedef std::chrono::milliseconds ms;
    typedef std::chrono::microseconds us;
    typedef std::chrono::duration<float> fsec;

    std::map<std::string, std::chrono::time_point<std::chrono::system_clock>> times;

    void startTimer(std::string name)
    {
        times[name] = Time::now(); //IS NOT ROS TIME!
    }

    us elapsedMicroseconds(std::string name)
    {
        fsec elaps = Time::now() - times[name];
        return std::chrono::duration_cast<us>(elaps);
    }

    ms elapsedMilliseconds(std::string name)
    {
        fsec elaps = Time::now() - times[name];
        return std::chrono::duration_cast<ms>(elaps);
    }

    void printTime(std::string name)
    {
        printf("Time for %s: %f ms", name.c_str(), float(elapsedMicroseconds(name).count()) / 1000.0f);
    }
} DEBUG_TIMINGS;
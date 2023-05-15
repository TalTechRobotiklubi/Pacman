#pragma once

/* LIBRARY INCLUDES ---------------------------------------------------------*/
#include <iostream>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>

/* CUSTOM INCLUDES ----------------------------------------------------------*/
#include "../config.hpp"
#include "../Misc/Time.hpp"

/* STRUCTS ------------------------------------------------------------------*/
typedef struct frame_struct{
    cv::Mat mat;
    unsigned long time;
} frame_t;

/* CLASSES ------------------------------------------------------------------*/
class xiAPIplusCameraOcv;

class Camera
{
    public:
        Camera(const std::string source, const int apiPreference);
        ~Camera();
        frame_t getFrame();
        void close();
        void showFrame(frame_t *frame, const std::string fallback);
    protected:
        /* cv::VideoCapture *cap; */
        xiAPIplusCameraOcv *cap;
};

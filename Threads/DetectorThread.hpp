#pragma once

/* LIBRARY INCLUDES ---------------------------------------------------------*/
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include <map>
#include <queue>

/* CUSTOM INCLUDES ----------------------------------------------------------*/
#include "Thread.hpp"
#include "../Camera/Camera.hpp"
#include "../Camera/Detector.hpp"

/* STRUCTS ------------------------------------------------------------------*/
typedef struct detector_result_struct{
    frame_t frame;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
} detector_result_t;

typedef struct detector_msg_box_struct{
    std::queue<detector_result_t> msgs;
    std::mutex mutex;
} detector_msg_box_t;

/* CLASSES ------------------------------------------------------------------*/
class DetectorThread : public Thread
{
    public:
        DetectorThread(const std::string threadName, 
                detector_msg_box_t *msgBox);
        void setFrame(frame_t *frame);
        int isFrameDetected();
        void writeToMsgBox();

    private:
        void run() override;
        void close() override;
        
        Detector detector;
        detector_msg_box_t *msgBox;
        detector_result_t result;
        frame_t frame;
        std::mutex frameMutex;
        std::mutex resultMutex;
        std::atomic<int> frameDetected = {1};
};

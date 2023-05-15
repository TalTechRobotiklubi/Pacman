#pragma once

/* LIBRARY INCLUDES ---------------------------------------------------------*/
#include <map>
#include <mutex>
#include <opencv2/core/mat.hpp>

/* CUSTOM INCLUDES ----------------------------------------------------------*/
#include "DetectorThread.hpp"
#include "Thread.hpp"
#include "../Camera/Camera.hpp"
#include "../Camera/Detector.hpp"
#include "../config.hpp"
#include "../Robot/Robot.hpp"

/* STRUCTS ------------------------------------------------------------------*/
typedef struct camera_result_struct{
    frame_t frame;
    std::vector<int> arucoIds;
    std::vector<std::vector<cv::Point2f>> arucoCorners;
} camera_result_t;

/* CLASSES ------------------------------------------------------------------*/
class CameraThread : public Thread
{
    public:
        CameraThread(const std::string threadName,
                     const std::string cameraSource,
                     const int cameraApiPreference);
        ~CameraThread();
        camera_result_t getResult();

    private:
        void run() override;
        void close() override;

        Camera *camera;
        detector_msg_box_t detectorMsgBox;
        int detectorThreadCounter = 0;
        camera_result_t result;
        std::mutex resultMutex, cameraMutex;
        std::vector<DetectorThread*> detectorThreads;
        unsigned long lastDetectorInputTime = 0;
        unsigned long lastFrameTimestamp = 0;
};

#pragma once

/* LIBRARY INCLUDES ---------------------------------------------------------*/
#include <cmath>
#include <opencv2/core.hpp>
#include <utility>

/* CUSTOM INCLUDES ----------------------------------------------------------*/
#include "../config.hpp"
#define M_PI 3.14159265358979323846

/* CLASSES ------------------------------------------------------------------*/
class Robot
{
    public:
        int undetectedCount;
        int maxPwr;
        float pMultPID, dMultPID;
        std::string cmd;

        Robot(const int id, const std::vector<cv::Point2f> corners,
                const int maxPwr, const float pMultPID, const float dMultPID);
        void update(const std::vector<cv::Point2f> corners);
        void updateTarget(const cv::Point2f target);
        
        int getId();
        std::vector<cv::Point2f> getArucoCorners();
        cv::Point2f getCenter();
        cv::Point2f getMid();
        cv::Point2f getTarget();
        cv::Vec2f getDirVec();
        cv::Vec2f getTargetVec();
        float getDirVecLen();
        float getTargetVecLen();
        float getAngle();
        float getTargetAngle();
        std::pair<int, int> getMotorPowers();

    protected:
        int id;
        std::vector<cv::Point2f> arucoCorners;
        cv::Point2f target = {-1.f, -1.f};
        cv::Point2f center, mid;
        cv::Vec2f dirVec, targetVec;
        float dirVecLen, targetVecLen;
        float angle=0.f, targetAngle=0.f, lastTargetAngle=0.f;
        std::pair<int, int> motorPowers;
        
        float calcAngle();
        float calcTargetAngle();
        std::pair<int, int> calcMotorPowers();
};

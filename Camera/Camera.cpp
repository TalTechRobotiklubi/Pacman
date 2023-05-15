/* LIBRARY INCLUDES ---------------------------------------------------------*/
/*
 * When using ximea cam, this include must be here, otherwise it won't work
 */ 
#include "xiApiPlusOcv.hpp" 
/* CUSTOM INCLUDES ----------------------------------------------------------*/
#include "Camera.hpp"

/* METHODS ------------------------------------------------------------------*/

/* Construct a new Camera object. The camera uses OpenCV VideoCapture or
 * xiAPIplusCameraOcv object and ArUco codes for the detection process.
 * 
 * Parameters:
 *      source - std::string, Media (image, video, stream etc.) path
 *      apiPreference - int, API prefernece for the media. See https://docs.
 *                      opencv.org/3.4/d4/d15/group__videoio__flags__base.
 *                      html#ga023786be1ee68a9105bf2e48c700294d for more info
 * 
 * Class variable:
 *      cap - cv::VideoCaptue/xiAPIplusCameraOcv, protected, video capture 
 *            class for reading data from images/videos/streams etc.
 * 
 */
Camera::Camera(const std::string source, const int apiPreference)
{
    /* OpenCV */
    /*this->cap = new cv::VideoCapture(source, apiPreference);*/
   
    /* Ximea */ 
    try{
        this->cap = new xiAPIplusCameraOcv();
        this->cap->OpenFirst();
        this->cap->SetExposureTime(16000); //10000 us = 10 ms
        this->cap->StartAcquisition();
    }catch(xiAPIplus_Exception& exp){
        exp.PrintError();
        exit(1);
    }
}

/**
 * Get single frame from the camera. NOTE: The frame is converted to grayscale
 * for better detection. You can use this frame in the detect method to detect
 * the ArUco codes from the frame.
 *
 * Returns: frame_t, image matrix (Type is declared in Camera.hpp)
 */
frame_t Camera::getFrame()
{
    frame_t frame;

    /* OpenCV */
    /* bool hasRead = this->cap->read(frame.mat); */

    /* Ximea */
    frame.mat = this->cap->GetNextImageOcvMat();

    if(!frame.mat.empty()){
        /* cv::cvtColor(frame.mat, frame.mat, cv::COLOR_BGR2GRAY); */
        cv::resize(frame.mat, frame.mat, frame.mat.size() / 2, 0, 0,
                cv::INTER_LINEAR);
    }/*if(!frame.empty()){
        //cv::threshold(frame, frame, 165, 255, 0);
        //cv::threshold(frame, frame, 165, 255, 3);
    }*/

    frame.time = Time::time();
    return frame;
}

Camera::~Camera()
{
    delete this->cap;
}

/**
 * Close the video capture/camera. Should be called out when we do not need the
 * camera anymore (i.e. at the end of the game, program exit).
 */
void Camera::close()
{
    /*this->cap->release();*/

    this->cap->StopAcquisition();
    this->cap->Close();
}

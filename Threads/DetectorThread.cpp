/* CUSTOM INCLUDES ----------------------------------------------------------*/
#include "DetectorThread.hpp"

/* METHODS ------------------------------------------------------------------*/
/**
 * Create new detector thread instance
 *
 * Parameters:
 *      threadName - std::string, Name for the thread
 *      msgBox - detector_msg_box*, Message box for communicating with other
 *               threads (all detector threads should have the same message
 *               box). See DetectorThread.hpp for more specific details of
 *               the detector_msg_box and detector_msg.
 * 
 * Info about the class variables:
 *      detector - Detector, private, The detector that this thread is using
 *                 for the actual work.
 *      msgBox - detector_msg_box*, private, Message box for communicating with
 *               other threads.
 *      result - detector_msg, private, Current result for the thread (based on
 *               the current frame). This is will be pushed to the msgBox when
 *               the detection (for the current frame) is completed.
 *      frame - frame_t, private, Current frame that will be used for detection
 *              and on which the result is based off. Can be set using the
 *              setFrame() method.
 *      frameMutex - std::mutex, private, Mutex for protecting the frame
 *                   variable (as this could potentially be accessed from
 *                   multiple threads at once).
 *      resultMutex - std::mutex, private, Mutex for protecting the result
 *                    variable (as this could potentially be accessed from
 *                    multiple threads at once).
 *      frameDetected - std::atomic<int>, private, Variable for indicating if
 *                      the current frame's ArUcos has been detected or not.
 */
DetectorThread::DetectorThread(const std::string threadName,
        detector_msg_box_t *msgBox):Thread(threadName)
{
    this->msgBox = msgBox;
}

/**
 * Actual implementation of the detector thread. This will detect ArUcos on the
 * given frame (this->frame). See Thread.cpp for more information on the run()
 * method.
 */
void DetectorThread::run()
{
    while(this->running){
        this->frameMutex.lock();

        if (this->frame.mat.empty() || this->frameDetected){
            this->frameMutex.unlock();
            continue;
        }

        detector.detectArucos(this->frame.mat, 1);
        this->frameMutex.unlock();
        
        this->frameDetected = 1;

        this->resultMutex.lock();
        this->result.frame.mat = this->frame.mat.clone();
        this->result.frame.time = this->frame.time;
        this->result.ids = detector.getIds();
        this->result.corners = detector.getCorners();
        this->resultMutex.unlock();

        this->writeToMsgBox();
    }
}

/**
 * See Thread.cpp for information about the close() method.
 */
void DetectorThread::close(){}

/**
 * Set new frame for detection.
 *
 * Parameters:
 *      frame - frame_t*, The frame struct (OpenCV frame with timestamp). See
 *              Camera.cpp and Camera.hpp for more information.
 */
void DetectorThread::setFrame(frame_t *frame)
{
    this->frameMutex.lock();
    this->frame.mat = frame->mat.clone();
    this->frame.time = frame->time;
    this->frameDetected = 0;
    this->frameMutex.unlock();
}

/**
 * Check if the current (this->frame) has been detected or not.
 *
 * Returns: int, 0 if the frame has not been detected
 *               1 if the frame has been detected
 */
int DetectorThread::isFrameDetected()
{
    return this->frameDetected;
}

/**
 * Write a message/result to the message box
 */
void DetectorThread::writeToMsgBox()
{
    this->msgBox->mutex.lock();
    this->resultMutex.lock();
    this->msgBox->msgs.push(this->result);
    this->resultMutex.unlock();
    this->msgBox->mutex.unlock();
}

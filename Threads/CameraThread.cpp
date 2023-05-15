/* CUSTOM INCLUDES ----------------------------------------------------------*/
#include "CameraThread.hpp"

/* METHODS ------------------------------------------------------------------*/
/**
 * Create a new camera thread.
 *
 * Parameters:
 *      threadName - std::string, Name for thread
 *      cameraSource - std::string, Path to the camera device (used only with
 *                     OpenCV and on linux)
 *      cameraApiPreference - int, Camera's API preference (used only with
 *                            OpenCV and on linux - see Camera.cpp for more
 *                            information)
 * 
 * Info about the class variables:
 *      camera - Camera*, private, Pointer to the camera instance that this
 *               thread is using to get pictures from the physical camera
 *               (initialized automatically in the constructor)
 *      cameraMutex - 
 *      detectorMsgBox - detector_msg_box, private, Message box for
 *                       communicating with the detector threads (see
 *                       DetectorThread.cpp for more details)
 *      detectorThreadCounter - Counter for iterating through the detector
 *                              threads unblockingly.
 *      robotManager - RobotManager*, private, Pointer to the robot manager
 *                     instance (initialized automatically in the constructor)
 *      result - camera_result, private, The camera thread's latest result.
 *               Use CameraThread::getResult() to get it.
 *      mutex - std::mutex, private, Mutex for protecting the result variable
 *              as the result can potentially be accessed from multiple threads
 *              at once.
 *      detectorThreads - std::vector<DetectorThread*>, private, The vector of
 *                        detector threads that the camera thread can use. It
 *                        is initialized automatically in the constructor and
 *                        the detector thread count is dependent on the 
 *                        DETECT_THREAD_NUM (see config.hpp for more details)
 *      lastDetectorInputTime - unsigned long, private, Timestamp of when we
 *                              inputted a frame to one of the detector threads
 *                              (used for controlling the internal FPS; see
 *                              CameraThread::run())
 *      lastFrameTimestamp - unsigned long, private, The last frame timestamp
 *                           that the result was based off (used for filtering
 *                           out old detected frames that come from the
 *                           detector threads; see CameraThread::run())
 */
CameraThread::CameraThread(const std::string threadName,
        const std::string cameraSource, const int cameraApiPreference)
    : Thread(threadName)
{
    this->camera = new Camera(cameraSource, cameraApiPreference);

    for(int i = 0; i < DETECT_THREAD_NUM; i++){
        this->detectorThreads.push_back(
            new DetectorThread("Detector Thread #" + std::to_string(i),
                &this->detectorMsgBox)
        );
    }

    for(DetectorThread *detectorThread : detectorThreads){
        detectorThread->start();
    }
}

/**
 * Destructor for the camera thread. Releases dynamically allocated memory.
 */
CameraThread::~CameraThread()
{
    delete this->camera;
    for(DetectorThread *detectorThread : detectorThreads){
        delete detectorThread;
    }
}

/**
 * Actual implementation of the camera thread. This will get frames from the
 * camera and detect ArUcos using detector threads. See Thread.cpp for more
 * information on the run() method.
 */
void CameraThread::run()
{
    while(this->running){
        if(this->detectorThreadCounter >= this->detectorThreads.size()){
            this->detectorThreadCounter = 0;
        }
        
        /* Set a new frame to the current detector thread if possible */
        if((Time::time() - this->lastDetectorInputTime) >=DETECT_FRAME_DELAY || 
             this->lastDetectorInputTime == 0){
            DetectorThread *detectorThread =
                this->detectorThreads[this->detectorThreadCounter];

            if(detectorThread->isFrameDetected()){
                this->cameraMutex.lock();
                frame_t currentFrame = this->camera->getFrame();
                this->cameraMutex.unlock();
                detectorThread->setFrame(&currentFrame);
                this->lastDetectorInputTime = Time::time();
            }

            this->detectorThreadCounter++;
        }
        
        /* Check if there is any new messages in the message box */
        this->detectorMsgBox.mutex.lock();
        if(this->detectorMsgBox.msgs.empty()){
            this->detectorMsgBox.mutex.unlock();
            continue;
        }
        
        /* Get the message from message box */ 
        detector_result_t detectorMsg = this->detectorMsgBox.msgs.front();
        this->detectorMsgBox.msgs.pop();
        this->detectorMsgBox.mutex.unlock();
        
        /* Filter out old (detected) frames */
        if(detectorMsg.frame.time < this->lastFrameTimestamp && 
                this->lastFrameTimestamp != 0){
            continue;
        }
        this->lastFrameTimestamp = detectorMsg.frame.time;
        
        /* Logging */
        /* this->showFrame(&detectorMsg.frame, "../res/empty-frame.png"); */
        
    
        /* Camera thread's result */
        this->resultMutex.lock();
        this->result.frame = detectorMsg.frame;
        this->result.arucoIds = detectorMsg.ids;
        this->result.arucoCorners = detectorMsg.corners;
        this->resultMutex.unlock();
    }
}

/**
 * Clean the camera thread (stop the detector threads) before the thread is
 * joined to the main thread. See also Thread.cpp stop() method.
 */
void CameraThread::close()
{
    for(DetectorThread *detectorThread : detectorThreads){
        detectorThread->stop(); 
    }
}

/**
 * Get latest result from the camera thread.
 *
 * Returns: camera_result, The result from the RobotManager with
 *          corresponding frame time (see RobotManager.cpp for more details) 
 */
camera_result_t CameraThread::getResult()
{
    this->resultMutex.lock();
    camera_result_t returnValue = this->result;
    this->resultMutex.unlock();
    return returnValue;
}

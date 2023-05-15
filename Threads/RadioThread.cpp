/* CUSTOM INCLUDES ----------------------------------------------------------*/
#include "RadioThread.hpp"

/* METHODS ------------------------------------------------------------------*/
/**
 * Create a new radio thread instance
 *
 * Parameters:
 *      threadName - std::string, Name for the thread
 *      cameraThread - CameraThread*, Pointer to the camera thread
 *      inputThread - InputThread*, Pointer to the input thread
 *      deviceName - std::string, Serial device name/path/port
 *      baudRate - unsigned int, Baud rate for the serial device
 *
 * Info about the class variables:
 *      cmdCenter - CommandCenter*, private, Pointer to CommandCenter instance
 *                  that radio thread will use to send the generated commands
 *      lastRadioMsgTime - 
 *      msg - 
 *      mutex - 
 *
 */
RadioThread::RadioThread(const std::string threadName,
        const std::string deviceName, const unsigned int baudRate) : 
    Thread(threadName)
{
    this->cmdCenter = new CommandCenter(deviceName, baudRate);
}

/**
 * Destructor for the radio thread. Releases dynamically allocated memory.
 */
RadioThread::~RadioThread()
{
    delete cmdCenter;
}

/**
 * Actual implementation of the radio thread. This will send commands to the
 * robots based on the provided message from outside the thread. See
 * RadioThread::setMsg for more information on the message part and see
 * Thread.cpp for more information on the run() method.
 */
void RadioThread::run()
{
    while(this->running){
        this->mutex.lock();
        if(this->msg.time <= this->lastRadioMsgTime){
            this->mutex.unlock();
            continue;
        }
        this->lastRadioMsgTime = this->msg.time;

        this->cmdCenter->sendCmds(this->msg.robotsWithCmd,this->msg.playerCmd);
        this->mutex.unlock();
    }
}

void RadioThread::setMsg(radio_msg_t newMsg)
{
    this->mutex.lock();
    this->msg.robotsWithCmd = newMsg.robotsWithCmd;
    this->msg.playerCmd = newMsg.playerCmd;
    this->msg.time = newMsg.time;
    this->mutex.unlock();    
}

/**
 * Clean the radio thread (close the radio serial) before the thread is joined
 * to the main thread. See also Thread.cpp stop() method.
 */
void RadioThread::close()
{
    this->cmdCenter->closeSerial();
}

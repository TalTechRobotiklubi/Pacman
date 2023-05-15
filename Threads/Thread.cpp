/* CUSTOM INCLUDES ----------------------------------------------------------*/
#include "Thread.hpp"

/* METHODS ------------------------------------------------------------------*/
/**
 * Creates a new thread object that can be started with the Thread::start()
 * method.
 *
 * NOTE: To start and stop the created thread, use the Thread::start() and 
 *       Thread::stop() methods.
 *
 * Parameters: threadName - std::string, Name for the thread
 *
 * Info about the class variables and virtual methods:
 *      thread - std::thread, protected, Actual thread that runs the
 *               Thread::run() method (see below)
 *      threadName - std::string, protected, The thread name
 *      running - std::atomic<int>, protected, Atomic int that indicates
 *                whether the thread is running or not (0 for not running, 1 
 *                for running)
 *      run() - virtual void, protected, The method that the std::thread runs
 *              to create an actual separate CPU thread (put the thread logic/
 *              implementation in this method). Must be overriden by a child
 *              class.
 *      close() - virtual void, protected, The method that can be used to clean
 *                the Thread object before the std::thread is joined to the
 *                main thread. Is automatically called out by the stop method -
 *                so no need to call it out yourself. Must be overriden by a
 *                child class.
 *
 * NOTE: This class has two virtual methods - Thread::run() and
 *       Thread::close(). As they are virtual, they must be overriden and thus
 *       there is no point in creating a simple Thread object - the Thread 
 *       class should act only as a base class for other child classes.
 * NOTE: To create a thread that is running as long as the program executes use
 *       the Thread::running variable in child classes. For example:
 *          ChildThread::run()
 *          {
 *              while(this->running){
 *                  // Code
 *              }
 *          }
 *       To stop the thread just call the stop() method from the child
 *       instance.
 */
Thread::Thread(const std::string threadName)
{
    this->threadName = threadName;
    this->running = 0;
}

/**
 * Start the thread. Creates and starts a std::thread using the virtual
 * Thread::run() method and sets the thread object into running state
 * (this->running == 1)
 */
void Thread::start()
{
    this->running = 1;
    this->thread = std::thread([this](){
        this->run();
    });
}

/**
 * Stop the thread (this->running == 0). Calls out the virtual
 * Thread::close() method that can be used for cleaning the thread up. The
 * std::thread will be joined.
 */
void Thread::stop()
{
    this->running = 0;
    this->close();
    this->thread.join();
}

/**
 * Get the thread state (check if the thread is running or not)
 *
 * Returns: int, 0 when the thread is not running
 *               1 when thread is running
 */
int Thread::isRunning()
{
    return this->running;
}

/**
 * Get thread name
 *
 * Returns: std::string, Thread name
 */
std::string Thread::getThreadName()
{
    return this->threadName;
}

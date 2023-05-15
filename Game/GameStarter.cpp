/* CUSTOM INCLUDES ----------------------------------------------------------*/
#include "GameStarter.hpp"

/* METHODS ------------------------------------------------------------------*/
GameStarter::GameStarter(const std::string cameraSource, const int cameraApi,
        const std::string radioSource, const int baudRate,
        const std::string windowTitle)
{
    this->cameraThread = new CameraThread("Camera Thread", cameraSource,
            cameraApi);
    this->inputThread = new InputThread("Input Thread", windowTitle);
    this->radioThread = new RadioThread("Radio Thread", radioSource, baudRate);
    
    this->threads.push_back(cameraThread);
    this->threads.push_back(inputThread);
    this->threads.push_back(radioThread);
}

void GameStarter::init()
{
    this->startThreads();
}

void GameStarter::start()
{
    /*ChaseGame chaseGame(this->cameraThread, this->inputThread,
            this->radioThread);
    chaseGame.init();
    chaseGame.run();
    chaseGame.close();*/
    PacmanGame pacman(this->cameraThread, this->inputThread,
            this->radioThread);
    pacman.init();
    pacman.run();
    pacman.close();
}

void GameStarter::stop()
{
    this->stopThreads();
}

void GameStarter::startThreads()
{
    for(Thread *thread : this->threads){
        thread->start();
    }
}

void GameStarter::stopThreads()
{
    for(Thread *thread : threads){
        thread->stop();
        delete thread;
    }
}

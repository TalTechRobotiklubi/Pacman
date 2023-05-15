/* CUSTOM INCLUDES ----------------------------------------------------------*/
#include "InputThread.hpp"

/* METHODS ------------------------------------------------------------------*/
/**
 * Create new input thread instance (spawns a SDL window for detecting the
 * inputs)
 *
 * Parameters:
 *      threadName - std::string, Name for the thread
 *      windowName - std::string, Name/title for the SDL window
 *
 * Info about the class variables:
 *      window - SDL_Window*, private, SDL window pointer (initialized by the
 *               SDL_CreateWindow function)
        windowName - std::string, private, SDL window name/title
 *      renderer - SDL_Renderer*, private, SDL renderer pointer (initialized by
 *                 the SDL_CreateRenderer function)
 *      keyboard - Uint8*, private, SDL keyboard state array. This is used to
 *                 get the key presses from the user. As this variables is 
 *                 private, use InputThread::isKeyPressed() method to check
 *                 whether key is pressed or not or InputThread::isKeyReleased()
 *                 for key release.
 *      releaseKeysMap - std::vector<std::string>, private, Vector for 
 *                       memorizing which keys should be watched for key
 *                       release. See InputThread::isKeyReleased() method.
 *      keyboardMutex - std::mutex, private, Mutex for protecting the keyboard
 *                      variable as the keyboard variable could potentially be
 *                      accessed from multiple threads at once
 */
InputThread::InputThread(const std::string threadName,
        const std::string windowName) : Thread(threadName)
{
    this->windowName = windowName;
    SDL_Init(SDL_INIT_VIDEO);
    
    cv::namedWindow("out", cv::WINDOW_NORMAL);
    cv::resizeWindow("out", 800, 600);
}

/**
 * Actual implementation of the input thread. This will create a SDL window for
 * detecting the inputs. See Thread.cpp for more information on the run()
 * method.
 */
void InputThread::run()
{
    this->window = SDL_CreateWindow(this->windowName.c_str(),
            SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 640, 480, 0);
    this->renderer = SDL_CreateRenderer(this->window, -1, 0);
    SDL_SetRenderDrawColor(this->renderer, 255, 255, 255, 255);

    while(this->running){
        /* Update the window */
        SDL_RenderClear(this->renderer);
        SDL_RenderPresent(this->renderer); 
        
        /* Read keyboard inputs with SDL */
        SDL_PumpEvents();
        this->keyboardMutex.lock();
        this->keyboard = SDL_GetKeyboardState(NULL);
        this->keyboardMutex.unlock();
    }
    
    SDL_DestroyRenderer(this->renderer);
    SDL_DestroyWindow(this->window);
    SDL_Quit();
}

/**
 * Check wether a key is pressed or not
 *
 * Parameters: key - std::string, The SDL defined name of the desired key to
 *                   query. See https://wiki.libsdl.org/SDL_Scancode and
 *                   https://wiki.libsdl.org/SDL_GetKeyboardState for more
 *                   information.
 *
 * NOTE: To check whether two or more keys are pressed, you can do something
 *       like this:
 *          if(inputThread.isKeyPressed("A") &&
 *              inputThread.isKeyPressed("B")){
 *                  // code
 *          }
 *
 * Returns: 0, if key is not pressed or is unknown
 *          1, if key is pressed
 */
int InputThread::isKeyPressed(const std::string key)
{
    SDL_Keycode keyCode = SDL_GetKeyFromName(key.c_str());

    if(keyCode == SDLK_UNKNOWN){
        return 0;
    }
    
    SDL_Scancode scanCode = SDL_GetScancodeFromKey(keyCode);
    int returnValue = 0;
    
    this->keyboardMutex.lock();
    if(this->keyboard != NULL && this->keyboard[scanCode]){
        returnValue = 1;
    }
    this->keyboardMutex.unlock();

    return returnValue;
}

/**
 * Check if key is released or not. It uses InputThread::isKeyPressed() and
 * InputThread::releaseKeysMap vector to check whether key is released or not.
 *
 * Returns: 0 if key has not been released
 *          1 if key has been released
 */
int InputThread::isKeyReleased(const std::string key)
{
    auto keyPos = std::find(this->releaseKeysMap.begin(),
            this->releaseKeysMap.end(), key);

    if(this->isKeyPressed(key) && keyPos == this->releaseKeysMap.end()){
        this->releaseKeysMap.push_back(key);
        return 0;
    }
    
    if(!this->isKeyPressed(key) && keyPos != this->releaseKeysMap.end()){
        /* Removing the key from the releaseKeysMap vector as it has been
         * released and we do not want to check it anymore */
        this->releaseKeysMap.erase(keyPos);
        return 1;
    }

    return 0;
}

/**
 * Show the given frame graphically.
 *
 * Parameters:
 *      frame - frame_t(cv::Mat), the frame for displaying
 *      fallback - std::string, The fallback image path/name. The fallback
 *                 image will be displayed if the frame is empty.
 */
void InputThread::showFrame(frame_t *frame, const std::string fallback)
{
    cv::waitKey(1);

    if (!frame->mat.empty()) {
        cv::imshow("out", frame->mat);
        return;
    }

    frame->mat = cv::imread(fallback);
    if (!frame->mat.empty()) {
        cv::imshow("out", frame->mat);
        return;
    }

    std::cout << "WARNING: Empty frame!" << std::endl;
}

/**
 * Clean the input thread (quit SDL) before the thread is joined to the main
 * thread. See also Thread.cpp stop() method.
 */
void InputThread::close()
{
}

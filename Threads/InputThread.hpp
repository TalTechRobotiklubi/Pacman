#pragma once

/* LIBRARY INCLUDES ---------------------------------------------------------*/
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include "SDL2/SDL.h"

/* CUSTOM INCLUDES ----------------------------------------------------------*/
#include "Thread.hpp"
#include "../config.hpp"
#include "../Camera/Camera.hpp"
#include "../Radio/CommandGenerator.hpp"

/* CLASSES ------------------------------------------------------------------*/
class InputThread : public Thread
{
    public:
        InputThread(const std::string threadName,const std::string windowName);
        int isKeyPressed(const std::string key);
        int isKeyReleased(const std::string key);
        void showFrame(frame_t *frame, const std::string fallback);

    private:
        void run() override;
        void close() override;
        
        SDL_Window *window;
        SDL_Renderer *renderer;
        /* Non-const pointer to const Uint8
         * See https://stackoverflow.com/a/16091893 for more info :D */
        const Uint8 *keyboard = NULL;
        std::mutex keyboardMutex;
        std::vector<std::string> releaseKeysMap;
        std::string windowName;
};

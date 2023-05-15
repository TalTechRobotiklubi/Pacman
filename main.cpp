/* LIBRARY INCLUDES ---------------------------------------------------------*/
#include <cstdlib>
#include <iostream>

/* CUSTOM INCLUDES ----------------------------------------------------------*/
#include "Game/GameStarter.hpp"

/* MAIN ---------------------------------------------------------------------*/
int main(int argc, char *argv[])
{
    /* Parse the command line arguments */
    if(argc < 3){
        std::cerr << "USAGE: " << argv[0] << " STREAM/VIDEO/IMAGE " <<
            "SERIAL_DEVICE BAUD_RATE" << std::endl;
        return 1;
    }
    
    char *errPtr; 
    int baudRate = strtol(argv[3], &errPtr, 10);
    if(errPtr[0] != 0){
        std::cerr << "USAGE: " << argv[0] << " STREAM/VIDEO/IMAGE " <<
            "SERIAL_DEVICE BAUD_RATE" << std::endl;
        return 1;
    }

    GameStarter gameStarter(argv[1], 0, argv[2], baudRate, "botswarm");

    gameStarter.init();
    gameStarter.start();
    gameStarter.stop();

    return 0;
}

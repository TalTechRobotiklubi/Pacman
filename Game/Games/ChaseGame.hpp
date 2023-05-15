#pragma once

/* LIBRARY INCLUDES ---------------------------------------------------------*/
#include <map>
#include <string>
#include <chrono>

/* CUSTOM INCLUDES ----------------------------------------------------------*/
#include "../ScoreManager.hpp"
#include "../../config.hpp"
#include "../../Grid/GridManager.hpp"
#include "../../Grid/Node.hpp"
#include "../../Grid/PathFinder.hpp"
#include "../../Misc/UnitConverter.hpp"
#include "../../Radio/CommandGenerator.hpp"
#include "../../Robot/Robot.hpp"
#include "../../Threads/CameraThread.hpp"
#include "../../Threads/InputThread.hpp"
#include "../../Threads/RadioThread.hpp"

/* CLASSES ------------------------------------------------------------------*/
class ChaseGame
{
    public:
        ChaseGame(CameraThread *cameraThread, InputThread *inputThread,
                     RadioThread *radioThread);
        void init();
        void run();
        void close();
        ~ChaseGame();

    protected:
        Robot findTarget(camera_result_t *cameraResult);
        void countScore();
        void handleGameState(Robot *targetRobot,
                camera_result_t *cameraResult);
        std::map<int, Robot> findRobotsInCollisionCourse(
                camera_result_t *cameraResult);
        std::map<int, Robot> genChaseCmds(camera_result_t *cameraResult, 
                std::map<int, Robot> collisionRobots);
        std::string genPlayerCmd();

        void logGameState();
        
        /* CONSTANTS */ 
        /**
         * Target ID
         */
        const int TARGET_ID = 45;
        /**
         * Distance that defines when a NPC robot has caught the player robot
         * (in cm)
         */
        const int CATCH_DISTANCE_CM = 10;
        /**
         * Maximum field of sight for a robot to use a motor set.
         * 
         * A              C
         *  \            /
         *   \          /
         *    \        /
         *     \      /
         *      +----+
         *      |\  /|
         *      | \/ |
         *      | B  |
         *      +----+
         *
         * Angle ABC is the field of sight for a robot and this constant
         * defines the angle size.
         */
        const int MOTORS_ANGLE_THRESH = 40;
        /**
         * How many undetected/empty frames are enabled before we send stop to
         * all robots
         */
        const int MAX_UNDETECTED_ROBOTS = 10;
        /**
         * How many continuous frames are enabled to not detect the target
         * before we send stop to all robots
         */
        const int MAX_UNDETECTED_TARGET = 10;
        
        /**
         * Absolute value for robots motors' maximum power
         */
        const int MAX_PWR = 300;

        /**
         * PID control constants for robots
         */
        const float P_CONST = 2.5f;
        const float D_CONST = 0.25f;

        /**
         * Game states:
         *      0 - init (show start menu)
         *      1 - running (game is being played)
         *      2 - paused
         *      3 - finished (game is over)
         *      4 - restarting
         */
        enum GAME_STATES{
            GAME_INIT,
            GAME_RUN,
            GAME_PAUSE,
            GAME_OVER,
            GAME_RESTART
        };
        
        CameraThread *cameraThread;
        InputThread *inputThread;
        RadioThread *radioThread;
        ScoreManager *scoreManager;
        UnitConverter *unitConverter;
        CommandGenerator *cmdGen;

        std::vector<std::vector<Node>> grid;
        float PX_TO_CM = 0.f;
        unsigned long lastCameraResultTime = 0;
        unsigned long score = 0, gameStart = 0, pauseStart = 0, pauseTime = 0;
        int gameState = 0, prevGameState = 0;
        int undetectedRobotsCounter = 0;
        int undetectedTargetCounter = 0;
};

#pragma once

/* LIBRARY INCLUDES ---------------------------------------------------------*/

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

/* CONSTANTS ----------------------------------------------------------------*/

/* CLASSES ------------------------------------------------------------------*/
class ChaseGame
{
    public:
        ChaseGame(CameraThread *cameraThread, InputThread *inputThread, 
                RadioThread *radioThread);
        ~ChaseGame();
        void init();
        void run();
        void close();

        /**
         * Target ID
         */
        const int TARGET_ID = 1;
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
        const int MOTORS_ANGLE_THRESH = 30;
        
        /**
         * The minimum distance in cm to a node in the robot's path that will
         * be the next target
         */
        const float CLOSEST_NODE_DIST = 5.f;
        
        /**
         * Distance that defines when a NPC robot has caught the player robot
         * (in cm)
         */
        const int CATCH_DISTANCE_CM = 17;
        
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
        
        /**
         * How many continuous frames are enabled to not detect the target
         * before we send stop to all robots
         */
        const int MAX_UNDETECTED_TARGET = 10;
        
        /**
         * How many undetected/empty frames are enabled before we send stop to
         * all robots
         */
        const int MAX_UNDETECTED_ROBOTS = 10;

        /**
         * Absolute value for robots motors' maximum power
         */
        const int MAX_PWR = 200;

        /**
         * PID control constants for robots
         */
        /**
         * constants for speed 150 and turning speed 100:
         * const float P_CONST = 5.f;
         * const float D_CONST = 1.f;
         */
        const float P_CONST = 5.f;
        const float D_CONST = 1.f;
        

    protected:
        CameraThread *cameraThread;
        InputThread *inputThread;
        RadioThread *radioThread;
        ScoreManager *scoreManager;
        GridManager *gridManager;
        UnitConverter *unitConverter;
        CommandGenerator *cmdGen;
        PathFinder *pathFinder;
        
        void logGameState();
        void saveStartPositions();
        void handleGameState(Robot *targetRobot,
                camera_result_t *cameraResult);
        void countScore(Robot *targetRobot);
        void calcPaths(Robot *targetRobot, camera_result_t *cameraResult);
        void calcRestartPaths(camera_result_t *cameraResult);
        void manageRobots(camera_result_t *cameraResult);
        Robot findTarget(camera_result_t *cameraResult);
        std::map<int, Robot> genCmds(camera_result_t *cameraResult);
        std::string genPlayerCmd();
        
        std::vector<std::vector<Node>> grid;
        std::map<int, std::vector<Node>> paths;
        std::map<int, Node> startNodes;
        std::map<int, Robot> robots;
        float PX_TO_CM = 0.f;
        
        unsigned long lastCameraResultTime = 0, lastPathCalcTime = 0;
        unsigned long score = 0, gameStart = 0, pauseStart = 0, pauseTime = 0;
        int gameState = 0, prevGameState = 0;
        int undetectedRobotsCounter = 0;
        int undetectedTargetCounter = 0;
};

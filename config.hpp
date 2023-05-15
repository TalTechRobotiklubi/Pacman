#pragma once

/* LIBRARY INCLUDES ---------------------------------------------------------*/
#include <opencv2/core/matx.hpp>

/* CONSTANTS ----------------------------------------------------------------*/

/**
 * Size (length of one side) of the robot ArUco in centimeters. This variable
 * is used in the Game::calcPxToCmConst for automatic pixels to centimeters
 * conversion.
 */
const float ARUCO_SIZE_CM = 8.f;

/**
 * One node size in the grid. In other words, it is the step (in pixels) that
 * will be used to create the grid.
 */
const int NODE_SIZE = 8;

/**
 * Max score entries in the score file
 */
const int MAX_SCORE_ENTRIES = 1024;

/**
 * The delimiter used in the score file for separating fields
 */
const std::string SCORE_FILE_DELIM = ";";

/**
 * The base vector for calculating robot angles
 */
const cv::Vec2f baseVec(0, 1);
const float baseVecLen = 1.f;

/** 
 */
const int PREAMBLE = 0;

/**
 * Data delimiter/separating character
 */
const char DATA_DELIM = ',';

/**
 * Last command type value. Used for invalid command checking
 */
const int LAST_CMD_TYPE = 3;

/**
 * Command type values
 *
 * MOTORS == motor set
 */
enum cmd_enum{
    CMD_END = 0,
    CMD_DRIVE = 1,
    CMD_TURN = 2,
    CMD_MOTORS = 3
};

/**
 * Maximum execution time (in ms) for turning command.
 * If execution time is exceeded, new command will be sent.
 */
const int TURN_MAX_EXEC_TIME = 2000;

/**
 * Maximum execution time (in ms) for drive distance command.
 * If execution time is exceeded, new command will be sent.
 */
const int DRIVE_MAX_EXEC_TIME = 7500;

/**
 * Delay for logging
 */
const int LOG_DELAY = 80;

/**
 * Delay used for creating internal fps.
 * Blocks getting new frames too frequently.
 */
const int DETECT_FRAME_DELAY = 17;

/**
 * Number of detector threads being used
 */
const int DETECT_THREAD_NUM = 3;

/**
 * Switch on/off camera logging (0 - off, 1 - on)
 */
const int ENABLE_CAMERA_LOGGING = 0;

/**
 * Switch on/off radio logging (0 - off, 1 - on)
 */
const int ENABLE_RADIO_LOGGING = 0;

/**
 * Maximum number of stop all commands
 */
const int MAX_STOP_ALL = 10;

/**
 * Stop all command
 */
const std::string END_ALL_CMD = "0000FF000107E";

/**
 * Buffer end character
 */
const std::string BUFFER_END = "G";

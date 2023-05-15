/* CUSTOM INCLUDES ----------------------------------------------------------*/
#include "CommandCenter.hpp"

/* METHODS ------------------------------------------------------------------*/
/*CommandCenter::CommandCenter(const std::string deviceName,
        const unsigned int baudRate){}*/
/**
 * Send command message to robots.
 * 
 * Parameters:
 *       robotsWithCmd - std::map<int, Robot>, Map where keys are IDs and 
 *           values are robots with commands.
 *       playerCmd - std::string, command message.
 */
void CommandCenter::sendCmds(std::map<int, Robot> robotsWithCmd, 
        std::string playerCmd)
{
    std::string buffer = "";

    if(robotsWithCmd.size() == 0){
        this->activeCmds.clear();
        this->stopAll();
        return;
    }

    if(playerCmd != this->lastPlayerCmd){
        buffer += playerCmd;
        this->lastPlayerCmd = playerCmd;
    }

    for(std::map<int, Robot>::iterator it = robotsWithCmd.begin();
        it != robotsWithCmd.end(); it++){
        
        cmd_t activeCmd; 
        if(this->activeCmds.find(it->first) != this->activeCmds.end()){
            activeCmd = this->activeCmds[it->first];
        }else{
            activeCmd.cmdType = -1;
        }
        
        int newCmdType = this->getCmdType(it->second.cmd);

        if(newCmdType < 0){
            std::cerr << "ERROR: Failed to get the command type!" << std::endl;
            continue;
        }

        if(this->isCmdEligible(&activeCmd, it->second, newCmdType)){
            buffer += it->second.cmd;
        }
    }
    
    if(buffer == ""){
        //this->log("There is no commands to send!");
        return;
    }

    buffer += BUFFER_END;
    
    this->log("Buffer: " + buffer);
    
    this->stopAllCounter = 0;
    this->lastSendTime = Time::time();
    this->writeString(buffer);
}

/**
 * Check if the command is eligible for sending.
 *
 * Parameters:
 *       activeCmd - cmd_t*, currently active command.
 *       robot - Robot, robot that corresponds to the active command.
 *       newCmdType - int, new command type (0 - stop/end; 1 - drive mm; 
 *          2 - turn degrees; 3 - motorset).
 * 
 * Returns: int, 0 - command is not eligible
 *               1 - command is eligible
 */
int CommandCenter::isCmdEligible(cmd_t *activeCmd, Robot robot, int newCmdType)
{
    unsigned long now = Time::time();

    if(newCmdType == 0){
        if(activeCmd->cmdType < 0){
            return 0;
        }else if(activeCmd->cmdType > 0){
            this->updateActiveCmd(activeCmd, robot, newCmdType);
            return 1;
        }else{
            if(activeCmd->endTimestamp >= now){
                return 0;
            }

            float endX = activeCmd->endPoint.x;
            float endY = activeCmd->endPoint.y;
            
            float robotX = robot.getMid().x;
            float robotY = robot.getMid().y;

            if(std::abs(endX - robotX) < 5 && std::abs(endY - robotY) < 5){
                this->activeCmds.erase(robot.getId()); 
                return 0;
            }else{
                this->updateActiveCmd(activeCmd, robot, newCmdType);
                return 1;
            } 
        }
    }else if(newCmdType == 1){
        if(activeCmd->cmdType != 1){
            this->updateActiveCmd(activeCmd, robot, newCmdType);
            return 1;
        }else{
            float endX = activeCmd->endPoint.x;
            float endY = activeCmd->endPoint.y;
            
            float robotX = robot.getCenter().x;
            float robotY = robot.getCenter().y;
            
            if(std::abs(endX - robotX) < 5 && std::abs(endY - robotY) < 5){
                this->updateActiveCmd(activeCmd, robot, newCmdType);
                return 1; 
            }

            if(activeCmd->endTimestamp <= now){
                this->updateActiveCmd(activeCmd, robot, newCmdType);
                return 1;
            }
            
            float newTargetAngle = robot.getTargetAngle();
            if(std::abs(activeCmd->endAngle - newTargetAngle) < 5){
                return 0;
            }else{
                this->updateActiveCmd(activeCmd, robot, newCmdType);
                return 1;
            }
        }
    }else if(newCmdType == 2){
        if(activeCmd->cmdType != 2){
            this->updateActiveCmd(activeCmd, robot, newCmdType);
            return 1;
        }else{
            if(activeCmd->endTimestamp <= now){
                this->updateActiveCmd(activeCmd, robot, newCmdType);
                return 1;
            }

            float newTargetAngle = robot.getTargetAngle();
            if(std::abs(activeCmd->endAngle - newTargetAngle) < 5){
                return 0;
            }else{
                this->updateActiveCmd(activeCmd, robot, newCmdType);
                return 1;
            }
        }
    }else if(newCmdType == 3){
        if(activeCmd->cmdType != 3){
            this->updateActiveCmd(activeCmd, robot, newCmdType);
            return 1;
        }else{
            int leftMotorDelta = std::abs(activeCmd->motorPowers.first - 
                    robot.getMotorPowers().first);
            int rightMotorDelta = std::abs(activeCmd->motorPowers.second - 
                    robot.getMotorPowers().second);

            if(leftMotorDelta > 5 || rightMotorDelta > 5){
                this->updateActiveCmd(activeCmd, robot, newCmdType);
                return 1;
            }else{
                return 0;
            }
        }
    }
    
    return 0;
}

/**
 * Update active/current command.
 *
 * Parameters:
 *       activeCmd - cmd_t*, currently active command
 *       robot - Robot, robot that corresponds to the active command
 *       newCmdType - int, new command type (0 - stop/end; 1 - drive mm; 
 *          2 - turn degrees; 3 - motorset).
 */
void CommandCenter::updateActiveCmd(cmd_t *activeCmd, Robot robot,
        int newCmdType)
{
    activeCmd->cmd = robot.cmd;
    activeCmd->cmdType = newCmdType;
    
    if(newCmdType == 0){
        activeCmd->endPoint = robot.getMid();
        activeCmd->endTimestamp = Time::time() + 500;
    }else if(newCmdType == 1){
        activeCmd->endPoint = robot.getTarget();
        activeCmd->endAngle = robot.getTargetAngle();
        activeCmd->endTimestamp = Time::time() + DRIVE_MAX_EXEC_TIME;
    }else if(newCmdType == 2){
        activeCmd->endAngle = robot.getTargetAngle();
        activeCmd->endTimestamp = Time::time() + TURN_MAX_EXEC_TIME;
    }else if(newCmdType == 3){
        activeCmd->motorPowers = robot.getMotorPowers();
    }else{
        return;
    }
    
    this->activeCmds[robot.getId()] = *activeCmd;
}


/**
 * Send stop to all robots
 */
void CommandCenter::stopAll()
{
    if(stopAllCounter < MAX_STOP_ALL){
        /* The command for sending stop to all robots is 0000FF000107E */
        this->log("Buffer: " + END_ALL_CMD+BUFFER_END);
        this->writeString(END_ALL_CMD+BUFFER_END);
        stopAllCounter++;
    }else if(ENABLE_RADIO_LOGGING){
        //this->log("There is no commands to send!");
    }
}

/**
 * Get command type.
 *
 * Parameters:
 *       cmd - std::string, command message.
 *
 * Returns: int, command type
 *       -1 -> error; 
 *        0 -> stop/end; 
 *        1 -> drive mm; 
 *        2 -> turn degrees; 
 *        3 -> motorset;
 */
int CommandCenter::getCmdType(std::string cmd)
{
    char *errPtr; 
    std::string typeStr = cmd.substr(6, 2);
    int type = strtol(typeStr.c_str(), &errPtr, 16);

    if(errPtr[0] != 0 || type > LAST_CMD_TYPE){
        return -1;
    }

    return type;
}

/**
 * Print radio message/buffer
 *
 * Parameters:
 *       msg - std::string, command message.
 */
void CommandCenter::log(std::string msg)
{
    if(!ENABLE_RADIO_LOGGING){
        return;
    }

    if((Time::time() - this->lastLogTime) <= LOG_DELAY
            && this->lastLogTime != 0){
        return;
    }
    this->lastLogTime = Time::time();

    std::cout << msg << std::endl;
}

/**
 * Close the serial. Should be called out when there is no need to send any
 * commands.
 */
void CommandCenter::closeSerial()
{
    this->close();
}

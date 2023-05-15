/* CUSTOM INCLUDES ----------------------------------------------------------*/
#include "CommandGenerator.hpp"

/* METHODS ------------------------------------------------------------------*/
/**
 * Construct a new CommandGenerator
 */
CommandGenerator::CommandGenerator(){}

/**
 * Generate radio sendable command message.
 * 
 * Parameters:
 *       id - int, Robot/Aruco id.
 *       cmdType - int, command type (0 - stop/end; 1 - drive mm; 
 *          2 - turn degrees; 3 - motorset)
 *       data - std::vector<int>, command info/arguments (e.g. motors values
 *              when generating motorset command)
 * 
 * Returns: std::string(cmd), Radio sendable command message as a string.
 */
std::string CommandGenerator::generate(/*const*/ int id, const int cmdType,
        const std::vector<int> data)
{
    if(PREAMBLE < 0 || PREAMBLE > 255){
        std::cerr << "ERROR: Command generation failed! Illegal preamble!" <<
            std::endl;
        return "";
    }if(id < 0 || id > 255){
        std::cerr << "ERROR: Command generation failed! Illegal robot ID!" <<
            std::endl;
        return "";
    }if(cmdType < 0 || cmdType > 255){
        std::cerr << "ERROR: Command generation failed! Illegal command type!"
            << std::endl;
        return "";
    }
    
    char cmd[512];
    int dataLen = data.size();
    
    /* Adding preamble, id and command type */  
    sprintf(cmd, "%02X%02X%02X%02X", PREAMBLE, PREAMBLE, id, cmdType);
    
    /* Adding data string and its length */
    char dataStr[256] = {0};
    for(int i = 0; i < dataLen; i++){
        if(i > 0){
            sprintf(dataStr, "%s%c", dataStr, DATA_DELIM);
        }

        if(data[i] < 0){
            sprintf(dataStr, "%s-%X", dataStr, -data[i]);
        }else{
            sprintf(dataStr, "%s%X", dataStr, data[i]);
        }
    }

    int dataStrLen = strnlen(dataStr, 256);
    if(dataStrLen < 1 || dataStrLen > 255){
        std::cerr << "ERROR: Msg gen failed! Illegal data string length!" <<
            std::endl;
        return "";
    }
    sprintf(cmd, "%s%02X%s", cmd, dataStrLen, dataStr);

    /* Adding checksum (G for the reading bug) */
    int checksum = this->findChecksum(cmd);
    if(checksum < 0){
        std::cerr << "ERROR: Msg gen failed! Checksum calculation failed!" <<
            std::endl;
        return "";
    }
    
    sprintf(cmd, "%s%02X", cmd, checksum);
    
    return std::string(cmd);
}

/**
 * Calculate the value of checksum.
 * 
 * Parameters:
 *       cmd - const char*, string of the command (without the checksum itself)
 * 
 * Returns: int, checksum value.
 */
int CommandGenerator::findChecksum(const char *cmd)
{
    int sum = 0;
    int cmdLen = strnlen(cmd, 513);

    if(cmdLen < 11 || cmdLen > 512){
        return -1;
    }

    int i = 4;
    for(; i < cmdLen; i++){
        sum += cmd[i];
    }

    sum %= 255;
    return sum;
}

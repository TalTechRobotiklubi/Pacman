/* CUSTOM INCLUDES ----------------------------------------------------------*/
#include "RobotManager.hpp"

/* METHODS ------------------------------------------------------------------*/
/**
 * Construct a RobotManager object
 *
 * Parameters:
 *      targetId - int, target robot ID
 * 
 * Info about the class variables:
 *      undetectedFrameCount - int, number of frames, with no detected robots
 *      undetectedTargetCOunt - int, number of frames, with undetected target
 *      targetId - int, target robots ID
 *      lastLogTime - unsigned long, timestamp of the last log (helps to 
 *                    prevent too much/fast logging
 *      newIds - std::vector<int>, vector array of new aruco ids
 *      newCorners - std::vector<std::vector<cv::Point2f>>, vector array of
 *                   newly detected aruco corners
 *      detectedRobots - std::map<int, Robot>, private, List of all the
 *                       robots that the camera sees/detects
 *      removedRobots - std::map<int, Robot>, private, List of the robots that
 *                      were detected at some point but then lost (either by
 *                      malfunction or just by removing the robot from the
 *                      playing field). This is used to send stop commands to
 *                      these robots just in case.
 */
RobotManager::RobotManager(){}

/**
 * Update the (robots) data based on the latest detection.
 * 
 * Parameters: 
 *      newIds - std::vector<int>, vector array of new ArUco ids
        newCorners - std::vector<std::vector<cv::Point2f>>, vector array of
 *                   newly detected ArUco corners
 *
 * Returns:
 *      std::map<std::string, std::map<int, Robot>>, Map with specific keys:
 *      "detected" - map of detected robots
 *      "removed" - map of undetected robots
 */
rob_manager_result_t RobotManager::update(std::vector<int>newIds, 
        std::vector<std::vector<cv::Point2f>> newCorners)
{
    this->newIds = newIds;
    this->newCorners = newCorners;

    rob_manager_result_t result;
    const int idsSize = this->newIds.size();
    
    /* Create/update the detected robots */ 
    for(int i = 0; i < idsSize; i++){
        int currentId = this->newIds[i];

        if(this->detectedRobots.find(currentId) != this->detectedRobots.end()){
            /* Update the robot */
            this->detectedRobots.at(currentId).update(this->newCorners[i]);
        }else{
            /* Create the robot */
            this->detectedRobots.insert(std::pair<int, Robot>(currentId,
                        Robot(currentId, this->newCorners[i]) ) );
        }
    }
    
    this->manageRobots();
    
    result.detectedRobots = this->detectedRobots;
    result.removedRobots = this->removedRobots;
    return result;
}

/**
 * Check if we are unable to detect robots for MAX_UNDETECTED_ROBOT times. 
 * If so, add them to the removedRobots vector and remove them 
 * from detectedRobots map.
 */
void RobotManager::manageRobots()
{
    for(std::map<int, Robot>::iterator it = this->detectedRobots.begin();
            it != this->detectedRobots.end(); it++){

        for(int i = 0; i < this->newIds.size(); i++){
            if(it->first == this->newIds[i]){
                it->second.undetectedCount = 0;
                if(this->removedRobots.find(this->newIds[i]) !=
                        this->removedRobots.end()){
                    this->removedRobots.erase(this->newIds[i]);
                }
                break;
            }else if(i+1 == this->newIds.size()){
                if(it->second.undetectedCount <= MAX_UNDETECTED_ROBOT){
                    it->second.undetectedCount++;
                }else{
                    this->removedRobots.insert(std::pair<int, Robot>(it->first,
                                it->second));
                }
            }
        }
        
    }
    
    /* Remove the undetected/removed robots from the detected map */
    for(std::map<int, Robot>::iterator it = this->removedRobots.begin();
            it != this->removedRobots.end(); it++){
        if(this->detectedRobots.find(it->first) != this->detectedRobots.end()){
            this->detectedRobots.erase(it->first);
        }
    }
}


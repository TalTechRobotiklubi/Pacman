/* CUSTOM INCLUDES ----------------------------------------------------------*/
#include "ChaseGame.hpp"

/* METHODS ------------------------------------------------------------------*/
ChaseGame::ChaseGame(CameraThread *cameraThread, InputThread *inputThread,
                     RadioThread *radioThread)
{
    this->cameraThread = cameraThread;
    this->inputThread = inputThread;
    this->radioThread = radioThread;
    this->cmdGen = new CommandGenerator();
    this->unitConverter = new UnitConverter();
    this->scoreManager = new ScoreManager("chase_scores.txt");
}

ChaseGame::~ChaseGame()
{
    delete this->cameraThread;
    delete this->inputThread;
    delete this->radioThread;
    delete this->cmdGen;
    delete this->unitConverter;
    delete this->scoreManager;
}

void ChaseGame::init()
{
    using namespace std::chrono_literals;
    
    this->scoreManager->init();
    
    while(this->PX_TO_CM == 0.f &&
            !(this->inputThread->isKeyPressed("left ctrl") &&
              this->inputThread->isKeyPressed("q"))){

        camera_result_t cameraResult = this->cameraThread->getResult();
        if(cameraResult.ids.size() == 0){
            std::cout << "No ArUcos detected! Retrying to set PX_TO_CM!" <<
                std::endl;
            std::this_thread::sleep_for(1s);
            continue;
        }

        this->PX_TO_CM =
            this->unitConverter->calcPXToCM(cameraResult.corners[0]);
    }
    std::cout << "PX_TO_CM = " << this->PX_TO_CM << std::endl;
    std::cout << "Chase game successfully initialized!" << std::endl;
}

void ChaseGame::run()
{
    using namespace std::chrono_literals;
    while(!(this->inputThread->isKeyPressed("left ctrl") &&
                this->inputThread->isKeyPressed("q"))){
        
        /* Get the result from camera thread */ 
        camera_result_t cameraResult = this->cameraThread->getResult();

        /* Display the detected frame */
        this->inputThread->showFrame(&cameraResult.frame,
                "../res/empty-frame.png");
        
        /* Check if we have already processed the given result */
        if(cameraResult.frame.time <= this->lastCameraResultTime){
            std::this_thread::sleep_for(1ms);
            continue;
        }
        this->lastCameraResultTime = cameraResult.frame.time;

        Robot targetRobot = this->findTarget(&cameraResult);
        
        this->handleGameState(&targetRobot, &cameraResult);
        this->countScore();
        
        radio_msg_t radioMsg;
        
        if(this->gameState == this->GAME_RUN){
            std::map<int, Robot> collisionRobots =
                this->findRobotsInCollisionCourse(&cameraResult);
            radioMsg.robotsWithCmd = this->genChaseCmds(&cameraResult, 
                    collisionRobots);
            radioMsg.playerCmd = this->genPlayerCmd();
        }

        radioMsg.time = this->lastCameraResultTime;
        this->radioThread->setMsg(radioMsg);

        this->logGameState();
    }
}

void ChaseGame::close()
{
    this->scoreManager->close();
}

void ChaseGame::logGameState()
{
    static unsigned long lastLogTime = 0;

    if(this->prevGameState != this->gameState){
        std::cout << "Game state: " << this->gameState << std::endl;
    }
}

Robot ChaseGame::findTarget(camera_result_t *cameraResult)
{
    Robot target(-1, {cv::Point2f(0.f, 0.f)});

    if(cameraResult->detectedRobots.find(this->TARGET_ID) != 
            cameraResult->detectedRobots.end()){
        target = cameraResult->detectedRobots.at(this->TARGET_ID);
        cameraResult->detectedRobots.erase(this->TARGET_ID);
    }

    return target;
}

void ChaseGame::countScore()
{
    if(this->gameState == this->GAME_RUN &&
            this->prevGameState == this->GAME_INIT){
        this->score = 0;
        this->pauseTime = 0;
        this->gameStart = Time::time();
    }else if(this->gameState == this->GAME_PAUSE &&
            this->prevGameState != this->GAME_PAUSE){
        this->pauseStart = Time::time();
    }else if(this->gameState != this->GAME_PAUSE &&
            this->prevGameState == this->GAME_PAUSE){
        this->pauseTime += Time::time() - this->pauseStart;
    }else if(this->gameState == this->GAME_OVER &&
            this->prevGameState != this->GAME_OVER){
        this->score = Time::time() - this->gameStart - this->pauseTime;
        
        score_entry_t score_entry;
        score_entry.name = "oliver";
        score_entry.score = this->score;
        score_entry.time = Time::epoch();

        this->scoreManager->addScore(score_entry);

        std::cout << "Score: " << this->score << std::endl;
    }
}

void ChaseGame::handleGameState(Robot *targetRobot,
    camera_result_t *cameraResult)
{
    this->prevGameState = this->gameState;

    if(this->gameState == this->GAME_INIT){
        if(this->inputThread->isKeyReleased("s")){
            this->gameState = this->GAME_RUN;
        }
    }else if(this->gameState == this->GAME_RUN){
        if(targetRobot->getId() == -1){
            if(this->undetectedTargetCounter >= this->MAX_UNDETECTED_TARGET){
                this->gameState = this->GAME_PAUSE;
            }else{
                this->undetectedTargetCounter++;
            }
            return;
        }
        this->undetectedTargetCounter = 0;
    
        if(cameraResult->detectedRobots.size() == 0){
            if(this->undetectedRobotsCounter >= this->MAX_UNDETECTED_ROBOTS){
                this->gameState = this->GAME_PAUSE;
            }else{
                this->undetectedRobotsCounter++;
            }
            return;
        }
        this->undetectedRobotsCounter = 0;
        
        cv::Point2f target = targetRobot->getCenter();
        for(std::map<int, Robot>::iterator it =
                cameraResult->detectedRobots.begin();
                it != cameraResult->detectedRobots.end(); it++){
            it->second.updateTarget(target);
            if(it->second.getTargetVecLen()*this->PX_TO_CM < 
                    this->CATCH_DISTANCE_CM){
                this->gameState = this->GAME_OVER;
            }
        }
        
        if(this->inputThread->isKeyReleased("p")){
            this->gameState = this->GAME_PAUSE;
        }
    }else if(this->gameState == this->GAME_PAUSE){
        if(this->inputThread->isKeyReleased("p")){
            this->gameState = this->GAME_RUN;
        }
    }else if(this->gameState == this->GAME_OVER){
        if(this->inputThread->isKeyReleased("r")){
            this->gameState = this->GAME_RESTART;
        }
    }
}

std::map<int, Robot> ChaseGame::findRobotsInCollisionCourse(
        camera_result_t *cameraResult)
{
    std::map<int, Robot> detectedRobots = cameraResult->detectedRobots;
    
    std::set<std::string> distances;
    std::map<int, Robot> robotsInCollisionCourse;

    for(std::map<int, Robot>::iterator i = detectedRobots.begin();
            i != detectedRobots.end(); i++){
            
        for(std::map<int, Robot>::iterator j = detectedRobots.begin();
            j != detectedRobots.end(); j++){

            if(i->first == j->first){
                continue;
            }

            std::string distanceId;
            if(i->first > j->first){
                distanceId = std::to_string(j->first) + "-" +
                    std::to_string(i->first);
            }else{
                distanceId = std::to_string(i->first) + "-" +
                    std::to_string(j->first);
            }

            if(distances.find(distanceId) != distances.end()){
                continue;
            }else{
                distances.insert(distanceId);
            }
            
            float distanceX = i->second.getCenter().x -j->second.getCenter().x;
            float distanceY = i->second.getCenter().y -j->second.getCenter().y;

            float distance = std::sqrt(std::pow(distanceX, 2.f) +
                    std::pow(distanceY, 2.f));

            if(distance*this->PX_TO_CM > 30){
                continue;
            }

            if(robotsInCollisionCourse.find(i->first) == 
                    robotsInCollisionCourse.end()){
                robotsInCollisionCourse.insert(std::pair<int, Robot>
                        (i->first, i->second));
            }

            if(robotsInCollisionCourse.find(j->first) == 
                    robotsInCollisionCourse.end()){
                robotsInCollisionCourse.insert(std::pair<int, Robot>
                        (j->first, j->second));
            }
        }
    }

    if(robotsInCollisionCourse.size() < 2){
        robotsInCollisionCourse.clear();
        return robotsInCollisionCourse;
    }
    
    /* Finding the VIP robot. */
    float shortestDistance =
        robotsInCollisionCourse.begin()->second.getTargetVecLen();
    int VIPRobotId = robotsInCollisionCourse.begin()->first;
    for(std::map<int, Robot>::iterator it = robotsInCollisionCourse.begin();
            it != robotsInCollisionCourse.end(); it++){
        float currentDistance = it->second.getTargetVecLen();

        if(shortestDistance > currentDistance){
            shortestDistance = currentDistance;
            VIPRobotId = it->first;
        }
    }

    /* For debugging */
    /*std::cout << "Robots in collision course are:" << std::endl;
    for(std::map<int, Robot>::iterator it = robotsInCollisionCourse.begin();
            it != robotsInCollisionCourse.end(); it++){
        std::cout << " " << it->first << std::endl;
    }
    std::cout << "The VIP robot is " << VIPRobotId;*/

    robotsInCollisionCourse.erase(VIPRobotId);
    return robotsInCollisionCourse;
}

/**
 * Generate chase commands using turn and motorset
 * 
 * Parameters:
 *       cameraResult - camera_result_t, Result from the camera thread
 * 
 * Returns: std::map<int, Robot>, Map where keys are IDs and values are robots 
 *       with commands.
 */
std::map<int, Robot> ChaseGame::genChaseCmds(camera_result_t *cameraResult,
        std::map<int, Robot> collisionRobots)
{
    std::map<int, Robot> robotsWithCmd; 

    std::map<int, Robot> detectedRobots = cameraResult->detectedRobots;
    std::map<int, Robot> removedRobots = cameraResult->removedRobots;

    /* Loop through all detected robos */ 
    for(std::map<int, Robot>::iterator it = detectedRobots.begin();
            it != detectedRobots.end(); it++){
        std::string cmd;

        /* Send stop if chaser robot gets too close to player */
        if(collisionRobots.find(it->first) != collisionRobots.end()){
            cmd = this->cmdGen->generate(it->first, CMD_END, {0});
            it->second.cmd = cmd;
            robotsWithCmd.insert(std::pair<int, Robot>(it->first, it->second));
            continue;
        }

        int turnAngle = std::round(it->second.getTargetAngle()); 
        
        /* Use motorset if chaser robot is at a small angle towards the target
           otherwise use the turn command */
        if(turnAngle > -this->MOTORS_ANGLE_THRESH &&
                turnAngle < this->MOTORS_ANGLE_THRESH){
            std::vector<int> data;
            data.push_back(it->second.getMotorPowers().first);
            data.push_back(it->second.getMotorPowers().second);

            cmd = this->cmdGen->generate(it->first, CMD_MOTORS, data);
        }else{
            cmd = this->cmdGen->generate(it->first, CMD_TURN,{turnAngle, 200});
        }
        
        it->second.cmd = cmd;
        robotsWithCmd.insert(std::pair<int, Robot>(it->first, it->second));
    }

    /* Send stop commands to removed robots */
    for(std::map<int, Robot>::iterator it = removedRobots.begin();
            it != removedRobots.end(); it++){
        std::string cmd = this->cmdGen->generate(it->first, CMD_END, {0});
        it->second.cmd = cmd;
        robotsWithCmd.insert(std::pair<int, Robot>(it->first, it->second));
    }

    return robotsWithCmd;
}

/**
 * Generate player command based on the inputs from the input thread
 *
 * Returns: std::string, Player command as a radio message
 */
std::string ChaseGame::genPlayerCmd()
{
    std::string playerCmd;

    if(this->inputThread->isKeyPressed("w") &&
            this->inputThread->isKeyPressed("a")){
        playerCmd =
            this->cmdGen->generate(this->TARGET_ID, CMD_MOTORS, {150, 300}); 
    }else if(this->inputThread->isKeyPressed("w") &&
            this->inputThread->isKeyPressed("d")){
        playerCmd =
            this->cmdGen->generate(this->TARGET_ID, CMD_MOTORS, {300, 150}); 
    }else if(this->inputThread->isKeyPressed("s") &&
            this->inputThread->isKeyPressed("d")){
        playerCmd =
            this->cmdGen->generate(this->TARGET_ID, CMD_MOTORS, {-300, -150}); 
    }else if(this->inputThread->isKeyPressed("s") &&
            this->inputThread->isKeyPressed("a")){
        playerCmd =
            this->cmdGen->generate(this->TARGET_ID, CMD_MOTORS, {-150, -300}); 
    }else if(this->inputThread->isKeyPressed("w")){
        playerCmd =
            this->cmdGen->generate(this->TARGET_ID, CMD_MOTORS, {300, 300}); 
    }else if(this->inputThread->isKeyPressed("a")){
        playerCmd =
            this->cmdGen->generate(this->TARGET_ID, CMD_MOTORS, {-300, 300}); 
    }else if(this->inputThread->isKeyPressed("d")){
        playerCmd =
            this->cmdGen->generate(this->TARGET_ID, CMD_MOTORS, {300, -300}); 
    }else if(this->inputThread->isKeyPressed("s")){
        playerCmd =
            this->cmdGen->generate(this->TARGET_ID, CMD_MOTORS, {-300, -300}); 
    }else{
        playerCmd = this->cmdGen->generate(this->TARGET_ID, CMD_END, { 0 });
    }

    return playerCmd;
}

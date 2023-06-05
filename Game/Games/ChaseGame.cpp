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
    this->gridManager = new GridManager();
    this->pathFinder = new PathFinder();
}

ChaseGame::~ChaseGame()
{
    delete this->cameraThread;
    delete this->inputThread;
    delete this->radioThread;
    delete this->cmdGen;
    delete this->unitConverter;
    delete this->scoreManager;
    delete this->gridManager;
    delete this->pathFinder;
}

void ChaseGame::init()
{
    using namespace std::chrono_literals;

    camera_result_t cameraResult;
    
    /* Wait for user input to start the game initilization */ 
    std::cout << "Press enter to initialize the game..." << std::endl;
    while(!this->inputThread->isKeyPressed("return") ||
          cameraResult.frame.mat.empty()){
        cameraResult = this->cameraThread->getResult();
        this->inputThread->showFrame(&cameraResult.frame,
                "../res/empty-frame.png");
        if(this->inputThread->isKeyPressed("left ctrl") &&
              this->inputThread->isKeyPressed("q")){
            return;
        }
        std::this_thread::sleep_for(16ms);
    }
    
    /* Creating grid */ 
    std::cout << "Starting grid creation!" << std::endl; 
    this->grid.clear();
    unsigned long gridStartTime = Time::time();
    this->grid = this->gridManager->createGrid(&cameraResult.frame);    
    std::cout << "Grid created! (took " << (Time::time() - gridStartTime) << 
        " ms)" << std::endl;
    
    /* PX_TO_CM calculation */ 
    while(this->PX_TO_CM == 0.f &&
            !(this->inputThread->isKeyPressed("left ctrl") &&
              this->inputThread->isKeyPressed("q"))){

        cameraResult = this->cameraThread->getResult();
        this->inputThread->showFrame(&cameraResult.frame,
                "../res/empty-frame.png");
        if(cameraResult.arucoIds.size() == 0){
            std::cout << "No ArUcos detected! Retrying to set PX_TO_CM!" <<
                std::endl;
            std::this_thread::sleep_for(1s);
            continue;
        }

        this->PX_TO_CM =
            this->unitConverter->calcPXToCM(cameraResult.arucoCorners[0]);
    }
    std::cout << "PX_TO_CM = " << this->PX_TO_CM << std::endl;
    this->pathFinder->PX_TO_CM = this->PX_TO_CM;

    /* Wait for user input to save the start positions */
    std::cout << "Press enter to save robot start positions..." << std::endl;
    while(!this->inputThread->isKeyPressed("return")){
        cameraResult = this->cameraThread->getResult();
        this->inputThread->showFrame(&cameraResult.frame,
                "../res/empty-frame.png");
        std::this_thread::sleep_for(16ms);
    }
    
    /* Save start positions */
    while(1){
        if(this->inputThread->isKeyPressed("left ctrl") &&
              this->inputThread->isKeyPressed("q")){
            return;
        }
        std::vector<int> arucoIds = cameraResult.arucoIds;
        
        int error = 0;
        if(arucoIds.size() < 2){
            std::cout << "There are not enought robots in the playing field!"<<
                "Chase game requires atleast 2 robots!" << std::endl;
            error = 1;
        }else if(std::find(arucoIds.begin(), arucoIds.end(),
                    this->TARGET_ID) == arucoIds.end()){
            std::cout << "Target robot not found!" << std::endl;
            error = 1;
        }

        if(error){
            cameraResult = this->cameraThread->getResult();
            this->inputThread->showFrame(&cameraResult.frame,
                    "../res/empty-frame.png");
            std::cout << "Retrying to set start positions!" << std::endl;
            std::this_thread::sleep_for(1s);
            continue;
        }

        this->startNodes = {};
        for(int i = 0; i < arucoIds.size(); i++){
            std::vector<cv::Point2f> arucoCorners =
                cameraResult.arucoCorners[i];

            cv::Point2f center = (arucoCorners[0] + arucoCorners[2]) / 2.f;

            int nodeI = (int) std::round(center.x/NODE_SIZE);
            int nodeJ = (int) std::round(center.y/NODE_SIZE);
        
            std::cout << nodeI << "; " << nodeJ << std::endl;
        
            if(nodeI > -1 && nodeJ > -1){
                if(nodeI < this->grid.size() &&
                    nodeJ < this->grid[nodeI].size()){
                    this->startNodes.insert(std::pair<int, Node>(arucoIds[i],
                            this->grid[nodeI][nodeJ]));
                }
            }
        }
        break;
    }
    std::cout << "Start positions saved!" << std::endl;
    
    
    /* Score manager initialization */ 
    this->scoreManager->init();
    std::cout << "Chase game successfully initialized!" << std::endl;
}

void ChaseGame::run()
{
    using namespace std::chrono_literals;
    unsigned long actionTotalTime = 0, actionCount = 0;

    while(!(this->inputThread->isKeyPressed("left ctrl") &&
                this->inputThread->isKeyPressed("q"))){
        
        /* Get the result from camera thread */
        camera_result_t cameraResult = this->cameraThread->getResult();
        frame_t current_frame = cameraResult.frame;
        
        /* Draw the paths */ 
        for(std::map<int, std::vector<Node>>::iterator it =this->paths.begin();
                it != this->paths.end() && (this->gameState == GAME_RUN || 
                         this->gameState == GAME_RESTART); it++){
            std::vector<Node> path = it->second;
            for(int i = 0; i < path.size(); i++){
                int x = path[i].getIndex().first;
                int y = path[i].getIndex().second;
                cv::rectangle(current_frame.mat,
                        this->grid[x][y].getCorners()[0],
                        this->grid[x][y].getCorners()[2],
                        cv::Scalar(255, 255, 255)
                );
            }
        }

        /* Display the frame with detected ArUcos, walls and paths */
        this->inputThread->showFrame(&current_frame,
                "../res/empty-frame.png");
        
        /* Check if we have already processed the given result */
        if(cameraResult.frame.time <= this->lastCameraResultTime){
            std::this_thread::sleep_for(1ms);
            continue;
        }
        this->lastCameraResultTime = cameraResult.frame.time;

        this->manageRobots(&cameraResult);

        Robot targetRobot = this->findTarget(&cameraResult);

        this->handleGameState(&targetRobot, &cameraResult);
        this->countScore(&targetRobot);
        
        radio_msg_t radioMsg;
    
        if(this->gameState == this->GAME_RUN){
            if((Time::time() - this->lastPathCalcTime) > 100){
                unsigned long startTime = Time::time();
                
                this->calcPaths(&targetRobot, &cameraResult);
                
                actionTotalTime += (Time::time() - startTime);
                actionCount++;
                this->lastPathCalcTime = Time::time();
            }
            radioMsg.robotsWithCmd = this->genCmds(&cameraResult);
            radioMsg.playerCmd = this->genPlayerCmd();
        }else if(this->gameState == this->GAME_RESTART){
            if((Time::time() - this->lastPathCalcTime) > 100){
                this->calcRestartPaths(&cameraResult);
                this->lastPathCalcTime = Time::time();
            }
            radioMsg.robotsWithCmd = this->genCmds(&cameraResult);
        }
        
        radioMsg.time = this->lastCameraResultTime;
        this->radioThread->setMsg(radioMsg);

        this->logGameState();
    }
    
    if(actionCount != 0){ 
        std::cout << "Action was done " << actionCount << " time(s)!" << 
            std::endl << "Action took on average: " <<
            (actionTotalTime/actionCount) << " ms!" << std::endl;;
    }
}


void ChaseGame::logGameState()
{
    if(this->prevGameState != this->gameState){
        std::cout << "Game state: " << this->gameState << std::endl;
    }
}

void ChaseGame::saveStartPositions()
{
    this->startNodes = {};
    for(std::map<int, Robot>::iterator it = this->robots.begin();
            it != this->robots.end(); it++){
        
        int nodeI = (int) std::round(it->second.getCenter().x/NODE_SIZE);
        int nodeJ = (int) std::round(it->second.getCenter().y/NODE_SIZE);
         
        std::cout << nodeI << "; " << nodeJ << std::endl;

        if(nodeI > -1 && nodeJ > -1){
            if(nodeI < this->grid.size() &&
                nodeJ < this->grid[nodeI].size()){
                this->startNodes.insert(std::pair<int, Node>(it->first,
                        this->grid[nodeI][nodeJ]));
            }
        }
    }
}

void ChaseGame::manageRobots(camera_result_t *cameraResult)
{
    const int idsSize = cameraResult->arucoIds.size();
    
    /* Create/update the detected robots */ 
    for(int i = 0; i < idsSize; i++){
        int currentId = cameraResult->arucoIds[i];

        if(this->robots.find(currentId) != this->robots.end()){
            /* Update the robot */
            this->robots.at(currentId).update(cameraResult->arucoCorners[i]);
        }else{
            /* Create the robot */
            this->robots.insert(std::pair<int, Robot>(currentId,
                        Robot(currentId, cameraResult->arucoCorners[i],
                            this->MAX_PWR, this->P_CONST, this->D_CONST) ) );
        }
    }
}

Robot ChaseGame::findTarget(camera_result_t *cameraResult)
{
    Robot target(-1, {cv::Point2f(0.f, 0.f)}, 0, 0, 0);

    const int idsSize = cameraResult->arucoIds.size();
    
    for(int i = 0; i < idsSize; i++){
        if(cameraResult->arucoIds[i] == TARGET_ID){
            target = Robot(TARGET_ID, cameraResult->arucoCorners[i],
                    this->MAX_PWR, this->P_CONST, this->D_CONST);
        }
    }

    return target;
}

void ChaseGame::handleGameState(Robot *targetRobot,
        camera_result_t *cameraResult)
{
    this->prevGameState = this->gameState;

    if(this->gameState == this->GAME_INIT){
        if(this->inputThread->isKeyReleased("s")){
            this->paths = {};
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
    
        cv::Point2f target = targetRobot->getCenter();
        for(std::map<int, Robot>::iterator it = this->robots.begin();
                it != this->robots.end(); it++){

            if(it->first == this->TARGET_ID){
                continue;
            }
            
            cv::Point2f diff = target - it->second.getCenter();            
            float distance = std::sqrt(diff.x*diff.x + diff.y*diff.y);

            if(distance*this->PX_TO_CM < this->CATCH_DISTANCE_CM){
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
    }else if(this->gameState == this->GAME_RESTART){
        int robotsRestartedCount = 0;
        for(std::map<int, Robot>::iterator it = this->robots.begin();
                it != this->robots.end(); it++){
            if(this->startNodes.find(it->first) == this->startNodes.end()){
                continue;
            }
            
            cv::Point2f target = this->startNodes.at(it->first).getCenter();
            
            cv::Point2f diff = target - it->second.getCenter();            
            float distance = std::sqrt(diff.x*diff.x + diff.y*diff.y);

            if(distance*this->PX_TO_CM < 5){
                robotsRestartedCount++;
            }
        }

        if(robotsRestartedCount == this->robots.size()){
            this->gameState = this->GAME_INIT;
        }
    }
}

void ChaseGame::countScore(Robot *targetRobot)
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
        score_entry.name = "player";
        score_entry.score = this->score;
        score_entry.time = Time::epoch();

        this->scoreManager->addScore(score_entry);

        std::cout << "Score: " << this->score << std::endl;
    }
}

void ChaseGame::calcPaths(Robot *targetRobot, camera_result_t *cameraResult)
{
    if(targetRobot->getId() == -1){
        return;
    }

    for(std::map<int, Robot>::iterator it = this->robots.begin();
            it != this->robots.end(); it++){

        if(it->first == this->TARGET_ID){
            continue;
        }
        
        int startNodeI = (int) std::round(it->second.getCenter().x/NODE_SIZE);
        int startNodeJ = (int) std::round(it->second.getCenter().y/NODE_SIZE);
        
        int targetNodeI = 
            (int) std::round(targetRobot->getCenter().x/NODE_SIZE);
        int targetNodeJ = 
            (int) std::round(targetRobot->getCenter().y/NODE_SIZE);
        
        if(startNodeI < 0 || startNodeJ < 0 ||
                startNodeI >= this->grid.size() ||
                startNodeJ >= this->grid[0].size()){
            continue;
        }
        
        if(targetNodeI < 0 || targetNodeJ < 0 ||
                targetNodeI >= this->grid.size() ||
                targetNodeJ >= this->grid[0].size()){
            continue;
        }

        Node startNode = this->grid[startNodeI][startNodeJ];
        Node targetNode = this->grid[targetNodeI][targetNodeJ];
        
        std::vector<Node> path = this->pathFinder->astar(this->grid,
                    startNode.getIndex(), targetNode.getIndex(), 1, it->first,
                    this->TARGET_ID, cameraResult->arucoIds,
                    cameraResult->arucoCorners);
        
        if(this->paths.find(it->first) != this->paths.end()){
            this->paths[it->first] = path;
        }else{
            this->paths.insert(std::pair<int, std::vector<Node>>(it->first,
                        path));
        }
    }
}

void ChaseGame::calcRestartPaths(camera_result_t *cameraResult)
{
    for(std::map<int, Robot>::iterator it = this->robots.begin();
            it != this->robots.end(); it++){
        if(this->startNodes.find(it->first) == this->startNodes.end()){
            continue;
        }
            
        cv::Point2f target = this->startNodes.at(it->first).getCenter();
        cv::Point2f diff = target - it->second.getCenter();            
        float distance = std::sqrt(diff.x*diff.x + diff.y*diff.y);

        if(distance*this->PX_TO_CM < 5){
            if(this->paths.find(it->first) != this->paths.end()){
                this->paths[it->first] = {};
            }else{
                this->paths.insert(std::pair<int, std::vector<Node>>(it->first,
                            {}));
            }
            continue;
        }
        
        int startNodeI = (int) std::round(it->second.getCenter().x/NODE_SIZE);
        int startNodeJ = (int) std::round(it->second.getCenter().y/NODE_SIZE);
        
        int targetNodeI = this->startNodes.at(it->first).getIndex().first;
        int targetNodeJ = this->startNodes.at(it->first).getIndex().second;
        
        if(startNodeI < 0 || startNodeJ < 0 ||
                startNodeI >= this->grid.size() ||
                startNodeJ >= this->grid[0].size()){
            continue;
        }
        
        if(targetNodeI < 0 || targetNodeJ < 0 ||
                targetNodeI >= this->grid.size() ||
                targetNodeJ >= this->grid[0].size()){
            continue;
        }

        Node startNode = this->grid[startNodeI][startNodeJ];
        Node targetNode = this->grid[targetNodeI][targetNodeJ];
        
        std::vector<Node> path = this->pathFinder->astar(this->grid,
                startNode.getIndex(), targetNode.getIndex(), 1, it->first, 0,
                cameraResult->arucoIds, cameraResult->arucoCorners);

        if(this->paths.find(it->first) != this->paths.end()){
            this->paths[it->first] = path;
        }else{
            this->paths.insert(std::pair<int, std::vector<Node>>(it->first,
                        path));
        }
    }
}

std::map<int, Robot> ChaseGame::genCmds(camera_result_t *cameraResult)
{
    std::map<int, Robot> robotsWithCmd; 

    /* Loop through all detected robots */
    for(std::map<int, Robot>::iterator it = this->robots.begin();
            it != this->robots.end(); it++){
        std::string cmd;

        if(this->paths.find(it->first) == this->paths.end()){
            continue;
        }

        std::vector<Node> currentPath = this->paths[it->first];

        if(currentPath.size() == 0){
            it->second.cmd = this->cmdGen->generate(it->first, CMD_END, {0});
            robotsWithCmd.insert(std::pair<int, Robot>(it->first, it->second));
            continue;
        }

        cv::Rect2f forbiddenZone(it->second.getArucoCorners()[0], 
                (it->second.getArucoCorners()[2] +
                 it->second.getArucoCorners()[3])/2);

        if(currentPath.size() == 1){
            it->second.updateTarget(currentPath.front().getCenter());
        }

        for(std::vector<Node>::iterator currentNode = currentPath.end();
                currentNode != currentPath.begin() && currentPath.size() != 1;
                currentNode--){
            if(currentNode == currentPath.begin()+1){
                it->second.updateTarget(currentNode->getCenter());
                break;
            }if(forbiddenZone.contains(currentNode->getCenter())){
                currentNode++;
                it->second.updateTarget(currentNode->getCenter());
                break;
            }

            float distanceX = currentNode->getCenter().x - 
                it->second.getMid().x;
            float distanceY = currentNode->getCenter().y - 
                it->second.getMid().y;

            float distance = std::sqrt(distanceX*distanceX + 
                    distanceY*distanceY);
            
            /*std::cout << "#" << count << ": " << distance*this->PX_TO_CM << 
                std::endl;*/
            if(distance*this->PX_TO_CM <= this->CLOSEST_NODE_DIST){
                it->second.updateTarget(currentNode->getCenter());
                break;
            }
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
            cmd = this->cmdGen->generate(it->first, CMD_TURN,{turnAngle, 100});
        }
        
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

    int playerSpeed = this->MAX_PWR;

    if(this->inputThread->isKeyPressed("w") &&
            this->inputThread->isKeyPressed("a")){
        playerCmd =
            this->cmdGen->generate(this->TARGET_ID, CMD_MOTORS, 
                    {playerSpeed/2, playerSpeed}); 
    }else if(this->inputThread->isKeyPressed("w") &&
            this->inputThread->isKeyPressed("d")){
        playerCmd =
            this->cmdGen->generate(this->TARGET_ID, CMD_MOTORS,
                    {playerSpeed, playerSpeed/2}); 
    }else if(this->inputThread->isKeyPressed("s") &&
            this->inputThread->isKeyPressed("d")){
        playerCmd =
            this->cmdGen->generate(this->TARGET_ID, CMD_MOTORS, 
                    {-playerSpeed, -playerSpeed/2}); 
    }else if(this->inputThread->isKeyPressed("s") &&
            this->inputThread->isKeyPressed("a")){
        playerCmd =
            this->cmdGen->generate(this->TARGET_ID, CMD_MOTORS,
                    {-playerSpeed/2, -playerSpeed}); 
    }else if(this->inputThread->isKeyPressed("w")){
        playerCmd =
            this->cmdGen->generate(this->TARGET_ID, CMD_MOTORS,
                    {playerSpeed, playerSpeed}); 
    }else if(this->inputThread->isKeyPressed("a")){
        playerCmd =
            this->cmdGen->generate(this->TARGET_ID, CMD_MOTORS,
                    {-playerSpeed, playerSpeed}); 
    }else if(this->inputThread->isKeyPressed("d")){
        playerCmd =
            this->cmdGen->generate(this->TARGET_ID, CMD_MOTORS,
                    {playerSpeed, -playerSpeed}); 
    }else if(this->inputThread->isKeyPressed("s")){
        playerCmd =
            this->cmdGen->generate(this->TARGET_ID, CMD_MOTORS,
                    {-playerSpeed, -playerSpeed}); 
    }else{
        playerCmd = this->cmdGen->generate(this->TARGET_ID, CMD_END, { 0 });
    }

    return playerCmd;
}

void ChaseGame::close()
{
    this->scoreManager->close();
}

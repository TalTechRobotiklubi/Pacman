/* CUSTOM INCLUDES ----------------------------------------------------------*/
#include "PacmanGame.hpp"

/* METHODS ------------------------------------------------------------------*/
PacmanGame::PacmanGame(CameraThread *cameraThread, InputThread *inputThread,
                     RadioThread *radioThread)
{
    this->cameraThread = cameraThread;
    this->inputThread = inputThread;
    this->radioThread = radioThread;
    this->cmdGen = new CommandGenerator();
    this->unitConverter = new UnitConverter();
    this->scoreManager = new ScoreManager("pacman_scores.txt");
    this->gridManager = new GridManager();
    this->pathFinder = new PathFinder();
    this->pathFinder->clearanceLevel = this->CLEARANCE;
}

PacmanGame::~PacmanGame()
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

void PacmanGame::init()
{
    using namespace std::chrono_literals;

    camera_result_t cameraResult;
    
    /* Wait for user input to start the wall detection */ 
    std::cout << "Press enter to start wall detection..." << std::endl;
    while(!this->inputThread->isKeyPressed("return")){
        cameraResult = this->cameraThread->getResult();
        this->inputThread->showFrame(&cameraResult.frame, 
                "../res/empty-frame.png");
        if(this->inputThread->isKeyPressed("left ctrl") &&
              this->inputThread->isKeyPressed("q")){
            return;
        }
        std::this_thread::sleep_for(16ms);
    }
    
    /* Grid creation and wall detection with clearance */ 
    int wallDetectionDone = 0;
    while(!wallDetectionDone){
        this->grid.clear();

        cameraResult = this->cameraThread->getResult();
        this->inputThread->showFrame(&cameraResult.frame, 
                "../res/empty-frame.png");
        
        std::cout << "Starting grid creation!" << std::endl; 
        unsigned long gridStartTime = Time::time();
        this->grid = this->gridManager->createGrid(&cameraResult.frame);    
        std::cout << "Grid created! (took " << (Time::time() - gridStartTime) << 
            " ms)" << std::endl;
    
        std::cout << "Starting wall detection!" << std::endl; 
        unsigned long wallStartTime = Time::time();
        this->grid = this->gridManager->detectWalls(&cameraResult.frame,
                this->grid, 1);
        std::cout << "Wall detection done! (took " <<
            (Time::time() - wallStartTime) << " ms)" << std::endl;
        
        std::cout << "Starting clearance!" << std::endl; 
        unsigned long clearanceStartTime = Time::time();
        this->grid = this->gridManager->addClearance(this->grid);
        std::cout << "Clearance done! (took " <<
            (Time::time() - clearanceStartTime) << " ms)" << std::endl;
        
        std::cout << "Press \"Enter\" to accept the wall detection or " <<
           "\"r\" to restart the wall detection..." << std::endl;
        while(1){
            if(this->inputThread->isKeyPressed("return")){
                wallDetectionDone = 1;
                std::cout << "Wall detection result accepted!" << std::endl;
                break;
            }else if(this->inputThread->isKeyPressed("r")){
                std::cout << "Restarting wall detection..." << std::endl;
                break; 
            }else if(this->inputThread->isKeyPressed("left ctrl") &&
              this->inputThread->isKeyPressed("q")){
                return;
            }
            
            cameraResult = this->cameraThread->getResult();

            for(int i = 0; i < this->grid.size(); i++){
                for(int j = 0; j < this->grid[i].size(); j++){
                    if(this->grid[i][j].hasWall){
                        cv::rectangle(cameraResult.frame.mat,
                                this->grid[i][j].getCorners()[0],
                                this->grid[i][j].getCorners()[2],
                                cv::Scalar(255, 0, 0)
                        );
                    }
                }
            }
            
            this->inputThread->showFrame(&cameraResult.frame, 
                    "../res/empty-frame.png");
            
            std::this_thread::sleep_for(16ms);
        }
    }
    
    /* PX_TO_CM calculation */ 
    while(this->PX_TO_CM == 0.f &&
            !(this->inputThread->isKeyPressed("left ctrl") &&
              this->inputThread->isKeyPressed("q"))){

        cameraResult = this->cameraThread->getResult();
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
    
    /* Score manager initialization */ 
    this->scoreManager->init();
    std::cout << "Pacman game successfully initialized!" << std::endl;
}

void PacmanGame::run()
{
    using namespace std::chrono_literals;
    unsigned long actionTotalTime = 0, actionCount = 0;

    while(!(this->inputThread->isKeyPressed("left ctrl") &&
                this->inputThread->isKeyPressed("q"))){
        
        /* Get the result from camera thread */
        camera_result_t cameraResult = this->cameraThread->getResult();
        frame_t current_frame = cameraResult.frame;
        
        /* Draw walls */
        for(int i = 0; i < this->grid.size(); i++){
            for(int j = 0; j < this->grid[i].size(); j++){
                if(this->grid[i][j].hasWall){
                    cv::rectangle(current_frame.mat,
                            this->grid[i][j].getCorners()[0],
                            this->grid[i][j].getCorners()[2],
                            cv::Scalar(255, 0, 0)
                    );
                }else if(this->grid[i][j].arucoId != -1 && 
                        (this->gameState == GAME_RUN || 
                         this->gameState == GAME_RESTART)){
                    cv::rectangle(current_frame.mat,
                            this->grid[i][j].getCorners()[0],
                            this->grid[i][j].getCorners()[2],
                            cv::Scalar(0, 0, 255)
                    );
                }
            }
        }
        
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
    
        if(this->gameState == this->GAME_RUN && 
                this->prevGameState == this->GAME_INIT){
            this->paths = {};
            if(this->startNodes.size() == 0){
                this->saveStartPositions();
            }
        }
        
        if(this->gameState == this->GAME_RUN){
            if((Time::time() - this->lastPathCalcTime) > 500){
                unsigned long startTime = Time::time();
                
                this->calcPaths(&targetRobot, &cameraResult);
                
                actionTotalTime += (Time::time() - startTime);
                actionCount++;
                this->lastPathCalcTime = Time::time();
            }
            radioMsg.robotsWithCmd = this->genCmds(&cameraResult);
            radioMsg.playerCmd = this->genPlayerCmd();
        }else if(this->gameState == this->GAME_RESTART){
            this->calcRestartPaths(&cameraResult);
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


void PacmanGame::logGameState()
{
    if(this->prevGameState != this->gameState){
        std::cout << "Game state: " << this->gameState << std::endl;
    }
}

void PacmanGame::saveStartPositions()
{
    this->startNodes = {};
    for(std::map<int, Robot>::iterator it = this->robots.begin();
            it != this->robots.end(); it++){
        
        int nodeI = (int) std::round(it->second.getCenter().x/NODE_SIZE);
        int nodeJ = (int) std::round(it->second.getCenter().y/NODE_SIZE);

        if(nodeI > -1 && nodeJ > -1){
            if(nodeI < this->grid.size() && nodeJ < this->grid[nodeI].size()){
                this->startNodes.insert(std::pair<int, Node>(it->first,
                        this->grid[nodeI][nodeJ]));
            }
        }
    }
}

void PacmanGame::manageRobots(camera_result_t *cameraResult)
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

Robot PacmanGame::findTarget(camera_result_t *cameraResult)
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

void PacmanGame::handleGameState(Robot *targetRobot,
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

void PacmanGame::countScore(Robot *targetRobot)
{
    if(targetRobot->getId() != -1){
        if(this->gameState == this->GAME_RUN){
            if((Time::time() - this->lastPunishTime) >= 5000){
                int targetNodeI = 
                    (int) std::round(targetRobot->getCenter().x/NODE_SIZE);
                int targetNodeJ = 
                    (int) std::round(targetRobot->getCenter().y/NODE_SIZE);

                std::vector<cv::Point2f> arucoCorners =
                    targetRobot->getArucoCorners();

                if(this->grid[targetNodeI][targetNodeJ].hasWall){
                    std::cout << "Player hit a wall!" << std::endl;
                    this->punishCount++;
                    this->lastPunishTime = Time::time();
                }
                
                /*for(int i = 0; i < 4; i++){
                    if(this->grid[targetNodeI][targetNodeJ].hasWall){
                        std::cout << "Player hit a wall!" << std::endl;
                        this->punishCount++;
                        this->lastPunishTime = Time::time();
                        break;
                    }
                    
                    targetNodeI = (int) std::round(arucoCorners[i].x/NODE_SIZE);
                    targetNodeJ = (int) std::round(arucoCorners[i].y/NODE_SIZE);
                }*/
            }

        }
    }

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

        if(this->score < 30000*this->punishCount){
            this->score = 0;
        }else{
            this->score -= 30000*this->punishCount;
        }

        score_entry_t score_entry;
        score_entry.name = "oliver";
        score_entry.score = this->score;
        score_entry.time = Time::epoch();

        this->scoreManager->addScore(score_entry);

        std::cout << "Score: " << this->score << std::endl;
    }
}

void PacmanGame::calcPaths(Robot *targetRobot, camera_result_t *cameraResult)
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
        
        /*int targetNodeI = 23;*/
        int targetNodeI = 
            (int) std::round(targetRobot->getCenter().x/NODE_SIZE);
        /*int targetNodeJ = 16;*/
        int targetNodeJ = 
            (int) std::round(targetRobot->getCenter().y/NODE_SIZE);

        Node startNode = this->grid[startNodeI][startNodeJ];
        Node targetNode = this->grid[targetNodeI][targetNodeJ];
        
        for(int i = 0; (startNode.clearance < this->CLEARANCE) && i < 4; i++){
            float startNodeX = it->second.getArucoCorners()[i].x;
            float startNodeY = it->second.getArucoCorners()[i].y;
            startNodeI = (int) std::round(startNodeX/NODE_SIZE);
            startNodeJ = (int) std::round(startNodeY/NODE_SIZE);

            startNode = this->grid[startNodeI][startNodeJ];
        }

        for(int i = 0; (targetNode.clearance < this->CLEARANCE) && i < 4; i++){
            float targetNodeX = targetRobot->getArucoCorners()[i].x;
            float targetNodeY = targetRobot->getArucoCorners()[i].y;
            targetNodeI = (int) std::round(targetNodeX/NODE_SIZE);
            targetNodeJ = (int) std::round(targetNodeY/NODE_SIZE);

            targetNode = this->grid[targetNodeI][targetNodeJ];
        }
        
        std::vector<Node> path = {};

        if(startNode.clearance < this->CLEARANCE ||
                targetNode.clearance < this->CLEARANCE){
            this->pathFinder->clearanceLevel = 1;
            startNodeI = (int) std::round(it->second.getCenter().x/NODE_SIZE);
            startNodeJ = (int) std::round(it->second.getCenter().y/NODE_SIZE);
            
            targetNodeI =
                (int) std::round(targetRobot->getCenter().x/NODE_SIZE);
            targetNodeJ = 
                (int) std::round(targetRobot->getCenter().y/NODE_SIZE);

            Node startNode = this->grid[startNodeI][startNodeJ];
            Node targetNode = this->grid[targetNodeI][targetNodeJ];
            
            for(int i = 0; startNode.hasWall && i < 4; i++){
                float startNodeX = it->second.getArucoCorners()[i].x;
                float startNodeY = it->second.getArucoCorners()[i].y;
                startNodeI = (int) std::round(startNodeX/NODE_SIZE);
                startNodeJ = (int) std::round(startNodeY/NODE_SIZE);

                startNode = this->grid[startNodeI][startNodeJ];
            }

            for(int i = 0; targetNode.hasWall && i < 4; i++){
                float targetNodeX = targetRobot->getArucoCorners()[i].x;
                float targetNodeY = targetRobot->getArucoCorners()[i].y;
                targetNodeI = (int) std::round(targetNodeX/NODE_SIZE);
                targetNodeJ = (int) std::round(targetNodeY/NODE_SIZE);

                targetNode = this->grid[targetNodeI][targetNodeJ];
            }
        }
        
        if(!startNode.hasWall && !targetNode.hasWall){
            path = this->pathFinder->astar(this->grid, startNode.getIndex(),
                    targetNode.getIndex(), 1, it->first, this->TARGET_ID,
                    cameraResult->arucoIds, cameraResult->arucoCorners);
        }

        /*path = this->pathFinder->astar(this->grid,
                startIndex, {30, 29});*/
        /*path = this->pathFinder->astar(this->grid,
                startNode.getIndex(), {23, 16}, it->first,
                this->TARGET_ID);*/

        /*std::cout << "Start: (" << startIndex.first << ", " << 
            startIndex.second << ")" << std::endl << "Target: (" << 
            targetIndex.first << ", " << targetIndex.second << ")" <<
            std::endl << "Size: " << path.size() << std::endl << std::endl;*/

        if(this->paths.find(it->first) != this->paths.end()){
            this->paths[it->first] = path;
        }else{
            this->paths.insert(std::pair<int, std::vector<Node>>(it->first,
                        path));
        }
        
        this->pathFinder->clearanceLevel = this->CLEARANCE;
    }
}

void PacmanGame::calcRestartPaths(camera_result_t *cameraResult)
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

        Node startNode = this->grid[startNodeI][startNodeJ];
        Node targetNode = this->grid[targetNodeI][targetNodeJ];
        
        for(int i = 0; (startNode.clearance < this->CLEARANCE) && i < 4; i++){
            float startNodeX = it->second.getArucoCorners()[i].x;
            float startNodeY = it->second.getArucoCorners()[i].y;
            startNodeI = (int) std::round(startNodeX/NODE_SIZE);
            startNodeJ = (int) std::round(startNodeY/NODE_SIZE);

            startNode = this->grid[startNodeI][startNodeJ];
        }

        std::vector<Node> path = {};
        
        if(startNode.clearance < this->CLEARANCE){
            this->pathFinder->clearanceLevel = 1;
            startNodeI = (int) std::round(it->second.getCenter().x/NODE_SIZE);
            startNodeJ = (int) std::round(it->second.getCenter().y/NODE_SIZE);
            
            Node startNode = this->grid[startNodeI][startNodeJ];
            
            for(int i = 0; startNode.hasWall && i < 4; i++){
                float startNodeX = it->second.getArucoCorners()[i].x;
                float startNodeY = it->second.getArucoCorners()[i].y;
                startNodeI = (int) std::round(startNodeX/NODE_SIZE);
                startNodeJ = (int) std::round(startNodeY/NODE_SIZE);

                startNode = this->grid[startNodeI][startNodeJ];
            }
        }
        
        if(!startNode.hasWall && !targetNode.hasWall){
            path = this->pathFinder->astar(this->grid, startNode.getIndex(),
                    targetNode.getIndex(), 1, it->first, 0,
                    cameraResult->arucoIds, cameraResult->arucoCorners);
        }

        if(this->paths.find(it->first) != this->paths.end()){
            this->paths[it->first] = path;
        }else{
            this->paths.insert(std::pair<int, std::vector<Node>>(it->first,
                        path));
        }

        
    }
}

std::map<int, Robot> PacmanGame::genCmds(camera_result_t *cameraResult)
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
            cmd = this->cmdGen->generate(it->first, CMD_TURN, {turnAngle, 100});
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
std::string PacmanGame::genPlayerCmd()
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

void PacmanGame::close()
{
    this->scoreManager->close();
}

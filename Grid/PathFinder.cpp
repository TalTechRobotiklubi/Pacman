/* CUSTOM INCLUDES ----------------------------------------------------------*/
#include "PathFinder.hpp"

#include <iostream>

/* METHODS ------------------------------------------------------------------*/
std::vector<Node> PathFinder::astar(std::vector<std::vector<Node>> grid,
                std::pair<int, int> startIndex,std::pair<int, int> targetIndex,
                int isArucoObstacle, int currentId, int targetId,
                std::vector<int> arucoIds,
                std::vector<std::vector<cv::Point2f>> arucoCorners)
{
    
    if(startIndex.first < 0 || startIndex.second < 0 ||
            startIndex.first > grid.size() ||
            startIndex.second > grid[0].size()){
        return {};
    }
    
    if(targetIndex.first < 0 || targetIndex.second < 0 ||
            targetIndex.first >= grid.size() ||
            targetIndex.second >= grid[0].size()){
        return {};
    }
    

    std::vector<Node*> openList;
    std::vector<Node*> closedList;
    
    /* Adding the start node to the open list */ 
    openList.push_back(&grid[startIndex.first][startIndex.second]);
    
    while(openList.size() > 0){
        std::sort(openList.begin(), openList.end(), [](Node *a, Node *b){
            return a->f < b->f;
        });

        Node *currentNode = openList[0];

        if(currentNode->getIndex().first == targetIndex.first && 
                currentNode->getIndex().second == targetIndex.second){
            closedList.push_back(currentNode);
            break;
        }

        openList.erase(openList.begin());
        closedList.push_back(currentNode);
       
        for(auto it = STEPS.begin(); it != STEPS.end(); it++){
            std::pair<int, int> surIndex =
                std::pair<int, int>(currentNode->getIndex().first+it->first,
                                    currentNode->getIndex().second+it->second);

            /* Boundaries check */
            if(surIndex.first < 0 || surIndex.second < 0 ||
                    surIndex.first >= grid.size() ||
                    surIndex.second >= grid[0].size()){
                continue;
            }

            Node *surNode = &grid[surIndex.first][surIndex.second];
            
            float distanceFromAruco = 15.f;
            if(isArucoObstacle && this->PX_TO_CM > 0){
                for(int i = 0; i < arucoIds.size(); i++){
                    if(surNode->tempCheckAruco(arucoIds, arucoCorners) ==
                        currentId){
                        break;
                    }if(arucoIds[i] == currentId || arucoIds[i] == targetId){
                        continue;
                    }

                    cv::Point2f arucoCenter = (arucoCorners[i][0] + 
                            arucoCorners[i][2]) / 2;
                    
                    cv::Point2f diff = arucoCenter - surNode->getCenter();            
                    float distance = std::sqrt(diff.x*diff.x + diff.y*diff.y);
                    distance *= this->PX_TO_CM;

                    if(distance < distanceFromAruco){
                        distanceFromAruco = distance;
                    }
                }
            }

            /* Check if the square is illegal terrain or it is already in the
               closed list */
            if(surNode->hasWall ||
                    (isArucoObstacle && distanceFromAruco < 15.f) ||
                    surNode->clearance < this->clearanceLevel ||
                    this->findNodeInVec(surNode, closedList) != -1){
                continue;
            }
            
            /* Heuristics' calculations */
            int g = 0;
            if(it->first != 0 && it->second != 0){
                g = currentNode->g + 14;
            }else{
                g = currentNode->g + 10;
            }

            /* "Manhattan" distances and h calculation */
            int manDistX = std::abs(targetIndex.first - surIndex.first);
            int manDistY = std::abs(targetIndex.second - surIndex.second);

            int h = (manDistX + manDistY) * 10;
            int f = g + h;
            
            int replaceIndex = this->findNodeInVec(surNode, openList);
            if(replaceIndex != -1 && g < surNode->g){
                surNode->parentIndex = currentNode->getIndex();
                surNode->g = g;
                surNode->h = h;
                surNode->f = f;
                openList[replaceIndex] = surNode;
            }else if(replaceIndex == -1){
                surNode->parentIndex = currentNode->getIndex();
                surNode->g = g;
                surNode->h = h;
                surNode->f = f;
                openList.push_back(surNode);
            }
        }
    }
        
    Node tmpNode = Node(0, 0, 0, 0, targetIndex);
    if(findNodeInVec(&tmpNode, closedList) == -1){
        return {};
    }
    
    int count = 0;
    std::vector<Node> path;
    Node *currentNode = closedList.back();
    while(currentNode->getIndex() != startIndex && count < closedList.size()){
        path.push_back(*currentNode);
        currentNode =
        &grid[currentNode->parentIndex.first][currentNode->parentIndex.second];
        count++;
    }

    std::reverse(path.begin(), path.end());
    return path;
}

int PathFinder::findNodeInVec(Node *n, std::vector<Node*> vec)
{
    for(std::vector<Node*>::iterator it = vec.begin(); it != vec.end(); it++){
        if(n->getIndex().first == (*it)->getIndex().first && 
                n->getIndex().second == (*it)->getIndex().second){
            return std::distance(std::begin(vec), it);
        }
    }

    return -1;
}

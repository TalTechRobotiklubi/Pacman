/* CUSTOM INCLUDES ----------------------------------------------------------*/
#include "GridManager.hpp"

/* METHODS ------------------------------------------------------------------*/
std::vector<std::vector<Node>> GridManager::createGrid(frame_t *frame)
{
    std::vector<std::vector<Node>> grid;
    
    this->gridColumnCount = std::ceil(frame->mat.cols/NODE_SIZE);
    this->gridRowCount = std::ceil(frame->mat.rows/NODE_SIZE);
    
    for(int i = 0; i < this->gridColumnCount; i++){
        std::vector<Node> currentRows;
        for(int j = 0; j < this->gridRowCount; j++){
            currentRows.push_back(
                    Node(i*NODE_SIZE, j*NODE_SIZE, i*NODE_SIZE+NODE_SIZE,
                        j*NODE_SIZE+NODE_SIZE, std::pair<int, int>(i, j))
            );
        }
        grid.push_back(currentRows);
    }    
    
    return grid;
}

std::vector<std::vector<Node>> GridManager::detectWalls(frame_t *frame,
        std::vector<std::vector<Node>> grid, int drawLines)
{
    cv::Ptr<cv::LineSegmentDetector> lsd =
        cv::createLineSegmentDetector(cv::LSD_REFINE_NONE);
    std::vector<cv::Vec4f> lines;
    lsd->detect(frame->mat, lines);
    
    if(drawLines){
        lsd->drawSegments(frame->mat, lines);
    }

    
    for(int i = 0; i < grid.size(); i++){
        for(int j = 0; j < grid[i].size(); j++){
            for(int k = 0; k < lines.size(); k++){
                cv::Vec4f currentVec = lines[k];
                Line currentLine(
                    cv::Point2f(currentVec[0], currentVec[1]),
                    cv::Point2f(currentVec[2], currentVec[3])
                );
                grid[i][j].checkWall(currentLine);
            }
        }
    }
    
    return grid;
}

std::vector<std::vector<Node>> GridManager::fastCheckArucos(
        std::vector<std::vector<Node>> grid, std::vector<int> arucoIds, 
        std::vector<std::vector<cv::Point2f>> arucoCorners)
{
    for(int i = 0; i < grid.size(); i++){
        for(int j = 0; j < grid[i].size(); j++){
            grid[i][j].arucoId = -1;
        }
    }

    for(int i = 0; i < arucoIds.size(); i++){
        for(int j = 0; j < 4; j++){
            int nodeI = (int) std::round(arucoCorners[i][j].x/NODE_SIZE);
            int nodeJ = (int) std::round(arucoCorners[i][j].y/NODE_SIZE);

            if(nodeI > -1 && nodeI < grid.size()){
                if(nodeJ > -1 && nodeJ < grid[nodeI].size()){
                    if(!grid[nodeI][nodeJ].hasWall){
                        grid[nodeI][nodeJ].arucoId = arucoIds[i];
                        continue;
                    }
                }
            }
        }
    }

    return grid;
}

std::vector<std::vector<Node>> GridManager::checkArucos(
        std::vector<std::vector<Node>> grid, std::vector<int> arucoIds, 
        std::vector<std::vector<cv::Point2f>> arucoCorners)
{
    for(int i = 0; i < grid.size(); i++){
        for(int j = 0; j < grid[i].size(); j++){
            grid[i][j].checkAruco(arucoIds, arucoCorners);
        }
    }

    return grid;
}


std::vector<std::vector<Node>> GridManager::addClearance(
        std::vector<std::vector<Node>> grid)
{
    for(int i = 0; i < grid.size(); i++){
        for(int j = 0; j < grid[i].size(); j++){
            if(grid[i][j].hasWall){
                continue;
            }

            int radius = 1;
            int clear = 1;
            while(clear){
                grid[i][j].clearance = radius;
                radius++;
                
                std::vector<std::pair<int, int>> radiusCorners = {
                    std::pair<int, int>(i-radius, j-radius),
                    std::pair<int, int>(i+radius, j-radius),
                    std::pair<int, int>(i+radius, j+radius),
                    std::pair<int, int>(i-radius, j+radius)
                };

                /* TODO: Improve the algorithm by checking the corners first */

                for(int k = 0; k < 4; k++){
                    if(!clear){
                        break;
                    }

                    std::pair<int, int> radiusCorner = radiusCorners[k];

                    if(radiusCorner.first < 0 ||
                            radiusCorner.first >= this->gridColumnCount ||
                            radiusCorner.second < 0 ||
                            radiusCorner.second >= this->gridRowCount){
                        clear = 0;
                        break;
                    }

                    std::pair<int, int> nextRadiusCorner =
                        radiusCorners[(k+1) % 4];
                    
                    int xLen = nextRadiusCorner.first - radiusCorner.first;
                    int yLen = nextRadiusCorner.second - radiusCorner.second;

                    int xDir = (xLen < 0) ? -1 : 1;
                    int yDir = (yLen < 0) ? -1 : 1;

                    for(int l = 0; l < abs(xLen); l++){
                        int currentX = radiusCorner.first + xDir * l;
                        if(currentX < 0 || currentX >= this->gridColumnCount ||
                                grid[currentX][radiusCorner.second].hasWall){
                            clear = 0;
                            break;
                        }
                    }
                    
                    for(int l = 0; l < abs(yLen); l++){
                        int currentY = radiusCorner.second + yDir * l;
                        if(currentY < 0 || currentY >= this->gridRowCount ||
                                grid[radiusCorner.first][currentY].hasWall){
                            clear = 0;
                            break;
                        }
                    }
                }
            }
        }
    }

    return grid;
}

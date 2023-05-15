/* CUSTOM INCLUDES ----------------------------------------------------------*/
#include "Node.hpp"

#include <iostream>

/* METHODS ------------------------------------------------------------------*/
Node::Node(int x1, int y1, int x2, int y2, std::pair<int, int> index)
{
    this->corners = {
        cv::Point2f(x1, y1), /* Top Left */
        cv::Point2f(x2, y1), /* Top Right */
        cv::Point2f(x2, y2), /* Bottom Right */
        cv::Point2f(x1, y2)  /* Bottom Left */
    };
    this->lines = {
        Line(this->corners[0], this->corners[1]), /* Top */
        Line(this->corners[1], this->corners[2]), /* Right */
        Line(this->corners[2], this->corners[3]), /* Bottom */
        Line(this->corners[3], this->corners[0])  /* Left */
    };
    this->index = index;
    this->center = (this->corners[0]+this->corners[2])/2;
}

void Node::checkWall(Line wallLine)
{
    for(int i = 0; i < 4; i++){
        if(this->lines[i].intersects(wallLine, this->index)){
            this->hasWall = 1;
            this->clearance = 0;
            return;
        }
    }
    
    /* Edge case - line starts and ends inside the node */
    cv::Rect2f nodeRect(this->corners[0], this->corners[2]);
    if(nodeRect.contains(wallLine.getFirstPoint()) &&
            nodeRect.contains(wallLine.getSecondPoint())){
        this->hasWall = 1;
        this->clearance = 0;
    }
}

int Node::tempCheckAruco(std::vector<int> arucoIds, 
        std::vector<std::vector<cv::Point2f>> arucoCorners)
{
    if(this->hasWall){
        return -1;
    }

    assert(arucoIds.size() == arucoCorners.size());

    for(int i = 0; i < arucoIds.size(); i++){
        cv::Rect2f arucoRect(arucoCorners[i][0], arucoCorners[i][2]);

        if(arucoRect.contains(this->center) || 
                arucoRect.contains(this->corners[0]) ||
                arucoRect.contains(this->corners[1]) ||
                arucoRect.contains(this->corners[2]) ||
                arucoRect.contains(this->corners[3])){
            return arucoIds[i];
        }
    }
    
    return -1;
}

void Node::checkAruco(std::vector<int> arucoIds, 
        std::vector<std::vector<cv::Point2f>> arucoCorners)
{
    if(this->hasWall){
        this->arucoId = -1;
        return;
    }
    
    int notContainCount = 0;
    int containId = -1;
    for(int i = 0; i < arucoIds.size(); i++){
        cv::Rect2f arucoRect(arucoCorners[i][0], arucoCorners[i][2]);

        if(!arucoRect.contains(this->center) && 
                !arucoRect.contains(this->corners[0]) &&
                !arucoRect.contains(this->corners[1]) && 
                !arucoRect.contains(this->corners[2]) &&
                !arucoRect.contains(this->corners[3])){
                notContainCount++;
        }else{
           this->arucoId = arucoIds[i]; 
           return;
        }
    }

    if(notContainCount == arucoIds.size()){
        this->arucoId = -1;
    }
}
std::vector<cv::Point2f> Node::getCorners()
{
    return this->corners;
}

std::vector<Line> Node::getLines()
{
    return this->lines;
}

cv::Point2f Node::getCenter()
{
    return this->center;
}

std::pair<int, int> Node::getIndex()
{
    return this->index;
}

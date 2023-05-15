/* Line intersection code original source:
 * https://martin-thoma.com/how-to-check-if-two-line-segments-intersect/ */
/* CUSTOM INCLUDES ----------------------------------------------------------*/
#include "Line.hpp"
#include <iostream>

/* METHODS ------------------------------------------------------------------*/
Line::Line(cv::Point2f p1, cv::Point2f p2)
{
    this->p1 = p1;
    this->p2 = p2;
    this->calcBoundingBox();
}

void Line::calcBoundingBox()
{
    /* Joonis siia */
    this->boundingBox.push_back(cv::Point2f(
            std::min(this->p1.x, this->p2.x), 
            std::min(this->p1.y, this->p2.y)));
    this->boundingBox.push_back(cv::Point2f(
            std::max(this->p1.x, this->p2.x),
            std::min(this->p1.y, this->p2.y)));
    this->boundingBox.push_back(cv::Point2f(
            std::max(this->p1.x, this->p2.x),
            std::max(this->p1.y, this->p2.y)));
    this->boundingBox.push_back(cv::Point2f(
            std::min(this->p1.x, this->p2.x),
            std::max(this->p1.y, this->p2.y)));
}

int Line::intersects(Line l, std::pair<int, int> nodeIndex)
{
    int printAnswer = 0;

    if(this->boundingBoxIntersects(l.getBoundingBox()) &&
            this->lineSegmentTouchesOrCrosses(l) && 
            l.lineSegmentTouchesOrCrosses(*this)){
        return 1;
    }

    return 0;
}
    
int Line::lineSegmentTouchesOrCrosses(Line l)
{
    return (int) (this->hasPoint(l.p1) || 
                  this->hasPoint(l.p2) ||
                  (this->isPointRightOfLine(l.p1) ^ 
                   this->isPointRightOfLine(l.p2)));
}

int Line::hasPoint(cv::Point2f p)
{
    Line tmpLine = Line(
            cv::Point2f(0, 0),
            cv::Point2f(this->p2.x - this->p1.x, this->p2.y - this->p1.y));
    cv::Point2f tmpPoint = cv::Point2f(p.x - this->p1.x, p.y - this->p1.y);

    float crossProduct = tmpLine.p2.cross(tmpPoint);

    return (int) (fabs(crossProduct) < 0.000001f);
}

int Line::isPointRightOfLine(cv::Point2f p)
{
    Line tmpLine = Line(
            cv::Point2f(0, 0),
            cv::Point2f(this->p2.x - this->p1.x, this->p2.y - this->p1.y));
    cv::Point2f tmpPoint = cv::Point2f(p.x - this->p1.x, p.y - this->p1.y);

    float crossProduct = tmpLine.p2.cross(tmpPoint);

    return (int) (crossProduct < 0.f);
}

int Line::boundingBoxIntersects(std::vector<cv::Point2f> boundingBox)
{
    if(boundingBox.size() != 4){
        return 0;
    }
    
    return (int) (this->boundingBox[0].x <= boundingBox[2].x &&
            this->boundingBox[2].x >= boundingBox[0].x &&
            this->boundingBox[0].y <= boundingBox[2].y &&
            this->boundingBox[2].y >= boundingBox[0].y);
}

void Line::setFirstPoint(cv::Point2f p1)
{
    this->p1 = p1;
    this->calcBoundingBox();
}

void Line::setSecondPoint(cv::Point2f p2)
{
    this->p2 = p2;
    this->calcBoundingBox();
}

cv::Point2f Line::getFirstPoint()
{
    return this->p1;
}

cv::Point2f Line::getSecondPoint()
{
    return this->p2;
}

std::vector<cv::Point2f> Line::getBoundingBox()
{
    return this->boundingBox;
}

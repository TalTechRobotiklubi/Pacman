/* CUSTOM INCLUDES ----------------------------------------------------------*/
#include "Robot.hpp"

/* METHODS ------------------------------------------------------------------*/
/**
 * Construct a new robot object.
 *
 * Parameters:
 *      id - int, The robot (ArUco code) ID
 *      corners - std::vector<cv::Point2f>, the detected AruCo corners
 *                coordinates that belongs to the robot
 *      target - cv::Point2f, target coordinates
 * 
 * Illustration 1 (0-3 are the ArUco corners given by OpenCV; C - robot 
 * center; M - robot mid point):
 *         0         1
 *          +---M---+
 *          |       |
 *          |   C   |
 *          |       |
 *          +-------+ 
 *         3         2
 *
 * Illustration 2 (C - camera center; R - robot center; T - target
 * center; CB - base vector; RD - robot direction vector; RT - target vector;
 * RD and CB scaled up for better readability of the illustration):
 *          |
 *          | 
 *         B^
 *          |
 *          |
 *          |
 *          |  +-----+   
 *          |  |     |   
 *          |  |  R--|------------>D
 *          |  | /   |          
 *          |  +-----+  
 *         C|  /       
 *--------- - /--------------------------
 *          |/
 *          /
 *         /|
 *        / |
 *  +-----+ |
 *  |   / | |
 *  |  T  | |
 *  |     | |
 *  +-----+ |
 *          |
 *          |
 * 
 * Info about the class variables:
 *      undetectedCount - int, public, How many frames has been the robot
 *                        undetected by the camera (see the undetection
 *                        constants in Camera.hpp and also Camera::update)
 *      id - int, protected, The robot (ArUco code) ID 
 *      center - cv::Point2f, protected, The robot center point (the C in the
 *               Illustration 1)
 *      mid - cv::Point2f, protected, The robot mid point (the M in the
 *            Illustration 1)
 *      target - cv::Point2f, protected, The target robot point (the T in 
 *            the Illustration 2)
 *      dirVec - cv::Vec2f, protected, The robot direction vector (vector CM in
 *               the Illustration 1 and vector RD in the Illustration 2)
 *      targetVec - cv::Vec2f, protected, The vector from robot center to 
 *                  target (center; the vector RT in the Illustration 2)
 *      dirVecLen - float, protected, The robot direction vector length (the
 *                  length of the vector CM in the Illustration 1)
 *      targetVecLen - float, protected, The target vector length (i.e. the
 *                     target distance from the robot; the vector RT length
 *                     in the Illustration 2)
 *      angle - float, protected, Robot angle; the angle between the base
 *              vector (CB in the Illustration 2) and robot direction vector
 *              (RD in the Illustration 2)
 *      targetAngle - float, protected, The angle between the robot and the
 *                    target (with direction - positive for clockwise and
 *                    negative for counter clockwise); the angle between
 *                    the vectors RD and RT in the Illustration 2
 *      lastTargetAngle - float, protected, The target angle before current
 *                        target angle (used for calculation in the
 *                        Robot::calcMotorPowers)
 *      motorPowers - std::pair<int, int>, protected, The motor powers for chasing
 *                    down the target
 *
 * The general procedure for getting data from this class is the following (if
 * not stated otherwise in some methods):
 *   * Call Robot::update
 *   * Use getters to get the desired value(s)
 */
Robot::Robot(const int id, const std::vector<cv::Point2f> corners,
        const int maxPwr, const float pMultPID, const float dMultPID)
{
    this->id = id;
    this->maxPwr = (abs(maxPwr) > 1000) ? 1000 : abs(maxPwr);
    this->pMultPID = pMultPID;
    this->dMultPID = dMultPID;
    if(id != -1){
        this->undetectedCount = 0;
        this->update(corners);
    }
}

/**
 * Update the robot properties (angle, target angle, direction vector, target
 * vector etc.)
 *
 * Parameters:
 *      corners - std::vector<cv::Point2f>, The updated AruCo corners'
 *                coordinates that belongs to the robot
 *      target - cv::Point2f, Updated target coordinates
 *
 * ArUco corners in the code are indexed in the following way:
 *         0         1
 *          +-------+
 *          | The   |
 *          | ArUco |
 *          | code  |
 *          +-------+ 
 *         3         2
 *
 * Returns: Does not return anything but updates the robot instance properties,
 *          use getters to get these values
 */
void Robot::update(const std::vector<cv::Point2f> corners)
{
    this->arucoCorners = corners;
    this->center = (corners[0] + corners[2]) / 2.f;
    this->mid = (corners[0] + corners[1]) / 2.f;

    this->dirVec = cv::Vec2f(this->mid.x - this->center.x,
            this->mid.y - this->center.y);
    this->dirVecLen = std::sqrt(std::pow(this->dirVec[0], 2.f) +
            std::pow(this->dirVec[1], 2.f));

    this->angle = this->calcAngle();
}

void Robot::updateTarget(const cv::Point2f target)
{
    this->target = target;
    this->targetVec = cv::Vec2f(target.x - this->center.x, 
           target.y - this->center.y);
    this->targetVecLen = std::sqrt(std::pow(this->targetVec[0], 2.f) +
            std::pow(this->targetVec[1], 2.f));
    
    this->lastTargetAngle = this->targetAngle;
    this->targetAngle = this->calcTargetAngle();
    
    this->motorPowers = this->calcMotorPowers();
}

/**
 * Calculate the robot angle relative to the base vector (see Robot.hpp for the
 * base vector). NOTE: This method is protected and also uses instance
 * (protected) properties (dirVec to be precise) for the calculation - to get
 * the calculated value, just call Robot::update and then use the
 * Robot::getAngle getter for getting the most recent angle.
 *
 * Calculation is just based on the robot direction vector and base vector
 * scalar.
 *
 * Illustration:
 *          |
 *          | 
 *         B^
 *          |
 *          |
 *          |
 *          |  +-----+   
 *          |  |     |   
 *          |  |  R--|--->D
 *          |  |     |          
 *          |  +-----+  
 *         C|         
 *--------- - ---------------------------
 *          |           
 *          |            
 *          |
 *          |
 *          |
 *          |
 *          |
 *          |
 * C is the camera center; CB is the base vector and RD is the robot
 * direction vector. The robot angle is equal to the angle between the vectors
 * CB and RD (MATH NOTE: we can move vectors in the coordinate system in any
 * direction as long as we do not change the direction, angle and size of the
 * vector)
 *
 *
 * Returns: float, The robot angle in degrees based on the robot direction
 *          vector and base vector.
 */
float Robot::calcAngle()
{
    const float denominator = this->dirVec[0] * baseVec[0] +
        this->dirVec[1] * baseVec[1];
    const float numerator = this->dirVecLen * baseVecLen;
    
    if(denominator == 0){
        return 0;
    }

    const float rads = std::acos(denominator / numerator);
    const float degs = (rads * 180.f / float(M_PI));
    
    return (this->dirVec[0] < 0.f) ? 180.f + degs : 180.f - degs;
}

/**
 * Calculate the angle between the robot and the target. NOTE: This method is
 * protected and also uses (protected) properties of the instance (targetVec
 * and robot angle to be precise) for the calculation - to get the calculated
 * value, just call Robot::update and then use the Robot::getTargetAngle getter
 * for getting the most recent angle.
 *
 * The calculation is quite simple - first we find the angle between the target
 * vector (that is the vector from robot center to target (center); target
 * vector is already found in the update method) and base vector. Second, we
 * subtract the robot angle from the target angle to get the answer with the
 * right direction (positive values are clockwise and negative values counter
 * clockwise).
 *
 * Illustration for the calculation:
 *          |
 *          | 
 *         B^
 *          |
 *          |
 *          |
 *          |  +-----+   
 *          |  |     |   
 *          |  |  R--|------------>D
 *          |  | /   |          
 *          |  +-----+  
 *         C|  /       
 *--------- - /--------------------------
 *          |/
 *          /
 *         /|
 *        / |
 *  +-----+ |
 *  |   / | |
 *  |  T  | |
 *  |     | |
 *  +-----+ |
 *          |
 *          |
 * The C is the camera center - the base vector is CB; robot direction
 * vector is RD and target vector is RT. We want to get the angle (and turning
 * direction) between the vectors RD and RT. First, we calculate the angle
 * between the vectors CB and RT (NOTE: we can move vectors in the coordinate
 * system in any direction as long as we do not change the direction, angle and
 * size of the vector). Then we subtract the robot angle (the angle between
 * vectors CB and RD) from the calculated angle to get the answer. NOTE: It
 * might seem easier just to calculate the angle between vectors RD and RT but
 * then we would not get the right direction as we get with the subtraction.
 *
 *
 * Returns: float, The angle (and direction) between the robot and target
 *          (positive values are clockwise and negative counter clockwise)
 */
float Robot::calcTargetAngle()
{

    /* Calculate the angle between the base vector and target vector */
    const float denominator = this->targetVec[0] * baseVec[0] + 
        this->targetVec[1] * baseVec[1];
    const float numerator = this->targetVecLen * baseVecLen;

    float rads = 0.f;
    if(numerator != 0.f){
        rads = std::acos(denominator / numerator);
    }

    float degs = (rads * 180.f/float(M_PI));
    degs = (this->targetVec[0] < 0.f) ? 180.f + degs : 180.f - degs;
    
    /* Calculate the angle (and direction) between the robot and target */
    float turnDegs = degs - this->angle;
    
    if(turnDegs > 180.f){
        turnDegs = -360+turnDegs;
    }else if(turnDegs < -180.f){
        turnDegs = 360+turnDegs;
    }

    return turnDegs;
}

/**
 * Calculate the motor PID controlled motor powers for chasing the target.
 * NOTE: This method is protected and also uses (protected) properties of the
 * instance for the calculation - to get the calculated value, just call
 * Robot::update and then use the Robot::getMotorPowers getter.
 *
 * Returns: std::pair<int, pair>, The motor powers (index 0 is left motor power;
 *          index 1 is right motor power)
 */
std::pair<int, int> Robot::calcMotorPowers()
{
    /* PID (Proportional Integral Derative) control */
    float u = this->pMultPID * this->targetAngle +
              this->dMultPID * (this->lastTargetAngle - this->targetAngle);
    
    /* Apply the PID control */
    float powerLeft = maxPwr + u;
    float powerRight = maxPwr - u;
    
    /* Limit the powers */ 
    if(powerLeft > maxPwr){
        powerLeft = maxPwr;
    }else if(powerLeft < -maxPwr){
        powerLeft = -maxPwr;
    }
    if(powerRight > maxPwr){
        powerRight = maxPwr;
    }else if(powerRight < -maxPwr){
        powerRight = -maxPwr;
    }

    std::pair<int, int> powers;

    if(u > 0){
        /* Turning left */
        powers = std::pair<int, int> (maxPwr, (int) (round(powerRight)));
        return powers;
    }else{
        /* Turning right */
        powers = std::pair<int, int> ((int) (round(powerLeft)), maxPwr);
        return powers;
    }
}

/**
 * Get robot ID
 *
 * Returns: int, Robot ID
 */
int Robot::getId()
{
    return this->id;
}

/**
 * Get robot ArUco corners
 *
 * Returns: std::vector<cv::Point2f>, Aruco corners, where index 0 is top left,
 *          index 1 is top right, index 2 is bottom right and index 3 is
 *          bottom left corner.
 */
std::vector<cv::Point2f> Robot::getArucoCorners()
{
    return this->arucoCorners;
}

/**
 * Get robot center point
 * 
 * If this is our ArUco code (where 0-3 are the ArUco corners given by OpenCV):
 *         0         1
 *          +---M---+
 *          |       |
 *          |   C   |
 *          |       |
 *          +-------+ 
 *         3         2
 * then the robot center is C
 * 
 * Returns: cv::Point2f, Robot center
 */
cv::Point2f Robot::getCenter()
{
    return this->center;
}

/**
 * Get robot mid point
 * 
 * Illustration (0-3 are the ArUco corners given by OpenCV):
 *         0         1
 *          +---M---+
 *          |       |
 *          |   C   |
 *          |       |
 *          +-------+ 
 *         3         2
 * then the robot mid is M
 *
 * Returns: cv::Point2f, Robot mid
 */
cv::Point2f Robot::getMid()
{
    return this->mid;
}

/**
 * Get robot target coordinates
 *
 * Returns: cv::Point2f, Target coordinates
 */
cv::Point2f Robot::getTarget()
{
    return this->target;
}

/**
 * Get robot direction vector
 *
 * Illustration (0-3 are the ArUco corners given by OpenCV):
 *         0         1
 *          +---M---+
 *          |       |
 *          |   C   |
 *          |       |
 *          +-------+ 
 *         3         2
 * then the robot direction vector is the vector made from the points C and M
 *
 * Returns: cv::Vec2f, Robot direction vector
 */
cv::Vec2f Robot::getDirVec()
{
    return this->dirVec;
}

/**
 * Ger robot direction vector length
 *
 * Illustration (0-3 are the ArUco corners given by OpenCV):
 *         0         1
 *          +---M---+
 *          |       |
 *          |   C   |
 *          |       |
 *          +-------+ 
 *         3         2
 * then the robot direction vector is the vector made from the points C and M
 *
 * Returns: float, Robot direction vector length in pixels
 */
float Robot::getDirVecLen()
{
    return this->dirVecLen;
}

/**
 * Get target vector
 *
 * Illustration (C - camera center; R - robot center; T - target (center);
 * CB - base vector; RD - robot direction vector; RT - target vector):
 *          |
 *          | 
 *         B^
 *          |
 *          |
 *          |
 *          |  +-----+   
 *          |  |     |   
 *          |  |  R--|------------>D
 *          |  | /   |          
 *          |  +-----+  
 *         C|  /       
 *--------- - /--------------------------
 *          |/
 *          /
 *         /|
 *        / |
 *  +-----+ |
 *  |   / | |
 *  |  T  | |
 *  |     | |
 *  +-----+ |
 *          |
 *          |
 *
 * Returns: cv::Vec2f, Target vector (RT in the illustration)
 */
cv::Vec2f Robot::getTargetVec()
{
    return this->targetVec;
}

/**
 * Get the target vector length (i.e. the target distance from the robot)
 * 
 * Illustration (C - camera center; R - robot center; T - target (center);
 * CB - base vector; RD - robot direction vector; RT - target vector):
 *          |
 *          | 
 *         B^
 *          |
 *          |
 *          |
 *          |  +-----+   
 *          |  |     |   
 *          |  |  R--|------------>D
 *          |  | /   |          
 *          |  +-----+  
 *         C|  /       
 *--------- - /--------------------------
 *          |/
 *          /
 *         /|
 *        / |
 *  +-----+ |
 *  |   / | |
 *  |  T  | |
 *  |     | |
 *  +-----+ |
 *          |
 *          |
 *
 * Returns: float, Target distance (RT in the illustration) in pixels
 */
float Robot::getTargetVecLen()
{
    return this->targetVecLen;
}

/**
 * Get the robot angle from to the base vector (see Robot.hpp for the
 * base vector). For more details on the calculation, see the 
 * Robot::calcAngle method.
 * 
 * Illustration (C - camera center; R - robot center; T - target (center);
 * CB - base vector; RD - robot direction vector; RT - target vector):
 *          |
 *          | 
 *         B^
 *          |
 *          |
 *          |
 *          |  +-----+   
 *          |  |     |   
 *          |  |  R--|------------>D
 *          |  | /   |          
 *          |  +-----+  
 *         C|  /       
 *--------- - /--------------------------
 *          |/
 *          /
 *         /|
 *        / |
 *  +-----+ |
 *  |   / | |
 *  |  T  | |
 *  |     | |
 *  +-----+ |
 *          |
 *          |
 *
 * Returns: float, Robot angle (the angle between the base vector and robot
 *          direction vector) in degrees
 */
float Robot::getAngle()
{
    return this->angle;
}

/**
 * Get the angle (and direction) between the robot and the target. For more
 * details on the calculation, see Robot::calcTargetAngle method.
 * 
 * Illustration (C - camera center; R - robot center; T - target (center);
 * CB - base vector; RD - robot direction vector; RT - target vector):
 *          |
 *          | 
 *         B^
 *          |
 *          |
 *          |
 *          |  +-----+   
 *          |  |     |   
 *          |  |  R--|------------>D
 *          |  | /   |          
 *          |  +-----+  
 *         C|  /       
 *--------- - /--------------------------
 *          |/
 *          /
 *         /|
 *        / |
 *  +-----+ |
 *  |   / | |
 *  |  T  | |
 *  |     | |
 *  +-----+ |
 *          |
 *          |
 *
 * Returns: float, The angle (and the dire
 */
float Robot::getTargetAngle()
{
    return this->targetAngle;
}

/**
 * Get motor powers for chasing the target
 *
 * Returns: std::vector<int>, The motor powers (index 0 is left motor power;
 *          index 1 is right motor power)
 */
std::pair<int, int> Robot::getMotorPowers()
{
    return this->motorPowers;
}

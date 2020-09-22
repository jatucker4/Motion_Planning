//
// Created by Johnathan Tucker on: 9/16/2020
// Purpose: Create class for bug algorithms
// Structure: The methods are used to navigate the bugs around a given workspace
//
//
#ifndef INC_5519_BUGHW2_BUG_H
#define INC_5519_BUGHW2_BUG_H

#include <string>
#include <cmath>
#include <iostream>
#include "Workspace.h"

using namespace std;
// Create Bug Class
class Bug {
public:
    Bug(string bugType, Workspace &workspace,tuple<double,double> qStart, tuple<double,double> qGoal);
    vector<tuple<double,double>> bugPath;
    tuple<double,double> q0;
    tuple<double,double> qG;
    tuple<double,double> currentLoc;
    double distTraveled;
    tuple<double,double> botHeading;
    double velMag;
    double sensorLength;
    Workspace *workObj;
    double step;
    double scalingFactor;
    string bugNum;
    tuple<double,double> collideCoords;
    tuple<double,double> locNearest;
    bool explored;
    bool explore;
    double distGoalTrack;
    bool nearCollide;
    tuple<double,double> closeLoc;
    double distToGoalFromStart;

    // Declare class methods necessary for implementing the bug algorithms
    string getCollisionStatus(tuple<double,double> robotLoc);
    tuple<double,double,string> moveForward(tuple<double,double> heading, double delt);
    bool distCheck(tuple<double,double> targetLocation);
    double distance(tuple<double,double> target);
    void rotate(double angle);
    tuple<double,double> getSensLoc(tuple<double,double> robotLoc);
    void setRobotState(tuple<double,double> newRobotState);
    static double vectorMag(tuple<double,double> vec);
    static tuple<double,double> vectorDiff(tuple<double,double>vec1, tuple<double,double>vec2);
    static tuple<double,double> normalizeVec(double x ,double y);
    tuple<double,double> getHeading(tuple<double,double> currPos, tuple<double,double> targetObjPos);
    double sendToGoal();
    bool bugHit();
    void followObstacle();
    bool getBugStatus();
    bool MLine();
    void nextObstacle();
};

Bug::Bug(string bugType, Workspace &workspace, tuple<double, double> qStart, tuple<double, double> qGoal) {
    // Declare class variables
    sensorLength = 0.1;
    velMag = 1;
    q0 = qStart;
    qG = qGoal;
    workObj = &workspace;
    bugNum = bugType;
    currentLoc = q0;
    bugPath.push_back(currentLoc);
    distTraveled = 0;
    step = 0.1;
    scalingFactor = 10;
    collideCoords = make_tuple(1000, 1000);
    locNearest = make_tuple(1000, 1000);
    explored = false;
    explore = false;
    distGoalTrack = 100000;
    nearCollide = true;
    closeLoc = make_tuple(1000,1000);
    distToGoalFromStart = 0.0;
}

// Function to get the collision status of the robot
string Bug::getCollisionStatus(tuple<double, double> robotLoc) {
    string status;
    tuple<double,double> sensorLoc = getSensLoc(robotLoc);

    bool robotHit = workObj->checkCollision(robotLoc);
    bool sensorHit = workObj->checkCollision(sensorLoc);

    if(!robotHit && !sensorHit){
        status = "nohit";
    }else if(!robotHit && sensorHit) {
        status = "hit";
    }
    return status;
}
// Class method for moving the bug forward at some desired rate
tuple<double, double, string> Bug::moveForward(tuple<double,double> heading, double delt) {
    double deltaX = velMag * get<0>(heading) * delt;
    double deltaY = velMag * get<1>(heading) * delt;

    double newXPos = get<0>(currentLoc) + deltaX;
    double newYPos = get<1>(currentLoc) + deltaY;

    tuple<double,double> newRobotLoc(newXPos,newYPos);

    string status = getCollisionStatus(newRobotLoc);

    tuple<double, double, string> locStatus(newXPos,newYPos,status);

    return locStatus;
}
// Check how far the robot is from a desired location
bool Bug::distCheck(tuple<double, double> targetLocation) {
    double distToLoc = distance(targetLocation);
    bool isClose = (distToLoc <= sensorLength);
    return isClose;
}
// Check how
tuple<double, double> Bug::normalizeVec(double x, double y) {
    tuple<double,double> vec(x,y);
    double mag = vectorMag(vec);

    double newX = x/mag;
    double newY = y/mag;
    tuple<double, double>normVec(newX,newY);

    return normVec;
}

double Bug::vectorMag(tuple<double, double> vec) {
    double x = get<0>(vec);
    double y = get<1>(vec);
    double exponent = 2.0;
    double magnitude = sqrt( pow(x, exponent) + pow(y,exponent));
    return magnitude;
}

tuple<double, double> Bug::vectorDiff(tuple<double, double> vec1, tuple<double, double> vec2) {
    double diffX = get<0>(vec2) - get<0>(vec1);
    double diffY = get<1>(vec2) - get<1>(vec1);
    tuple<double, double> vecDiff(diffX, diffY);

    return vecDiff;
}

double Bug::distance(tuple<double, double> target) {
    tuple<double,double> vecDiff = vectorDiff(target, currentLoc);
    double distToLocation = vectorMag(vecDiff);

    return distToLocation;
}

tuple<double, double> Bug::getHeading(tuple<double, double> currPos, tuple<double, double> targetObjPos) {
    tuple<double,double> tempHeading = vectorDiff(currPos, targetObjPos);
    tuple<double,double> normTempHeading = normalizeVec(get<0>(tempHeading),get<1>(tempHeading));

    return normTempHeading;
}
// This method allows the bug to rotate by some desired angle
void Bug::rotate(double angle) {
    tuple<double,double> currHeading = botHeading;
    double currHeadX = get<0>(currHeading);
    double currHeadY = get<1>(currHeading);

    double c = cos(angle);
    double s = sin(angle);

    double newHeadX = (currHeadX * c) - (currHeadY * s);
    double newHeadY = (currHeadY * c) + (currHeadX * s);

    tuple<double,double> newBotHead = normalizeVec(newHeadX,newHeadY);

    botHeading = newBotHead;

}
// This method gets the tactile sensors location
tuple<double, double> Bug::getSensLoc(tuple<double, double> robotLoc) {
    tuple<double,double> currHeading = botHeading;
    double currHeadX = get<0>(currHeading);
    double currHeadY = get<1>(currHeading);

    double currLocX = get<0>(robotLoc);
    double currLocY = get<1>(robotLoc);

    double sensorX = currLocX + (currHeadX * sensorLength);
    double sensorY = currLocY + (currHeadY * sensorLength);

    tuple<double,double> sensorLoc(sensorX,sensorY);

    return sensorLoc;
}

void Bug::setRobotState(tuple<double, double> newRobotState) {
    double currDistTraveled = distance(newRobotState);
    distTraveled += currDistTraveled;

    currentLoc = newRobotState;
    bugPath.push_back(newRobotState);
}
// This method kicks of the bug to head towards goal
double Bug::sendToGoal() {
    // Move until you hit an obstacle
    while(!distCheck(qG)){
        bool bugHitWall = bugHit();
        // If you hit an obstacle navigate it
        if(bugHitWall){
            followObstacle();
        }
    }
    return distTraveled;
}

bool Bug::bugHit() {
    // The role of this method is to determine if the bug hit an obstacle
    tuple<double,double> targetHeading = getHeading(currentLoc, qG);
    botHeading = targetHeading;
    double delta = step;
    tuple<double, double> currHeading = botHeading;
    tuple<double, double> currRobLoc = currentLoc;
    // Check the initial collision status at the current bug heading
    string status = getCollisionStatus(currRobLoc);

    while (status != "hit") {
        if (distCheck(qG)) {
            // If the bug is at goal return false
            return false;
        }
        tuple<double, double, string> robLocStatus = moveForward(currHeading, delta);
        status = get<2>(robLocStatus);
        tuple<double, double> newRobLoc(get<0>(robLocStatus), get<1>(robLocStatus));
        // If it hits doesn't het an obstacle set the new robot state and take a step forward
        if (status == "nohit") {
            currRobLoc = newRobLoc;
            setRobotState(newRobLoc);
            delta = step;
            // If it does hit set the robot state and return that it hit.
        } else if (status == "hit") {
            currRobLoc = newRobLoc;
            setRobotState(newRobLoc);
            return true;
        }
    }
}

void Bug::followObstacle(){
    // The purpose of this method is to follow the recently hit obstacle
    double delta_T = step;
    // The bug status function is used to determine if the bug should leave the object
    while(!getBugStatus()){
        rotate(M_PI);
        string status = getCollisionStatus(currentLoc);
        // If we took a step and it wasn't into the obstacle then continue
        while(status != "hit"){
            if(bugNum == "bug2"){
                double angle = -1.0 * ((2*M_PI)/90);
                rotate(angle);

                status = getCollisionStatus(currentLoc);
            }else{
                double angle = (2*M_PI)/90;
                rotate(angle);

                status = getCollisionStatus(currentLoc);
            }
        }
        // If we toook a step into the obstacle then turn
        while(status != "nohit"){
            if(bugNum == "bug2"){
                double angle = (2*M_PI)/90;
                rotate(angle);

                status = getCollisionStatus(currentLoc);
            }else{
                double angle = -1.0 * ((2*M_PI)/90);
                rotate(angle);

                status = getCollisionStatus(currentLoc);
            }
        }
        tuple<double,double> currHeading = botHeading;
        tuple<double,double,string> newLocStat = moveForward(currHeading, delta_T);

        tuple<double,double> newRobLoc(get<0>(newLocStat),get<1>(newLocStat));

        setRobotState(newRobLoc);
    }
}

bool Bug::getBugStatus() {

    if(bugNum == "bug1"){
        tuple<double,double> currLoc = currentLoc;
        double distGoal = distance(qG);

        if((get<0>(locNearest) == 1000) && (get<1>(locNearest) == 1000) ) {
            collideCoords = currLoc;
            locNearest = currLoc;
        }
        if (distGoal < distGoalTrack) {
            distGoalTrack = distGoal;
            locNearest = currLoc;
        }
        tuple<double,double> startLoc = collideCoords;
        nearCollide = distCheck(startLoc);

        if (!explore && !nearCollide) {
            explore = true;
        }else if(explore && nearCollide){
            explored = true;
        }
        closeLoc = locNearest;
        if (explored && distCheck(closeLoc)) {
            nextObstacle();
            return true;
        }else {
            return false;
        }

    }else{

        tuple<double,double> currLoc = currentLoc;
        if((get<0>(collideCoords) == 1000) && (get<1>(collideCoords) == 1000) ) {
            collideCoords = currLoc;
            distToGoalFromStart = distance(qG);
            return false;
        }
        tuple<double,double> firstEncountered = collideCoords;
        double currDist = distance(qG);
        bool closeToGoal = currDist < distToGoalFromStart;

        bool notAtStart = !distCheck(firstEncountered);

        if(MLine() && closeToGoal && notAtStart) {
            nextObstacle();
            return true;
        }
        return false;
    }
}

bool Bug::MLine(){
    tuple<double,double> startLoc = q0;
    tuple<double,double> goalLoc = qG;
    double startLocY = get<1>(startLoc);

    tuple<double,double> headingToGoal = getHeading(startLoc, goalLoc);
    double mLineSlope = (get<1>(headingToGoal)) / (get<0>(headingToGoal));

    double robotX = get<0>(currentLoc);

    double mLineY = startLocY + robotX * mLineSlope;
    double mLineX = robotX;

    tuple<double,double> mLinePosAtRobotX (mLineX, mLineY);
    bool check = distCheck(mLinePosAtRobotX);
    return check;
}

void Bug::nextObstacle() {
    collideCoords = make_tuple(1000, 1000);
    locNearest = make_tuple(1000, 1000);
    explored = false;
    explore = false;
    distGoalTrack = 100000;
    nearCollide = true;
    closeLoc = make_tuple(1000,1000);
}

#endif //INC_5519_BUGHW2_BUG_H
//
// Created by John on 11/3/2020.
//

#ifndef INC_5519_HW4_GOALRRT_H
#define INC_5519_HW4_GOALRRT_H
#include "PRMPlanner.h"

class GoalRRT {
public:
    GoalRRT(CSpace2D &myspace, int numIters, double radius, tuple<double,double> qStart, tuple<double,double> qgoal,
            double goalProb, double goalTerm);
//    void sampleSpace();
    double numConnections();
//    void reconstructPath();
//    void connectSamples();
    void makeMap();
    void findPathToGoal();
    void RRT();
    static bool compareFunc(pair<int,double> &a, pair<int,double> &b);
    bool doIntersect(tuple<double,double> p1, tuple<double,double> q1, tuple<double,double> p2, tuple<double,double> q2);
    bool checkCollisionPath(vector<tuple<double,double>> coords);
    bool checkCollision(tuple<double,double> node);
    double orientation(tuple<double,double> p, tuple<double,double> q, tuple<double,double> r);
    bool onSegment(tuple<double,double> p, tuple<double,double> q, tuple<double,double> r);
    double distanceToNode(tuple<double, double> target1, tuple<double,double> target2);
    tuple<double, double> vectorDiff(tuple<double, double> vec1, tuple<double, double> vec2);
    double vectorMag(tuple<double, double> vec);
    void makePlan();
    tuple<double, double> vectorAdd(tuple<double, double> tuple2, tuple<double, double> tuple1);

    bool flag;
    int maxIters;
    tuple<int,int> yBounds;
    tuple<int,int> xBounds;
    vector<tuple<double,double>> nodeIters;
    vector<tuple<double,double>> nodePath;
    vector<tuple<int,int>> cameFrom;
    CSpace2D *mySpace2D;
    tuple<double,double> qstart;
    double epsilon;
    tuple<double,double> qgoal;
    ArrayXXd nodeMap;
    double r;
    int flag2;
    double pGoal;
    VectorXd ofInterest;
    vector<tuple<double,double,double,double>> nodeConns;
    tuple<double,double> goalBiasRange;


};

GoalRRT::GoalRRT(CSpace2D &myspace, int numIters, double radius, tuple<double, double> qStart,
                       tuple<double, double> qGoal, double goalProb, double goalTerm) {
    epsilon = goalTerm;
    maxIters = numIters;
    pGoal = goalProb;
    r = radius;
    yBounds = myspace.yBound;
    xBounds = myspace.xBound;
    mySpace2D =&myspace;
    qstart = qStart;
    qgoal = qGoal;
    flag = true;
    goalBiasRange = tuple<double,double>(0.05,0.1);
    flag2 = 1;
    makePlan();
}
//
//void GoalRRT::sampleSpace() {
//    random_device rd;
//    mt19937 eng(rd());
//    uniform_real_distribution<> xdist(get<0>(xBounds), get<1>(xBounds));
//    uniform_real_distribution<> ydist(get<0>(yBounds), get<1>(yBounds));
//    for(int i = 0; i <= numSamples; i++){
//        tuple<double,double> temp(xdist(eng),ydist(eng));
//        nodeIters.push_back(temp);
//    }
//}

double GoalRRT::distanceToNode(tuple<double, double> target1, tuple<double,double> target2) {
    tuple<double,double> vecDiff = vectorDiff(target1, target2);
    double distToLocation = vectorMag(vecDiff);

    return distToLocation;
}

double GoalRRT::vectorMag(tuple<double, double> vec) {
    double x = get<0>(vec);
    double y = get<1>(vec);
    double exponent = 2.0;
    double magnitude = sqrt( pow(x, exponent) + pow(y,exponent));
    return magnitude;
}

tuple<double, double> GoalRRT::vectorDiff(tuple<double, double> vec1, tuple<double, double> vec2) {
    double diffX = get<0>(vec2) - get<0>(vec1);
    double diffY = get<1>(vec2) - get<1>(vec1);
    tuple<double, double> vecDiff(diffX, diffY);

    return vecDiff;
}

bool GoalRRT::checkCollisionPath(vector<tuple<double,double>> coords) {
    // the variable coords contains the origin locations for each link;
    tuple<double,double> link1Origin = coords[0];
    tuple<double,double> link2Origin = coords[1];
    for(auto obstacle : mySpace2D->obstacles){
        // If one of the obstacles is a triangle the primitives need to be created in a special way.
        // Else it's a rectangle and we can do as before
        tuple<double,double> obsVert1 = obstacle[0];
        tuple<double,double> obsVert2 = obstacle[1];
        tuple<double,double> obsVert3 = obstacle[2];
        tuple<double,double> obsVert4 = obstacle[3];

        bool prim1 = doIntersect(link1Origin,link2Origin, obsVert1, obsVert2);
        bool prim2 = doIntersect(link1Origin,link2Origin, obsVert2,obsVert3);
        bool prim3 = doIntersect(link1Origin,link2Origin, obsVert3,obsVert4);
        bool prim4 = doIntersect(link1Origin,link2Origin, obsVert4,obsVert1);

        if(prim1 || prim2 || prim3 || prim4){
            return true;
        }
    }
    return false;
}
bool GoalRRT::onSegment(tuple<double,double> p, tuple<double,double> q, tuple<double,double> r){

    if (get<0>(q) <= max(get<0>(p), get<0>(r)) && get<0>(q) >= min(get<0>(p), get<0>(r)) &&
        get<1>(q) <= max(get<1>(p), get<1>(r)) && get<1>(q) >= min(get<1>(p), get<1>(r))){
        return true;
    }
    return false;
}

double GoalRRT::orientation(tuple<double,double> p, tuple<double,double> q, tuple<double,double> r){
    double val = (get<1>(q) - get<1>(p)) * (get<0>(r) - get<0>(q)) -
                 (get<0>(q) - get<0>(p)) * (get<1>(r) - get<1>(q));

    if (val == 0) return 0;  // colinear

    return (val > 0)? 1: 2; // clock or counterclock wise
}

bool GoalRRT::doIntersect(tuple<double,double> p1, tuple<double,double> q1, tuple<double,double> p2, tuple<double,double> q2){
    // Simple intersection function pulled from tutorials.org
    // Find the four orientations needed for general and
    // special cases
    double o1 = orientation(p1, q1, p2);
    double o2 = orientation(p1, q1, q2);
    double o3 = orientation(p2, q2, p1);
    double o4 = orientation(p2, q2, q1);

    if (o1 != o2 && o3 != o4)
        return true;

    // Special Cases
    // p1, q1 and p2 are colinear and p2 lies on segment p1q1
    if (o1 == 0 && onSegment(p1, p2, q1)) return true;

    // p1, q1 and q2 are colinear and q2 lies on segment p1q1
    if (o2 == 0 && onSegment(p1, q2, q1)) return true;

    // p2, q2 and p1 are colinear and p1 lies on segment p2q2
    if (o3 == 0 && onSegment(p2, p1, q2)) return true;

    // p2, q2 and q1 are colinear and q1 lies on segment p2q2
    if (o4 == 0 && onSegment(p2, q1, q2)) return true;
    return false; // Doesn't fall in any of the above cases
}

bool GoalRRT::checkCollision(tuple<double,double> node) {
    //Iterate through each obstacle, construct primitaves and check if there was a collision
    double xLoc = get<0>(node);
    double yLoc = get<1>(node);
    for(auto obstacle : mySpace2D->obstacles){
        double bottom = get<1>(obstacle[0]);
        double right = get<0>(obstacle[1]);
        double top = get<1>(obstacle[2]);
        double left = get<0>(obstacle[3]);

        if(((yLoc >= bottom) && (xLoc >= right) && (yLoc <= top) && (xLoc <= left))){
            return true;
        }
    }
    return false;
}

void GoalRRT::findPathToGoal() {
///////////////////////LETS ATTEMPT A*///////////////////////////////
    tuple<double,double> nBest = qstart;
    vector<double> valueCopy1;
    vector<double> valueCopy2;
    vector<int> queueNodeIndex;
    vector<double> queueNodeValue;
    vector<pair<int,double>> openList;
    vector<double> gOfNVector;
    vector<tuple<double,double>> closedParentIndex;
    double gOfNParentDist;
    vector<tuple<double,double>> closedList;
    VectorXd rowOfInterest;
    int iter = 0;
    closedList.push_back(qstart);
    double temp;
    // left index is parent, right index is child
    double gN = 0;
    vector<tuple<double,double>>::iterator itClosed;
    vector<pair<int,double>>::iterator itOpen;
    int indexOfInterest = 0;
    pair<int,double> tempPair;
    rowOfInterest = nodeMap.row(indexOfInterest);
    //Iterate through the row of interest to get all of the connected nodes indices and weights;
    for(int i = 0; i!=rowOfInterest.size();i++) {
        itClosed = find(closedList.begin(),closedList.end(),nodeIters[i]);
        if(rowOfInterest[i] != 0  && itClosed==closedList.end()){
            //Get the distanceToNode from the parent node to the current node
            gOfNParentDist = distanceToNode(qstart, nodeIters[i]);
            //Put the index of each into the camefrom Vec
            cameFrom.push_back(tuple<int,int>(0,i));
            //Put that distanceToNode into the gOfN vector
            gOfNVector.push_back(gOfNParentDist);
            // calculate the total length
            temp = distanceToNode(nBest, nodeIters[i]) + distanceToNode(nodeIters[i], qgoal);
            openList.push_back(pair<int,double>(i,temp));
        }
    }
    // Sort the distances based on the value
    sort(openList.begin(),openList.end(),compareFunc);
    // Begin while loop
    while (!openList.empty()){
        // Pick nbest from the open list based on heuristic
        indexOfInterest = openList[openList.size()-1].first;
//        gOfNParentDist = gOfNVector[openList.size()-1];
        nBest = nodeIters[indexOfInterest];
        queueNodeIndex.push_back(indexOfInterest);
        closedList.push_back(nBest);
        openList.pop_back();

        rowOfInterest = nodeMap.row(indexOfInterest);
        //Iterate through the row of interest to get all of the connected nodes indices and weights;
        for(int i = 0; i!=rowOfInterest.size();i++) {
            // Determine if the node is in the closed list
            itClosed = find(closedList.begin(),closedList.end(),nodeIters[i]);
            // If the node is connected to the parent and not in the closed list
            if(rowOfInterest[i] != 0  && itClosed==closedList.end()){
                gN = gOfNParentDist + distanceToNode(nBest, nodeIters[i]);
                temp = distanceToNode(nBest, nodeIters[i]) + distanceToNode(nodeIters[i], qgoal);
                tempPair = make_pair(i,temp);
                // Determine if the node is in the open list
                itOpen = find(openList.begin(),openList.end(),tempPair);
                if(itOpen==openList.end()){
                    cameFrom.push_back(tuple<int,int>(indexOfInterest,i));
                    // calculate the total length
                    openList.push_back(tempPair);
                }// TODO implement the backpointer component
            }
        }

        if(distanceToNode(nBest,qgoal) <= epsilon){
            break;
        }

        sort(openList.begin(),openList.end(),compareFunc);

        iter++;
        if(iter == maxIters){
            flag2 = 0;
        }
    }
}

void GoalRRT::makePlan() {
    RRT();
    makeMap();
    findPathToGoal();
}

double GoalRRT::numConnections() {
    double conCount = 0;
    for(int i = 0; i!=ofInterest.size(); i++){
        if(ofInterest[i] == 1){
            conCount++;
        }
    }
    return conCount;
}

bool GoalRRT::compareFunc(pair<int,double> &a, pair<int,double> &b) {
    return a.second > b.second;
}

void GoalRRT::RRT() {
    //Need to generate a random sample between 0 and 1
    //If this sample lies within the goal bias range then set the same to qgoal
    //If it doesn't than sample Qfree
    tuple<double,double> qNear;
    tuple<double,double> qNew;
    tuple<double,double> qRand;
    tuple<double,double,double,double> lineSeg;
    tuple<double,double> v;
    tuple<double,double> u;
    tuple<double,double> pathInc;
    vector<tuple<double,double>> checkCol;
    double biasCheck;
    double distPrev;
    double distCurr;
    nodeIters.push_back(qstart);
    qNear = qstart;
    double stepSize = 0.5;
    static random_device rd;
    bool flag1 = true;
    int iters = 0;
    static mt19937 eng(rd());
    uniform_real_distribution<> xdist(get<0>(xBounds), get<1>(xBounds));
    uniform_real_distribution<> ydist(get<0>(yBounds), get<1>(yBounds));
    uniform_real_distribution<> sampOne(0, 1);
    while(true){
        iters++;
        //Get a random sample between 0 and 1;
        biasCheck = sampOne(eng);
        if(biasCheck <= pGoal) {
            qRand = qgoal;
        }else{
            while(flag1){
                qRand = tuple<double,double>(xdist(eng),ydist(eng));
                if(!checkCollision(qRand)){
                    flag1 = false;
                }
            }
        }
        flag1 = true;
        // Next we need to find the closest node to this sampled node
        distPrev = distanceToNode(qRand,qstart);
        for(int i = 0; i!=nodeIters.size(); i++){
            distCurr = distanceToNode(nodeIters[i],qRand);
            if(distCurr<distPrev){
                qNear = nodeIters[i];
                distPrev = distCurr;
            }
//            if(distCurr <= r){
//                qNear = nodeIters[i];
//                break;
//            }
        }
        // Now we need to take a step in that direction and if that segment is collision free we can add it to the list
        // node iters
        v = vectorDiff(qNear,qRand);
        u = tuple<double,double>(stepSize * get<0>(v)/vectorMag(v),stepSize * get<1>(v)/vectorMag(v) );
        pathInc = vectorAdd(qNear,u);
        checkCol.push_back(qNear);
        checkCol.push_back(pathInc);
//        checkCol.push_back(qNear);
//        checkCol.push_back(qRand);
        if(!checkCollisionPath(checkCol) && !checkCollision(pathInc)) {
            qNew = pathInc;
            nodeIters.push_back(qNew);
            lineSeg = make_tuple(get<0>(qNear),get<1>(qNear),get<0>(pathInc),get<1>(pathInc));
            nodeConns.push_back(lineSeg);
            if (distanceToNode(qNew, qgoal) <= epsilon) {
                break;
            }
        }
        if(iters == maxIters){
            flag2 = 0;
            break;
        }
        checkCol.clear();
    }
}

void GoalRRT::makeMap(){
    nodeMap = ArrayXXd::Zero(nodeIters.size(),nodeIters.size());
    tuple<double,double,double,double> tempInc;
    for(int i = 0; i!= nodeIters.size(); i++){
        for(int j = 0; j!= nodeIters.size(); j++){
            tempInc = make_tuple(get<0>(nodeIters[i]),get<1>(nodeIters[i]),get<0>(nodeIters[j]),get<1>(nodeIters[j]));
            for(int k = 0; k!=nodeConns.size(); k++){
                if( tempInc == nodeConns[k]){
                    nodeMap(i,j) = 1;
                }
            }
        }
    }
}

tuple<double, double> GoalRRT::vectorAdd(tuple<double, double> tuple2, tuple<double, double> tuple1) {
    double diffX = get<0>(tuple1) + get<0>(tuple2);
    double diffY = get<1>(tuple1) + get<1>(tuple2);
    tuple<double, double> vecAdd(diffX, diffY);
    return vecAdd;
}

#endif //INC_5519_HW4_GOALRRT_H

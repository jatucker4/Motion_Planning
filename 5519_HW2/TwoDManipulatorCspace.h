//
// Created by John on 10/3/2020.
//

#ifndef INC_5519_HW2_TWODMANIPULATORCSPACE_H
#define INC_5519_HW2_TWODMANIPULATORCSPACE_H
#include <utility>

#include "TwoLink.h"
// Create a class for the 2 link manipulator C space

using namespace std;
class TwoDManipulatorCSpace {
public:
    TwoDManipulatorCSpace(vector<vector<tuple<double,double>>> ObjVertices,TwoLink &myArm);
    void discretizeCSpace();
    void getCollisionBools();
    vector<tuple<double, double>> forwardKinematics(tuple<double,double> thetas);
    vector<tuple<double, double>> discretizeLinks(vector<tuple<double, double>> origins);
    bool checkCollision(vector<tuple<double,double>> coords);
    bool onSegment(tuple<double,double> p, tuple<double,double> q, tuple<double,double> r);
    double orientation(tuple<double,double> p, tuple<double,double> q, tuple<double,double> r);
    bool doIntersect(tuple<double,double> p1, tuple<double,double> q1, tuple<double,double> p2, tuple<double,double> q2);
    bool getPrimitiveIntersection(vector<tuple<double, double>> pointOne, vector<tuple<double, double>> pointTwo);
    vector<double> binaryList;
    vector<tuple<double,double>> discCoordList;
    double xLim;
    double yLim;
    double resolution;
    vector<vector<tuple<double,double>>> obstacles;
    TwoLink *manipulator;
    vector<double> collBools;

};

TwoDManipulatorCSpace::TwoDManipulatorCSpace(vector<vector<tuple<double,double>>>  ObjVertices, TwoLink &myArm) {
    // Constructor that assigns class variables
    obstacles = move(ObjVertices);
    manipulator =  &myArm;
    xLim = 2 * M_PI;
    yLim = 2 * M_PI;
    resolution = 500;
}

void TwoDManipulatorCSpace::discretizeCSpace() {
    // First create the discretized vector of x and y values
    vector<double> xVals;
    vector<double> yVals;
    xVals.push_back(0);
    double sum;

    double additive = xLim/resolution;
    sum = additive;
    // Loop through the resolution to get values from 0 to 2pi
    for(int i = 1; i<=resolution; i++){
        xVals.push_back(sum);
        sum += additive;
    }
    // Create the y vals
    yVals = xVals;
    // Create the grid
    for(auto iter : yVals){
        for(auto iter2 : xVals){
            discCoordList.emplace_back(iter2,iter);
        }
    }
}

void TwoDManipulatorCSpace::getCollisionBools() {
    vector<tuple<double,double>> effectorPos;
    vector<tuple<double,double>> linkCoords;
    for(auto tup : discCoordList){

        effectorPos = forwardKinematics(tup);
        if(checkCollision(effectorPos)){
            collBools.push_back(1);
        }else{
            collBools.push_back(0);
        }
    }
}

vector<tuple<double, double>> TwoDManipulatorCSpace::forwardKinematics(tuple<double,double> thetas) {
    double thetaOne = get<0>(thetas);
    double thetaTwo = get<1>(thetas);
    int count = 0;
    if (thetaOne >= M_PI/4 && thetaOne <= 3*M_PI/4){
        count+=1;
    }
    tuple<double,double> endEffector;
    vector<tuple<double,double>> coordList;
    double dist = manipulator->distBetwJoint;
    // First I need to create the transformation matrix object for each link
    Matrix<double, 3, 3> T1;
    Matrix<double, 3, 3> T2;
    Matrix<double, 3, 3> T3;
    // Now create and initialize the vector to determine the xy of the origin of each links local frame
    Vector3d L1;
    L1 << 0,0,1;
    // Now initialize each matrix
    T1 << cos(thetaOne), -sin(thetaOne), 0,
            sin(thetaOne), cos(thetaOne), 0,
            0, 0, 1;
    T2 << cos(thetaTwo), -sin(thetaTwo), dist,
            sin(thetaTwo), cos(thetaTwo), 0,
            0, 0, 1;
    T3 << 1, 0, dist,
            0, 1, 0,
            0, 0, 1;
    // Now we can determine the origin of each links local frame by multiplying the T matrices by the L1 vector
    Matrix<double, 3,1> L1Global = T1 * L1;
    Matrix<double, 3,1> L2Global = (T1 * T2) * L1;
    Matrix<double, 3,1> EEFGlobal = (T1 * T2 * T3) * L1;
    // Create the tuples for each link origin
    double EFX = EEFGlobal[0];
    double EFY = EEFGlobal[1];
    endEffector = make_tuple(EFX, EFY);
    tuple<double, double> L1Origin (L1Global[0],L1Global[1]);
    tuple<double, double> L2Origin (L2Global[0],L2Global[1]);
    coordList.push_back(L1Origin);
    coordList.push_back(L2Origin);
    coordList.push_back(endEffector);
    return coordList;
}

vector<tuple<double, double>> TwoDManipulatorCSpace::discretizeLinks(vector<tuple<double, double>> origins) {
    vector<tuple<double, double>> coordinates;
    double X;
    double Y;
    int temp = 500;
    for(int i = 1;i<=temp; i++){
        X = (get<0>(origins[0])) + ((get<0>(origins[1])) - (get<0>(origins[0]))) * i / temp;
        Y = (get<1>(origins[0])) + ((get<1>(origins[1])) - (get<1>(origins[0]))) * i / temp;
        coordinates.emplace_back(X,Y);
    }
    for(int i = 1;i<=temp; i++){
        X = (get<0>(origins[1])) + ((get<0>(origins[2])) - (get<0>(origins[1]))) * i / resolution;
        Y = (get<1>(origins[1])) + ((get<1>(origins[2])) - (get<1>(origins[1]))) * i / resolution;
        coordinates.emplace_back(X,Y);
    }
    return coordinates;
}

bool TwoDManipulatorCSpace::getPrimitiveIntersection(vector<tuple<double, double>> pointOne, vector<tuple<double, double>> pointTwo) {
    double A1 = (get<1>(pointOne[0]) - get<1>(pointOne[1])) / (get<0>(pointOne[0]) - get<0>(pointOne[1]));
    double A2 = (get<1>(pointTwo[0]) - get<1>(pointTwo[1])) / (get<0>(pointTwo[0]) - get<0>(pointTwo[1]));
    double b1 = (get<1>(pointOne[0])) - (A1 * get<0>(pointOne[0]));
    double b2 = (get<1>(pointTwo[0])) - (A2 * get<0>(pointTwo[0]));

    if( max(get<0>(pointOne[0]) , get<0>(pointOne[1])) < min(get<0>(pointTwo[0]) , get<0>(pointTwo[1]))){
        return false;
    }
    if(A1 == A2){
        // Segments are parallel
        return false;
    }
    double Xa = (b2 - b1) / (A1 - A2);
    if( (Xa < max( min(get<0>(pointOne[0]),get<0>(pointOne[1])) , min(get<0>(pointTwo[0]),get<0>(pointTwo[1])))) ||
            (Xa > min( max(get<0>(pointOne[0]),get<0>(pointOne[1])) , max(get<0>(pointTwo[0]),get<0>(pointTwo[1]))))){
        return false;
    }else{
        return true;
    }
}

bool TwoDManipulatorCSpace::checkCollision(vector<tuple<double,double>> coords) {
    // the variable coords contains the origin locations for each link;
    tuple<double,double> link1Origin = coords[0];
    tuple<double,double> link2Origin = coords[1];
    tuple<double,double> EndEffOrigin = coords[2];
    for(auto obstacle : obstacles){
        // If one of the obstacles is a triangle the primitives need to be created in a special way.
        if(obstacle.size() == 3){
            tuple<double,double> obsVert1 = obstacle[0];
            tuple<double,double> obsVert2 = obstacle[1];
            tuple<double,double> obsVert3 = obstacle[2];

            bool prim1 = doIntersect(link1Origin,link2Origin, obsVert1,obsVert2);
            bool prim2 = doIntersect(link1Origin,link2Origin, obsVert2,obsVert3);
            bool prim3 = doIntersect(link1Origin,link2Origin, obsVert3,obsVert1);
            bool prim4 = doIntersect(link2Origin,EndEffOrigin, obsVert1,obsVert2);
            bool prim5 = doIntersect(link2Origin,EndEffOrigin, obsVert2,obsVert3);
            bool prim6 = doIntersect(link2Origin,EndEffOrigin, obsVert3,obsVert1);

            if(prim1 || prim2 || prim3 || prim4 || prim5 || prim6){
                return true;
            }

        }else{
            // Else it's a rectangle and we can do as before
            tuple<double,double> obsVert1 = obstacle[0];
            tuple<double,double> obsVert2 = obstacle[1];
            tuple<double,double> obsVert3 = obstacle[2];
            tuple<double,double> obsVert4 = obstacle[3];

            bool prim1 = doIntersect(link1Origin,link2Origin, obsVert1, obsVert2);
            bool prim2 = doIntersect(link1Origin,link2Origin, obsVert2,obsVert3);
            bool prim3 = doIntersect(link1Origin,link2Origin, obsVert3,obsVert4);
            bool prim4 = doIntersect(link1Origin,link2Origin, obsVert4,obsVert1);
            bool prim5 = doIntersect(link2Origin,EndEffOrigin, obsVert1, obsVert2);
            bool prim6 = doIntersect(link2Origin,EndEffOrigin, obsVert2,obsVert3);
            bool prim7 = doIntersect(link2Origin,EndEffOrigin, obsVert3,obsVert4);
            bool prim8 = doIntersect(link2Origin,EndEffOrigin, obsVert4,obsVert1);

            if(prim1 || prim2 || prim3 || prim4 || prim5 || prim6 || prim7 || prim8){
                return true;
            }
        }
    }
    return false;
}

bool TwoDManipulatorCSpace::onSegment(tuple<double,double> p, tuple<double,double> q, tuple<double,double> r)
{

    if (get<0>(q) <= max(get<0>(p), get<0>(r)) && get<0>(q) >= min(get<0>(p), get<0>(r)) &&
            get<1>(q) <= max(get<1>(p), get<1>(r)) && get<1>(q) >= min(get<1>(p), get<1>(r))){
        return true;
    }
    return false;
}

double TwoDManipulatorCSpace::orientation(tuple<double,double> p, tuple<double,double> q, tuple<double,double> r)
{
    double val = (get<1>(q) - get<1>(p)) * (get<0>(r) - get<0>(q)) -
            (get<0>(q) - get<0>(p)) * (get<1>(r) - get<1>(q));

    if (val == 0) return 0;  // colinear

    return (val > 0)? 1: 2; // clock or counterclock wise
}

bool TwoDManipulatorCSpace::doIntersect(tuple<double,double> p1, tuple<double,double> q1, tuple<double,double> p2, tuple<double,double> q2)
{
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
#endif //INC_5519_HW2_TWODMANIPULATORCSPACE_H

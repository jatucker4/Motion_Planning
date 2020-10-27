//
// Created by John on 10/3/2020.
//

#ifndef INC_5519_HW2_TWODMANIPULATORCSPACE_H
#define INC_5519_HW2_TWODMANIPULATORCSPACE_H
#include <utility>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "TwoLink.h"
// Create a class for the 2 link manipulator C space

using namespace std;
class TwoDManipulatorCSpace {
public:
    // Class methods
    TwoDManipulatorCSpace(vector<vector<tuple<double,double>>> ObjVertices,TwoLink &myArm);
    void createBrushGrid();
    vector<tuple<double, double>> forwardKinematics(tuple<double,double> thetas);
    vector<tuple<double, double>> discretizeLinks(vector<tuple<double, double>> origins);
    bool checkCollision(vector<tuple<double,double>> coords);
    bool onSegment(tuple<double,double> p, tuple<double,double> q, tuple<double,double> r);
    double orientation(tuple<double,double> p, tuple<double,double> q, tuple<double,double> r);
    bool doIntersect(tuple<double,double> p1, tuple<double,double> q1, tuple<double,double> p2, tuple<double,double> q2);
    // Class variables
    vector<double> binaryList;
    vector<tuple<double,double>> discCoordList;
    double xLim;
    double yLim;
    double resolution;
    vector<vector<tuple<double,double>>> obstacles;
    TwoLink *manipulator;
    vector<double> collBools;
    vector<tuple<double,double>> wavePath;
    vector<vector<tuple<double,double>>> workPath;
    ArrayXXd brushDistList;
};

TwoDManipulatorCSpace::TwoDManipulatorCSpace(vector<vector<tuple<double,double>>>  ObjVertices, TwoLink &myArm) {
    // Constructor that assigns class variables
    obstacles = move(ObjVertices);
    manipulator =  &myArm;
    xLim = 2 * M_PI;
    yLim = 2 * M_PI;
    resolution = 150;
    ArrayXXd m = ArrayXXd::Zero(resolution,resolution);
    brushDistList = m;
}
vector<tuple<double, double>> TwoDManipulatorCSpace::forwardKinematics(tuple<double,double> thetas) {
    /*
     * Function for inverse kinematics
     * Inputs: tuple of thetas for each link
     * Outputs: vector of tuples that represent the origin for each link and the end effector
     */
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
    /*
     * Function that discretizes the links to check collisions more accurately
     * Inputs: vector of tuples that represent the link/end effector origins
     */
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

bool TwoDManipulatorCSpace::checkCollision(vector<tuple<double,double>> coords) {
    /*
     * Method for checking collisions with obstacles
     * Inputs: Vector of tuples that represent all of the xy coords to be checked
     * Outputs: Bool where true indicates collision false indicates no collision
     */
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
            // Use primitives to check if the links intersect
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
void TwoDManipulatorCSpace::createBrushGrid(){
    /*
     * This method discretizes the wavefront grid and calculates a plan for the given twolink manipulator
     * Inputs: N/A
     * Outputs: N/A
     */
    vector<tuple<double,double>> effectorPos;
    vector<tuple<double,double>> linkCoords;
    tuple<double,double> tup;
    VectorXd xCoords;
    xCoords.setLinSpaced(resolution,0,yLim );
    VectorXd yCoords;
    yCoords.setLinSpaced(resolution,0,yLim);
    int xIndex = 0;
    int yIndex = 0;
    // Create the grid
    ArrayXXd waveGridList;
    // First discretize the grid and create the foundation for the wavefront grid by marking where obstacles are
    for(int i = 0; i <= resolution-1; i ++){
        for(int j = 0; j <= resolution-1; j++){
            discCoordList.emplace_back(xCoords[j],yCoords[i]);
            tup = tuple<double,double>(xCoords[j],yCoords[i]);
            effectorPos = forwardKinematics(tup);
            if(checkCollision(effectorPos)){
                brushDistList(j,i) = 1;
            }else{
                brushDistList(j,i) = 0;
            }
            if(abs(xCoords[j] - 3.141)<0.021  && yCoords[i] == 0 && xCoords[j] > 0){
                brushDistList(j,i) = 2;
            }
            if(abs(xCoords[j] - 0) == 0 && abs(yCoords[i]-0)==0){
                xIndex = j;
                yIndex = i;
            }
        }
    }
    // Next create the rest of the wavefront grid
    int index = 2;
    double xres = resolution;
    double yres = resolution;
    while((brushDistList == 0).any()){
        // First loop through every grid space and check if there is a one
        for(int i = 0; i <= yres-1; i ++){
            for(int j = 0; j <= xres-1; j++){
                // if there is a 1 check it's neighbors
                if(brushDistList(j,i) == index){
                    // Conditionals for the border cases
                    // If we're at the bottom border
                    if(i == 0 && j == 0){
                        if( brushDistList(j+1, i) == 0){
                            brushDistList(j+1, i) = index + 1;
                        }
                        if(brushDistList(j, i+1) == 0){
                            brushDistList(j, i+1)= index + 1;
                        }
                    }else if (i == 0 && j == xres-1){
                        if( brushDistList(j-1, i) == 0){
                            brushDistList(j-1, i) = index + 1;
                        }
                        if(brushDistList(j, i+1) == 0){
                            brushDistList(j, i+1)= index + 1;
                        }
                    }else if (i == yres-1 && j == 0){
                        if( brushDistList(j+1, i) == 0){
                            brushDistList(j+1, i) = index + 1;
                        }
                        if(brushDistList(j, i-1) == 0){
                            brushDistList(j, i-1)= index + 1;
                        }
                    }else if (i == yres-1 && j == xres-1){
                        if( brushDistList(j-1, i) == 0){
                            brushDistList(j-1, i) = index + 1;
                        }
                        if(brushDistList(j, i-1) == 0){
                            brushDistList(j, i-1)= index + 1;
                        }
                    }else if (i == 0){
                        if(brushDistList(j-1, i) == 0){
                            brushDistList(j-1, i) = (index + 1);
                        }
                        if(brushDistList(j+1, i) == 0){
                            brushDistList(j+1, i) = (index + 1);
                        }
                        if(brushDistList(j, i+1) == 0){
                            brushDistList(j, i+1) = (index + 1);
                        }
                    }else if (j == 0){
                        if(brushDistList(j, i-1) == 0){
                            brushDistList(j, i-1) = (index + 1);
                        }
                        if(brushDistList(j+1, i) == 0){
                            brushDistList(j+1, i) = (index + 1);
                        }
                        if(brushDistList(j, i+1) == 0){
                            brushDistList(j, i+1) = (index + 1);
                        }
                    }else if (i == yres-1){
                        if(brushDistList(j-1, i) == 0){
                            brushDistList(j-1, i) = (index + 1);
                        }
                        if(brushDistList(j+1, i) == 0){
                            brushDistList(j+1, i) = (index + 1);
                        }
                        if(brushDistList(j, i-1) == 0){
                            brushDistList(j, i-1) = (index + 1);
                        }
                    }else if (j == xres-1){
                        if(brushDistList(j-1, i) == 0){
                            brushDistList(j-1, i) = (index + 1);
                        }
                        if(brushDistList(j, i-1) == 0){
                            brushDistList(j, i-1) = (index + 1);
                        }
                        if(brushDistList(j, i+1) == 0){
                            brushDistList(j, i+1) = (index + 1);
                        }
                    }else{
                        if(brushDistList(j-1, i) == 0){
                            brushDistList(j-1, i) = (index + 1);
                        }
                        if(brushDistList(j+1, i) == 0){
                            brushDistList(j+1, i) = (index + 1);
                        }
                        if(brushDistList(j, i+1) == 0){
                            brushDistList(j, i+1) = (index + 1);
                        }
                        if(brushDistList(j, i-1) == 0){
                            brushDistList(j, i-1) = (index + 1);
                        }
                    }
                }
            }
        }
        // increment index by one so on the next pass it's checking if there is a 2 and index any zero neighbors to 3
        index++;
        // repeat
        if(index == 500){
            break;
        }
    }
    // Finally calculate a path to goal
    int upenergy;
    int downenergy;
    bool flag;
    int leftenergy;
    int rightenergy;
    int tileVal;
    // Find the index for start and get the value of that tile
    int energy = brushDistList(xIndex,yIndex);
    while (energy != 2){
        for(int i = 0; i <= yres-1; i ++){
            for(int j = 0; j <= xres-1; j++) {
                if(j == xIndex && i == yIndex) {
                    wavePath.emplace_back(xCoords[j],yCoords[i]);
                    cout<< "emplacing: " << xCoords[j] << " " << yCoords[i] << endl;

                    upenergy = brushDistList(j,i+1);
                    if(i == 0){
                        downenergy = 10000;
                    }else{
                        downenergy = brushDistList(j,i-1);
                    }
                    if(j==0){
                        leftenergy = 10000;
                    }else{
                        leftenergy = brushDistList(j-1,i);
                    }
                    if(j == xres-1){
                        flag = true;
                        break;
                    }else{
                        rightenergy = brushDistList(j+1,i);
                    }
                    if (rightenergy == 1){
                        rightenergy = 100000;
                    }
                    if (leftenergy == 1){
                        leftenergy = 100000;
                    }
                    if (upenergy == 1){
                        upenergy = 100000;
                    }
                    if (downenergy == 1){
                        downenergy = 100000;
                    }
                    if (upenergy<downenergy && upenergy<leftenergy && upenergy<rightenergy){
                        xIndex = j;
                        yIndex = i+1;
                        tileVal = brushDistList(xIndex,yIndex);
                    } else if (downenergy<upenergy && downenergy<leftenergy && downenergy<rightenergy) {
                        xIndex = j;
                        yIndex = i-1;
                        tileVal = brushDistList(xIndex,yIndex);
                    } else if (leftenergy<upenergy && leftenergy<downenergy && leftenergy<rightenergy) {
                        xIndex = j-1;
                        yIndex = i;
                        tileVal = brushDistList(xIndex,yIndex);
                    } else if (rightenergy<leftenergy && rightenergy<downenergy && rightenergy==upenergy){
                        xIndex = j+1;
                        yIndex = i;
                        tileVal = brushDistList(xIndex,yIndex);
                    } else if (rightenergy<leftenergy && rightenergy<upenergy && rightenergy==downenergy) {
                        xIndex = j;
                        yIndex = i-1;
                        tileVal = brushDistList(xIndex, yIndex);
                    } else if (rightenergy<leftenergy && rightenergy<downenergy && rightenergy<upenergy){
                        xIndex = j+1;
                        yIndex = i;
                        tileVal = brushDistList(xIndex,yIndex);
                    }
                }
            }
            if(flag){
                break;
            }
        }
        if(flag){
            break;
        }else{
            energy = tileVal;
        }
    }
    vector<tuple<double,double>> temp1;
    for(auto tup2 : wavePath){
        temp1 = forwardKinematics(tup2);
        workPath.push_back(temp1);
    }
}
#endif //INC_5519_HW2_TWODMANIPULATORCSPACE_H

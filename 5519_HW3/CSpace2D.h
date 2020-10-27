//
// Created by John on 10/18/2020.
//

#ifndef INC_5519_HW3_CSPACE2D_H
#define INC_5519_HW3_CSPACE2D_H
#include <string>
#include <cmath>
#include <tuple>
#include <vector>
#include <iostream>
#include "Eigen/Core"
#include "Eigen/Geometry"

using namespace std;
using namespace Eigen;

class CSpace2D {
public:
    CSpace2D(vector<vector<tuple<double,double>>> ObjVertices, tuple<int,int> xBounds, tuple<int,int> yBounds, int res);
    CSpace2D(double delta, vector<vector<tuple<double,double>>> ObjVertices, tuple<int,int> xBounds,
             tuple<int,int> yBounds, tuple<double,double> qgoal,tuple<double,double> qstart);
    vector<double> distToObst(tuple<double,double> currentLoc);
    void discretizeCSpace(tuple<int,int> xBounds, tuple<int,int> yBounds);
    void discretizeCSpace(tuple<int,int> xBounds, tuple<int,int> yBounds,
                          tuple<double,double> qgoal,tuple<double,double> qstart, int xres, int yres);
    void createBrushGrid();
    bool checkCollision(double xLoc, double yLoc);

    int resolution;
    vector<tuple<double,double>> discCoordList;
    ArrayXXd brushDistList;
    vector<vector<tuple<double,double>>> obstacles;
    tuple<int,int> xBound;
    tuple<int,int> yBound;
    int startXIndex;
    int startYIndex;
    int xres;
    int yres;
};

CSpace2D::CSpace2D(vector<vector<tuple<double,double>>> ObjVertices, tuple<int,int> xBounds,
                   tuple<int,int> yBounds, int res) {
    resolution = res;
    obstacles = ObjVertices;
    xBound = xBounds;
    yBound = yBounds;
    ArrayXXd m = ArrayXXd::Zero(res,res);
    brushDistList = m;
    startXIndex = 0;
    startYIndex = 0;
    xres = res;
    yres = res;
    discretizeCSpace(xBound,yBound);

    createBrushGrid();
}

CSpace2D::CSpace2D(double delta, vector<vector<tuple<double, double>>> ObjVertices, tuple<int, int> xBounds,
                   tuple<int, int> yBounds, tuple<double,double> qgoal,tuple<double,double> qstart) {
    xres = round((get<1>(xBounds) - get<0>(xBounds))/delta);
    yres = round((get<1>(yBounds) - get<0>(yBounds))/delta);

    obstacles = ObjVertices;
    xBound = xBounds;
    yBound = yBounds;
    ArrayXXd m = ArrayXXd::Zero(xres,yres);
    brushDistList = m;
    resolution = xres;
    discretizeCSpace(xBound,yBound, qgoal,qstart,xres,yres);

}
void CSpace2D::discretizeCSpace(tuple<int,int> xBounds, tuple<int,int> yBounds) {
    cout << "Discetrizing the CSpace" << endl;
    // First get the minX/maxX and minY/maxY variables
    int minX = get<0>(xBounds);
    int maxX = get<1>(xBounds);
    int minY = get<0>(yBounds);
    int maxY = get<1>(yBounds);

    // Next create linearly spaced vectors using the Eigen package
    VectorXd xCoords;
    xCoords.setLinSpaced(resolution,minX,maxX);
    VectorXd yCoords;
    yCoords.setLinSpaced(resolution,minY,maxY);

    // Iterate through a dynamic array to create the discretized grid
    for(int i = 0; i <= resolution-1; i ++){
        for(int j = 0; j <= resolution-1; j++){
            discCoordList.emplace_back(xCoords[j],yCoords[i]);

            // While we're here we should check obstacle collisions and create the initial grid for brushfire
            if (checkCollision(xCoords[j],yCoords[i])){
                brushDistList(j,i) = 1;
            }else{
                brushDistList(j,i) = 0;
            }
        }
    }
}

vector<double> CSpace2D::distToObst(tuple<double,double> currentLoc) {
    // First get the minX/maxX and minY/maxY variables
    int minX = get<0>(xBound);
    int maxX = get<1>(xBound);
    int minY = get<0>(yBound);
    int maxY = get<1>(yBound);

    vector<double> distList;
    // First get the index of the grid vector that gets the currentLoc
    int xIndex;
    int yIndex;
    // Next create linearly spaced vectors using the Eigen package
    VectorXd xCoords;
    xCoords.setLinSpaced(resolution,minX,maxX);
    VectorXd yCoords;
    yCoords.setLinSpaced(resolution,minY,maxY);

    // Iterate through a dynamic array to create the discretized grid
    for(int i = 0; i <= resolution-1; i ++) {
        for (int j = 0; j <= resolution - 1; j++) {
            if(xCoords[j] == get<0>(currentLoc) && yCoords[i] == get<1>(currentLoc)){
                xIndex = j;
                yIndex = i;
                break;
            }
        }
    }
    double dist = brushDistList(xIndex,yIndex);
    for(int i = 0; i <= obstacles.size(); i++){
        distList.push_back(dist);
    }
    return distList;
}

void CSpace2D::createBrushGrid(){
    cout << "Creating the Brush Grid" << endl;
    // index to check variable
    int index = 1;
    // Complete the brush fire grid by incrementing the distance for neighboring cells
    // While any of the cells are still zero
    while((brushDistList == 0).any()){
        // First loop through every grid space and check if there is a one
        for(int i = 0; i <= resolution-1; i ++){
            for(int j = 0; j <= resolution-1; j++){
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
                    }else if (i == 0 && j == resolution-1){
                        if( brushDistList(j-1, i) == 0){
                            brushDistList(j-1, i) = index + 1;
                        }
                        if(brushDistList(j, i+1) == 0){
                            brushDistList(j, i+1)= index + 1;
                        }
                    }else if (i == resolution-1 && j == 0){
                        if( brushDistList(j+1, i) == 0){
                            brushDistList(j+1, i) = index + 1;
                        }
                        if(brushDistList(j, i-1) == 0){
                            brushDistList(j, i-1)= index + 1;
                        }
                    }else if (i == resolution-1 && j == resolution-1){
                        if( brushDistList(j-1, i) == 0){
                            brushDistList(j+1, i) = index + 1;
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
                    }else if (i == resolution-1){
                        if(brushDistList(j-1, i) == 0){
                            brushDistList(j-1, i) = (index + 1);
                        }
                        if(brushDistList(j+1, i) == 0){
                            brushDistList(j+1, i) = (index + 1);
                        }
                        if(brushDistList(j, i-1) == 0){
                            brushDistList(j, i-1) = (index + 1);
                        }
                    }else if (j == resolution-1){
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
    }
}

bool CSpace2D::checkCollision(double xLoc, double yLoc) {
    //Iterate through each obstacle, construct primitaves and check if there was a collision
    for(auto obstacle : obstacles){
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

void CSpace2D::discretizeCSpace(tuple<int, int> xBounds, tuple<int, int> yBounds,
                                tuple<double, double> qgoal,tuple<double,double> qstart, int xres, int yres) {
    // First get the minX/maxX and minY/maxY variables
    int minX = get<0>(xBounds);
    int maxX = get<1>(xBounds);
    int minY = get<0>(yBounds);
    int maxY = get<1>(yBounds);
    double check1;
    double check11;
    double check2;
    double check22;
    // Next create linearly spaced vectors using the Eigen package
    VectorXd xCoords;
    xCoords.setLinSpaced(xres,minX,maxX);
    VectorXd yCoords;
    yCoords.setLinSpaced(yres,minY,maxY);
    if (obstacles.size()==5){
        check1 = 0.19;
        check11 = 0.19;
        check2 = 0.043;
        check22 = 0.043;
    }else if(obstacles.size()==9){
        check1 = 0.1273;
        check11 = 0.0001;
        check2 = 0.1273;
        check22 = 0.06;
    }
    // Iterate through a dynamic array to create the discretized grid
    for(int i = 0; i <= yres-1; i ++){
        for(int j = 0; j <= xres-1; j++){
            discCoordList.emplace_back(xCoords[j],yCoords[i]);
            // While we're here we should check obstacle collisions and create the initial grid for brushfire
            if (checkCollision(xCoords[j],yCoords[i])){
                brushDistList(j,i) = 1;
            }else{
                brushDistList(j,i) = 0;
            }
            if( abs(xCoords[j] - get<0>(qgoal))<=check11 &&    abs(yCoords[i] - get<1>(qgoal))<=check1 && xCoords[j] > 0 && yCoords[i]>0){
                brushDistList(j,i) = 2;
            }
            if(abs(xCoords[j] - get<0>(qstart))<=check22 &&   abs(yCoords[i] - get<1>(qstart))<=check2 && xCoords[j]>0 && yCoords[i]>0){
                startXIndex = j;
                startYIndex = i;
                cout << "START FOR WAVEPLANNER: " << xCoords[j] << ", " << yCoords[i] << endl;
            }

        }
    }
}


#endif //INC_5519_HW3_CSPACE2D_H

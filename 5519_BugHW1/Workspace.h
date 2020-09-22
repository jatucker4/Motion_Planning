//
// Created by John on 9/16/2020.
//

#ifndef INC_5519_BUGHW2_WORKSPACE_H
#define INC_5519_BUGHW2_WORKSPACE_H

#include <tuple>
#include <utility>
#include <vector>

using  namespace std;
// Create workspace class
class Workspace {
public:
    Workspace(vector<vector<tuple<double,double>>> obsList);
    vector<vector<tuple<double,double>>> obstacles;
    bool checkCollision(tuple<double,double> currentLocation);
};
// Construct the class with a vector of obstacles
Workspace::Workspace(vector<vector<tuple<double, double>>> obsList) {
    obstacles = obsList;
}
// Create checkCollision method
bool Workspace::checkCollision(tuple<double, double> currentLocation) {
    double xLoc = get<0>(currentLocation);
    double yLoc = get<1>(currentLocation);

    //Iterate through each obstacle, construct primitaves and check if there was a collision
    for(auto obstacle : obstacles){
        double bottom = get<1>(obstacle[0]);
        double right = get<0>(obstacle[1]);
        double top = get<1>(obstacle[2]);
        double left = get<0>(obstacle[3]);

        if(((yLoc >= bottom) && (xLoc <= right) && (yLoc <= top) && (xLoc >= left))){
            return true;
        }
    }
    return false;
}
#endif //INC_5519_BUGHW2_WORKSPACE_H

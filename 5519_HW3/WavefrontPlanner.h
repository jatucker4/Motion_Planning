//
// Created by John on 10/16/2020.
//

#ifndef INC_5519_HW3_WAVEFRONTPLANNER_H
#define INC_5519_HW3_WAVEFRONTPLANNER_H

#include "GradientPlanner.h"
class WavefrontPlanner {
public:
    WavefrontPlanner(CSpace2D &myspace,tuple<double,double> qStart, tuple<double,double> qGoal);
    void createWaveGrid();
    void getWavePath();
    vector<tuple<double,double>> wavePath;
    tuple<double,double> Qstart;
    tuple<double,double> Qgoal;
    vector<tuple<double,double>> discList;
    ArrayXXd waveGridList;
    int res;
    int startXIndex;
    int startYIndex;
    int minY;
    int minX;
    int maxY;
    int maxX;
    int xres;
    int yres;
};

WavefrontPlanner::WavefrontPlanner(CSpace2D &myspace, tuple<double, double> qStart, tuple<double, double> qGoal) {
    Qstart = qStart;
    Qgoal = qGoal;
    waveGridList = myspace.brushDistList;
    res = myspace.resolution;
    xres = myspace.xres;
    yres = myspace.yres;
    discList = myspace.discCoordList;
    startXIndex = myspace.startXIndex;
    startYIndex = myspace.startYIndex;
    minY = get<0>(myspace.yBound);
    maxY = get<1>(myspace.yBound);
    minX = get<0>(myspace.xBound);
    maxX = get<1>(myspace.xBound);
    createWaveGrid();
}

void WavefrontPlanner::createWaveGrid() {
    // Now we need to iterate to label all of the grids near qgoal
    // index to check variable
    int index = 2;
    // Complete the brush fire grid by incrementing the distance for neighboring cells
    // While any of the cells are still zero
    while((waveGridList == 0).any()){
        // First loop through every grid space and check if there is a one
        for(int i = 0; i <= yres-1; i ++){
            for(int j = 0; j <= xres-1; j++){
                // if there is a 1 check it's neighbors
                if(waveGridList(j,i) == index){
                    // Conditionals for the border cases
                    // If we're at the bottom border
                    if(i == 0 && j == 0){
                        if( waveGridList(j+1, i) == 0){
                            waveGridList(j+1, i) = index + 1;
                        }
                        if(waveGridList(j, i+1) == 0){
                            waveGridList(j, i+1)= index + 1;
                        }
                    }else if (i == 0 && j == xres-1){
                        if( waveGridList(j-1, i) == 0){
                            waveGridList(j-1, i) = index + 1;
                        }
                        if(waveGridList(j, i+1) == 0){
                            waveGridList(j, i+1)= index + 1;
                        }
                    }else if (i == yres-1 && j == 0){
                        if( waveGridList(j+1, i) == 0){
                            waveGridList(j+1, i) = index + 1;
                        }
                        if(waveGridList(j, i-1) == 0){
                            waveGridList(j, i-1)= index + 1;
                        }
                    }else if (i == yres-1 && j == xres-1){
                        if( waveGridList(j-1, i) == 0){
                            waveGridList(j-1, i) = index + 1;
                        }
                        if(waveGridList(j, i-1) == 0){
                            waveGridList(j, i-1)= index + 1;
                        }
                    }else if (i == 0){
                        if(waveGridList(j-1, i) == 0){
                            waveGridList(j-1, i) = (index + 1);
                        }
                        if(waveGridList(j+1, i) == 0){
                            waveGridList(j+1, i) = (index + 1);
                        }
                        if(waveGridList(j, i+1) == 0){
                            waveGridList(j, i+1) = (index + 1);
                        }
                    }else if (j == 0){
                        if(waveGridList(j, i-1) == 0){
                            waveGridList(j, i-1) = (index + 1);
                        }
                        if(waveGridList(j+1, i) == 0){
                            waveGridList(j+1, i) = (index + 1);
                        }
                        if(waveGridList(j, i+1) == 0){
                            waveGridList(j, i+1) = (index + 1);
                        }
                    }else if (i == yres-1){
                        if(waveGridList(j-1, i) == 0){
                            waveGridList(j-1, i) = (index + 1);
                        }
                        if(waveGridList(j+1, i) == 0){
                            waveGridList(j+1, i) = (index + 1);
                        }
                        if(waveGridList(j, i-1) == 0){
                            waveGridList(j, i-1) = (index + 1);
                        }
                    }else if (j == xres-1){
                        if(waveGridList(j-1, i) == 0){
                            waveGridList(j-1, i) = (index + 1);
                        }
                        if(waveGridList(j, i-1) == 0){
                            waveGridList(j, i-1) = (index + 1);
                        }
                        if(waveGridList(j, i+1) == 0){
                            waveGridList(j, i+1) = (index + 1);
                        }
                    }else{
                        if(waveGridList(j-1, i) == 0){
                            waveGridList(j-1, i) = (index + 1);
                        }
                        if(waveGridList(j+1, i) == 0){
                            waveGridList(j+1, i) = (index + 1);
                        }
                        if(waveGridList(j, i+1) == 0){
                            waveGridList(j, i+1) = (index + 1);
                        }
                        if(waveGridList(j, i-1) == 0){
                            waveGridList(j, i-1) = (index + 1);
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

void WavefrontPlanner::getWavePath() {
    int tileVal;
    // Find the index for start and get the value of that tile
    int energy = waveGridList(startXIndex,startYIndex);
    // Loop through the neighbors of the "current location" grid adding it's location to the discrete path
    // Current indices are j = xstart, i = ystart
    // Loop through the discGrid until you get to that index and add it to the path list
    // Check the grids around in disc list
    int xIndex = startXIndex;
    int yIndex = startYIndex;
    int upenergy;
    int downenergy;
    bool flag;
    int leftenergy;
    int rightenergy;
    VectorXd xCoords;
    xCoords.setLinSpaced(xres,minX,maxX);
    VectorXd yCoords;
    yCoords.setLinSpaced(yres,minY,maxY);
    cout << "STARTING AT: " << xCoords[startXIndex] << " " << yCoords[startYIndex] << endl;
    while (energy != 2){
        for(int i = 0; i <= yres-1; i ++){
            for(int j = 0; j <= xres-1; j++) {
                if(j == xIndex && i == yIndex) {
                    wavePath.emplace_back(xCoords[j],yCoords[i]);
                    cout<< "emplacing: " << xCoords[j] << " " << yCoords[i] << endl;
                    upenergy = waveGridList(j,i+1);
                    downenergy = waveGridList(j,i-1);
                    leftenergy = waveGridList(j-1,i);
                    if(j == xres-1){
                        flag = true;
                        break;
                    }else{
                        rightenergy = waveGridList(j+1,i);
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
                        tileVal = waveGridList(xIndex,yIndex);
                    } else if (downenergy<upenergy && downenergy<leftenergy && downenergy<rightenergy) {
                        xIndex = j;
                        yIndex = i-1;
                        tileVal = waveGridList(xIndex,yIndex);
                    } else if (leftenergy<upenergy && leftenergy<downenergy && leftenergy<rightenergy) {
                        xIndex = j-1;
                        yIndex = i;
                        tileVal = waveGridList(xIndex,yIndex);
                    } else if (rightenergy<leftenergy && rightenergy<downenergy && rightenergy==upenergy){
                        xIndex = j+1;
                        yIndex = i;
                        tileVal = waveGridList(xIndex,yIndex);
                    } else if (rightenergy<leftenergy && rightenergy<upenergy && rightenergy==downenergy) {
                        xIndex = j + 1;
                        yIndex = i;
                        tileVal = waveGridList(xIndex, yIndex);
                    } else if (rightenergy<leftenergy && rightenergy<downenergy && rightenergy<upenergy){
                        xIndex = j+1;
                        yIndex = i;
                        tileVal = waveGridList(xIndex,yIndex);
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
}


#endif //INC_5519_HW3_WAVEFRONTPLANNER_H

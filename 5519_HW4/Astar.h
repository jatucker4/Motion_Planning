//
// Created by John on 11/6/2020.
//

#ifndef INC_5519_HW4_ASTAR_H
#define INC_5519_HW4_ASTAR_H
#include "GoalRRT.h"

class Astar {
public:
    Astar(Eigen::ArrayXXd& nodeMat, Eigen::ArrayXXd& edgeMat,vector<tuple<double,double>> nodes,vector<int> heur);
    ArrayXXd nodeMap;
    ArrayXXd edgeMap;
    vector<tuple<double,double>> nodeIter;
    void doAstar();
    vector<int> heuristic;
    vector<tuple<double,double>> cameFrom;
    int iterNum;
    tuple<double,double> nBest;
    static bool compareFunc(pair<int,double> &a, pair<int,double> &b);
};

Astar::Astar(ArrayXXd &nodeMat, ArrayXXd &edgeMat, vector<tuple<double, double>> nodes,vector<int> heur) {
    nodeMap = nodeMat;
    edgeMap = edgeMat;
    nodeIter = nodes;
    heuristic = heur;
    iterNum = 0;
}

void Astar::doAstar() {
    vector<tuple<double,double>> frontier;
    frontier.push_back(nodeIter[0]);
    cameFrom.emplace_back(nodeIter[0]);
    double costSoFar = 0;
    double newCost = 0;
    tuple<double,double> current;
    int indexOfInterest = 0;
    double priority;
    VectorXd rowOfInterest;
    vector<tuple<double,double>>::iterator itClosed;
    while(!frontier.empty()){
        iterNum++;
        current = frontier[indexOfInterest];
        if (current == nodeIter[13]){
            break;
        }
        rowOfInterest = nodeMap.row(indexOfInterest);
        for(int i = 0; i!=rowOfInterest.size();i++){
            if(rowOfInterest[i] != 0){
                newCost = edgeMap(indexOfInterest,i);
                itClosed = find(cameFrom.begin(),cameFrom.end(),nodeIter[i]);
                if(itClosed==cameFrom.end() || newCost < costSoFar){
                    costSoFar+=newCost;
                    priority = newCost + heuristic[i];
                    frontier.push_back(nodeIter[i]);
                    indexOfInterest = i;
                }
            }
        }

    }
}

bool Astar::compareFunc(pair<int,double> &a, pair<int,double> &b) {
    return a.second > b.second;
}


#endif //INC_5519_HW4_ASTAR_H

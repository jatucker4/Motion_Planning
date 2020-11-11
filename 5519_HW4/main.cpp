#include <iostream>
#include <fstream>
#include "Astar.h"
int main() {
    /////////////////////////////////////Exercise 2 Part A//////////////////////////////////////////////////////////////
    cout << "Beginning Problem 2a" << endl;
    vector<vector<tuple<double, double>>> obsList;
    // Create the vectors for each obstacle and fill them with the tuples of coordinates
    vector<tuple<double,double>> obsCoords1;
    obsCoords1.emplace_back(3.5,0.5);
    obsCoords1.emplace_back(3.5,1.5);
    obsCoords1.emplace_back(4.5,1.5);
    obsCoords1.emplace_back(4.5,0.5);


    vector<tuple<double,double>> obsCoords2;
    obsCoords2.emplace_back(6.5,-1.5);
    obsCoords2.emplace_back(6.5,-0.5);
    obsCoords2.emplace_back(7.5,-0.5);
    obsCoords2.emplace_back(7.5,-1.5);

    obsList.push_back(obsCoords1);
    obsList.push_back(obsCoords2);
//    // Create necessary constants
    int resolution = 50;
    tuple<int,int> xBounds(-1,11);
    tuple<int,int> yBounds(-3,3);
    CSpace2D myNewSpace(obsList,xBounds,yBounds,resolution);
    int n = 200;
    double r = 1;
    tuple<double,double> qstart(0,0);
    tuple<double,double> qgoal(10, 0);

    PRMPlanner myPRMPlan((CSpace2D &) myNewSpace,n,r,qstart,qgoal);
    ofstream myfile1;
    myfile1.open("nodeGridLocs1.txt");
    for(auto& tuple : myPRMPlan.nodeIters){
        myfile1 << get<0>(tuple) << " " << get<1>(tuple) << endl;
    }
    myfile1.close();

    ofstream myfile2;
    myfile2.open("connectionList1.txt");
    myfile2 << myPRMPlan.nodeMap;
    myfile2.close();

    ofstream myfile3;
    myfile3.open("nodePath.txt");
    for(auto& tuple : myPRMPlan.nodePath){
        myfile3 << get<0>(tuple) << " " << get<1>(tuple) << endl;
    }
    myfile3.close();

    ofstream myfile900;
    myfile900.open("camefrom.txt");
    for(auto& tuple : myPRMPlan.cameFrom){
        myfile900 << get<0>(tuple) << " " << get<1>(tuple) << endl;
    }
    myfile900.close();
    ////////////////////////////////////////BEGIN 2 A BENCHMARKING WITHOUT SMOOTHING///////////////////////////////////
    int benchNum = 100;
    n = 200;
    r = 0.5;
    vector<tuple<int,int>> pathA;
    vector<tuple<double,double>> nodeVecA;
    vector<double> timeVecA;
    vector<int> validFlagA;
    PRMPlanner** planArrayA1 = new PRMPlanner*[benchNum];
    for (int i = 0; i!= benchNum; i++){
        auto start = std::chrono::high_resolution_clock::now();
        planArrayA1[i] = new PRMPlanner((CSpace2D &) myNewSpace,n,r,qstart,qgoal);
        auto finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = finish - start;
        timeVecA.push_back(elapsed.count());
        validFlagA.push_back(planArrayA1[i]->flag2);
        for(auto& tuple : planArrayA1[i]->cameFrom){
            pathA.push_back(tuple);
        }
        for(auto& tup : planArrayA1[i]->nodeIters){
            nodeVecA.push_back(tup);
        }
    }
    delete[] planArrayA1;
    r = 1;
    vector<tuple<int,int>> pathA2;
    vector<tuple<double,double>> nodeVecA2;
    vector<double> timeVecA2;
    vector<int> validFlagA2;
    PRMPlanner** planArrayA2 = new PRMPlanner*[benchNum];
    for (int i = 0; i!= benchNum; i++){
        auto start = std::chrono::high_resolution_clock::now();
        planArrayA2[i] = new PRMPlanner((CSpace2D &) myNewSpace,n,r,qstart,qgoal);
        auto finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = finish - start;
        timeVecA2.push_back(elapsed.count());
        validFlagA2.push_back(planArrayA2[i]->flag2);
        for(auto& tuple : planArrayA2[i]->cameFrom){
            pathA2.push_back(tuple);
        }
        for(auto& tup : planArrayA2[i]->nodeIters){
            nodeVecA2.push_back(tup);
        }
    }
    delete[] planArrayA2;
    r = 1.5;
    vector<tuple<int,int>> pathA3;
    vector<tuple<double,double>> nodeVecA3;
    vector<double> timeVecA3;
    vector<int> validFlagA3;
    PRMPlanner** planArrayA3 = new PRMPlanner*[benchNum];
    for (int i = 0; i!= benchNum; i++){
        auto start = std::chrono::high_resolution_clock::now();
        planArrayA3[i] = new PRMPlanner((CSpace2D &) myNewSpace,n,r,qstart,qgoal);
        auto finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = finish - start;
        timeVecA3.push_back(elapsed.count());
        validFlagA3.push_back(planArrayA3[i]->flag2);
        for(auto& tuple : planArrayA3[i]->cameFrom){
            pathA3.push_back(tuple);
        }
        for(auto& tup : planArrayA3[i]->nodeIters){
            nodeVecA3.push_back(tup);
        }
    }
    delete[] planArrayA3;
    r = 2;
    vector<tuple<int,int>> pathA4;
    vector<tuple<double,double>> nodeVecA4;
    vector<double> timeVecA4;
    vector<int> validFlagA4;
    PRMPlanner** planArrayA4 = new PRMPlanner*[benchNum];
    for (int i = 0; i!= benchNum; i++){
        auto start = std::chrono::high_resolution_clock::now();
        planArrayA4[i] = new PRMPlanner((CSpace2D &) myNewSpace,n,r,qstart,qgoal);
        auto finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = finish - start;
        timeVecA4.push_back(elapsed.count());
        validFlagA4.push_back(planArrayA4[i]->flag2);
        for(auto& tuple : planArrayA4[i]->cameFrom){
            pathA4.push_back(tuple);
        }
        for(auto& tup : planArrayA4[i]->nodeIters){
            nodeVecA4.push_back(tup);
        }
    }
    delete[] planArrayA4;
    ofstream timeBenchFile2;
    timeBenchFile2.open("time_bench_n200.txt");
    for(int i = 0; i!=timeVecA.size(); i++){
        timeBenchFile2 << timeVecA[i] << endl;
    }
    for(int i = 0; i!=timeVecA2.size(); i++){
        timeBenchFile2 << timeVecA2[i] << endl;
    }
    for(int i = 0; i!=timeVecA3.size(); i++){
        timeBenchFile2 << timeVecA3[i] << endl;
    }
    for(int i = 0; i!=timeVecA4.size(); i++){
        timeBenchFile2 << timeVecA4[i] << endl;
    }
    timeBenchFile2.close();
    ofstream flagBenchFile2;
    flagBenchFile2.open("flag_bench_n200.txt");
    for(int i = 0; i!=validFlagA.size(); i++){
        flagBenchFile2 << validFlagA[i] << endl;
    }
    for(int i = 0; i!=validFlagA2.size(); i++){
        flagBenchFile2 << validFlagA2[i] << endl;
    }
    for(int i = 0; i!=validFlagA3.size(); i++){
        flagBenchFile2 << validFlagA3[i] << endl;
    }
    for(int i = 0; i!=validFlagA4.size(); i++){
        flagBenchFile2 << validFlagA4[i] << endl;
    }
    flagBenchFile2.close();

    ofstream pathBenchFile2;
    pathBenchFile2.open("path_bench_n200.txt");
    for(auto& tup : pathA){
        pathBenchFile2 << get<0>(tup) << " " << get<1>(tup) << endl;
    }
    for(auto& tup : pathA2){
        pathBenchFile2 << get<0>(tup) << " " << get<1>(tup) << endl;
    }
    for(auto& tup : pathA3){
        pathBenchFile2 << get<0>(tup) << " " << get<1>(tup) << endl;
    }
    for(auto& tup : pathA4){
        pathBenchFile2 << get<0>(tup) << " " << get<1>(tup) << endl;
    }
    pathBenchFile2.close();
    ofstream nodeBenchFile2;
    nodeBenchFile2.open("node_bench_n200.txt");
    for(auto& tup : nodeVecA){
        nodeBenchFile2 << get<0>(tup) << " " << get<1>(tup) << endl;
    }
    for(auto& tup : nodeVecA2){
        nodeBenchFile2 << get<0>(tup) << " " << get<1>(tup) << endl;
    }
    for(auto& tup : nodeVecA3){
        nodeBenchFile2 << get<0>(tup) << " " << get<1>(tup) << endl;
    }
    for(auto& tup : nodeVecA4){
        nodeBenchFile2 << get<0>(tup) << " " << get<1>(tup) << endl;
    }
    nodeBenchFile2.close();
    //////////////N=500 BENCHMARKING/////
    n = 500;
    r = 0.5;
    vector<tuple<int,int>> pathA5;
    vector<tuple<double,double>> nodeVecA5;
    vector<double> timeVecA5;
    vector<int> validFlagA5;
    PRMPlanner** planArrayA51 = new PRMPlanner*[benchNum];
    for (int i = 0; i!= benchNum; i++){
        auto start = std::chrono::high_resolution_clock::now();
        planArrayA51[i] = new PRMPlanner((CSpace2D &) myNewSpace,n,r,qstart,qgoal);
        auto finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = finish - start;
        timeVecA5.push_back(elapsed.count());
        validFlagA5.push_back(planArrayA51[i]->flag2);
        for(auto& tuple : planArrayA51[i]->cameFrom){
            pathA5.push_back(tuple);
        }
        for(auto& tup : planArrayA51[i]->nodeIters){
            nodeVecA5.push_back(tup);
        }
    }
    delete[] planArrayA51;
    r = 1;
    vector<tuple<int,int>> pathA52;
    vector<tuple<double,double>> nodeVecA52;
    vector<double> timeVecA52;
    vector<int> validFlagA52;
    PRMPlanner** planArrayA52 = new PRMPlanner*[benchNum];
//    PRMPlanner* planArrayA1 = (PRMPlanner*)malloc(sizeof(PRMPlanner) * benchNum);
    for (int i = 0; i!= benchNum; i++){
        auto start = std::chrono::high_resolution_clock::now();
        planArrayA52[i] = new PRMPlanner((CSpace2D &) myNewSpace,n,r,qstart,qgoal);
        auto finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = finish - start;
        timeVecA52.push_back(elapsed.count());
        validFlagA52.push_back(planArrayA52[i]->flag2);
        for(auto& tuple : planArrayA52[i]->cameFrom){
            pathA52.push_back(tuple);
        }
        for(auto& tup : planArrayA52[i]->nodeIters){
            nodeVecA52.push_back(tup);
        }

    }
    delete[] planArrayA52;
    r = 1.5;
    vector<tuple<int,int>> pathA53;
    vector<tuple<double,double>> nodeVecA53;
    vector<double> timeVecA53;
    vector<int> validFlagA53;
    PRMPlanner** planArrayA53 = new PRMPlanner*[benchNum];
//    PRMPlanner* planArrayA1 = (PRMPlanner*)malloc(sizeof(PRMPlanner) * benchNum);
    for (int i = 0; i!= benchNum; i++){
        auto start = std::chrono::high_resolution_clock::now();
        planArrayA53[i] = new PRMPlanner((CSpace2D &) myNewSpace,n,r,qstart,qgoal);
        auto finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = finish - start;
        timeVecA53.push_back(elapsed.count());
        validFlagA53.push_back(planArrayA53[i]->flag2);
        for(auto& tuple : planArrayA53[i]->cameFrom){
            pathA53.push_back(tuple);
        }
        for(auto& tup : planArrayA53[i]->nodeIters){
            nodeVecA53.push_back(tup);
        }
    }
    delete[] planArrayA53;
    r = 2;
    vector<tuple<int,int>> pathA54;
    vector<double> timeVecA54;
    vector<tuple<double,double>> nodeVecA54;
    vector<int> validFlagA54;
    PRMPlanner** planArrayA54 = new PRMPlanner*[benchNum];
//    PRMPlanner* planArrayA1 = (PRMPlanner*)malloc(sizeof(PRMPlanner) * benchNum);
    for (int i = 0; i!= benchNum; i++){
        auto start = std::chrono::high_resolution_clock::now();
        planArrayA54[i] = new PRMPlanner((CSpace2D &) myNewSpace,n,r,qstart,qgoal);
        auto finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = finish - start;
        timeVecA54.push_back(elapsed.count());
        validFlagA54.push_back(planArrayA54[i]->flag2);
        for(auto& tuple : planArrayA54[i]->cameFrom){
            pathA54.push_back(tuple);
        }
        for(auto& tup : planArrayA54[i]->nodeIters){
            nodeVecA54.push_back(tup);
        }
    }
    delete[] planArrayA54;
    ofstream timeBenchFile1;
    timeBenchFile1.open("time_bench_n500.txt");
    for(int i = 0; i!=timeVecA5.size(); i++){
        timeBenchFile1 << timeVecA5[i] << endl;
    }
    for(int i = 0; i!=timeVecA52.size(); i++){
        timeBenchFile1 << timeVecA52[i] << endl;
    }
    for(int i = 0; i!=timeVecA53.size(); i++){
        timeBenchFile1 << timeVecA53[i] << endl;
    }
    for(int i = 0; i!=timeVecA54.size(); i++){
        timeBenchFile1 << timeVecA54[i] << endl;
    }
    timeBenchFile1.close();
    ofstream flagBenchFile1;
    flagBenchFile1.open("flag_bench_n500.txt");
    for(int i = 0; i!=validFlagA5.size(); i++){
        flagBenchFile1 << validFlagA5[i] << endl;
    }
    for(int i = 0; i!=validFlagA52.size(); i++){
        flagBenchFile1 << validFlagA52[i] << endl;
    }
    for(int i = 0; i!=validFlagA53.size(); i++){
        flagBenchFile1 << validFlagA53[i] << endl;
    }
    for(int i = 0; i!=validFlagA54.size(); i++){
        flagBenchFile1 << validFlagA54[i] << endl;
    }
    flagBenchFile1.close();

    ofstream pathBenchFile1;
    pathBenchFile1.open("path_bench_n500.txt");
    for(auto& tup : pathA5){
        pathBenchFile1 << get<0>(tup) << " " << get<1>(tup) << endl;
    }
    for(auto& tup : pathA52){
        pathBenchFile1 << get<0>(tup) << " " << get<1>(tup) << endl;
    }
    for(auto& tup : pathA53){
        pathBenchFile1 << get<0>(tup) << " " << get<1>(tup) << endl;
    }
    for(auto& tup : pathA54){
        pathBenchFile1 << get<0>(tup) << " " << get<1>(tup) << endl;
    }
    pathBenchFile1.close();
    ofstream nodeBenchFile1;
    nodeBenchFile1.open("node_bench_n500.txt");
    for(auto& tup : nodeVecA5){
        nodeBenchFile1 << get<0>(tup) << " " << get<1>(tup) << endl;
    }
    for(auto& tup : nodeVecA52){
        nodeBenchFile1 << get<0>(tup) << " " << get<1>(tup) << endl;
    }
    for(auto& tup : nodeVecA53){
        nodeBenchFile1 << get<0>(tup) << " " << get<1>(tup) << endl;
    }
    for(auto& tup : nodeVecA54){
        nodeBenchFile1 << get<0>(tup) << " " << get<1>(tup) << endl;
    }
    nodeBenchFile1.close();
    ////////////////////////////////////BEGIN SECOND PRM SCENARIO///////////////////////////////////////////////////////
    cout << "Beginning Problem 2b Workspace 1" << endl;
    vector<vector<tuple<double, double>>> obsList1;
    // Create the vectors for each obstacle and fill them with the tuples of coordinates
    vector<tuple<double,double>> obsCoords11;

    obsCoords11.emplace_back(1,1);
    obsCoords11.emplace_back(1,5);
    obsCoords11.emplace_back(2,5);
    obsCoords11.emplace_back(2,1);

    vector<tuple<double,double>> obsCoords21;
    obsCoords21.emplace_back(3,4);
    obsCoords21.emplace_back(3,12);
    obsCoords21.emplace_back(4,12);
    obsCoords21.emplace_back(4,4);



    vector<tuple<double,double>> obsCoords3;
    obsCoords3.push_back(tuple<double,double>(3,12));
    obsCoords3.push_back(tuple<double,double>(3,13));
    obsCoords3.push_back(tuple<double,double>(12,13));
    obsCoords3.push_back(tuple<double,double>(12,12));



    vector<tuple<double,double>> obsCoords4;
    obsCoords4.push_back(tuple<double,double>(12,5));
    obsCoords4.push_back(tuple<double,double>(12,13));
    obsCoords4.push_back(tuple<double,double>(13,13));
    obsCoords4.push_back(tuple<double,double>(13,5));



    vector<tuple<double,double>> obsCoords5;
    obsCoords5.push_back(tuple<double,double>(6,5));
    obsCoords5.push_back(tuple<double,double>(6,6));
    obsCoords5.push_back(tuple<double,double>(12,6));
    obsCoords5.push_back(tuple<double,double>(12,5));



    obsList1.push_back(obsCoords11);
    obsList1.push_back(obsCoords21);
    obsList1.push_back(obsCoords3);
    obsList1.push_back(obsCoords4);
    obsList1.push_back(obsCoords5);

    tuple<int,int> xBounds1(-1,13);
    tuple<int,int> yBounds1(-1,13);
    tuple<double,double> qstart1(0,0);
    tuple<double,double> qgoal1(10, 10);
    n = 200;
    r = 2;

    CSpace2D myNewSpace1(obsList1,xBounds1,yBounds1,resolution);
    PRMPlanner myPRMPlan1((CSpace2D &) myNewSpace1,n,r,qstart1,qgoal1);

    ofstream myfile11;
    myfile11.open("nodeGridLocs2.txt");
    for(auto& tuple : myPRMPlan1.nodeIters){
        myfile11 << get<0>(tuple) << " " << get<1>(tuple) << endl;
    }
    myfile11.close();

    ofstream myfile21;
    myfile21.open("connectionList2.txt");
    myfile21 << myPRMPlan1.nodeMap;
    myfile21.close();

    ofstream myfile31;
    myfile31.open("nodePath2.txt");
    for(auto& tuple : myPRMPlan1.nodePath){
        myfile31 << get<0>(tuple) << " " << get<1>(tuple) << endl;
    }
    myfile31.close();

    ofstream myfile37;
    myfile37.open("camefrom2.txt");
    for(auto& tuple : myPRMPlan1.cameFrom){
        myfile37 << get<0>(tuple) << " " << get<1>(tuple) << endl;
    }
    myfile37.close();
    //////////////////////////BEGIN BENCHMARKING FOR WORKSPACE 1 AT 200 NODES//////////////
    n = 200;
    r = 1;
    vector<tuple<int,int>> pathB;
    vector<double> timeVecB;
    vector<int> validFlagB;
    PRMPlanner** planArrayB1 = new PRMPlanner*[benchNum];
    for (int i = 0; i!= benchNum; i++){
        auto start = std::chrono::high_resolution_clock::now();
        planArrayB1[i] = new PRMPlanner((CSpace2D &) myNewSpace1,n,r,qstart1,qgoal1);
        auto finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = finish - start;
        timeVecB.push_back(elapsed.count());
        validFlagB.push_back(planArrayB1[i]->flag2);
        for(auto& tuple : planArrayB1[i]->cameFrom){
            pathB.push_back(tuple);
        }
    }
    delete[] planArrayB1;
    r = 2;
    vector<tuple<int,int>> pathB2;
    vector<double> timeVecB2;
    vector<int> validFlagB2;
    PRMPlanner** planArrayB2 = new PRMPlanner*[benchNum];
    for (int i = 0; i!= benchNum; i++){
        auto start = std::chrono::high_resolution_clock::now();
        planArrayB2[i] = new PRMPlanner((CSpace2D &) myNewSpace1,n,r,qstart1,qgoal1);
        auto finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = finish - start;
        timeVecB2.push_back(elapsed.count());
        validFlagB2.push_back(planArrayB2[i]->flag2);
        for(auto& tuple : planArrayB2[i]->cameFrom){
            pathB2.push_back(tuple);
        }
    }
    delete[] planArrayB2;
    ofstream timeBenchFile3;
    timeBenchFile3.open("time_bench_n200_W1.txt");
    for(int i = 0; i!=timeVecB.size(); i++){
        timeBenchFile3 << timeVecA[i] << endl;
    }
    for(int i = 0; i!=timeVecB2.size(); i++){
        timeBenchFile3 << timeVecA2[i] << endl;
    }
    timeBenchFile3.close();
    ofstream flagBenchFile3;
    flagBenchFile3.open("flag_bench_n200_W1.txt");
    for(int i = 0; i!=validFlagB.size(); i++){
        flagBenchFile3 << validFlagB[i] << endl;
    }
    for(int i = 0; i!=validFlagB2.size(); i++){
        flagBenchFile3 << validFlagB2[i] << endl;
    }
    flagBenchFile3.close();

    ofstream pathBenchFile3;
    pathBenchFile3.open("path_bench_n200_W1.txt");
    for(auto& tup : pathB){
        pathBenchFile3 << get<0>(tup) << " " << get<1>(tup) << endl;
    }
    for(auto& tup : pathB2){
        pathBenchFile3 << get<0>(tup) << " " << get<1>(tup) << endl;
    }
    pathBenchFile3.close();
    //////////////N=500 BENCHMARKING/////
    n = 500;
    r = 1;
    vector<tuple<int,int>> pathB5;
    vector<double> timeVecB5;
    vector<int> validFlagB5;
    PRMPlanner** planArrayB51 = new PRMPlanner*[benchNum];
    for (int i = 0; i!= benchNum; i++){
        auto start = std::chrono::high_resolution_clock::now();
        planArrayB51[i] = new PRMPlanner((CSpace2D &) myNewSpace1,n,r,qstart1,qgoal1);
        auto finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = finish - start;
        timeVecB5.push_back(elapsed.count());
        validFlagB5.push_back(planArrayB51[i]->flag2);
        for(auto& tuple : planArrayB51[i]->cameFrom){
            pathB5.push_back(tuple);
        }
    }
    delete[] planArrayB51;
    r = 2;
    vector<tuple<int,int>> pathB52;
    vector<double> timeVecB52;
    vector<int> validFlagB52;
    PRMPlanner** planArrayB52 = new PRMPlanner*[benchNum];
//    PRMPlanner* planArrayA1 = (PRMPlanner*)malloc(sizeof(PRMPlanner) * benchNum);
    for (int i = 0; i!= benchNum; i++){
        auto start = std::chrono::high_resolution_clock::now();
        planArrayB52[i] = new PRMPlanner((CSpace2D &) myNewSpace1,n,r,qstart1,qgoal1);
        auto finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = finish - start;
        timeVecB52.push_back(elapsed.count());
        validFlagB52.push_back(planArrayB52[i]->flag2);
        for(auto& tuple : planArrayB52[i]->cameFrom){
            pathB52.push_back(tuple);
        }
    }
    delete[] planArrayB52;
    ofstream timeBenchFile4;
    timeBenchFile4.open("time_bench_n500_W1.txt");
    for(int i = 0; i!=timeVecB5.size(); i++){
        timeBenchFile4 << timeVecB5[i] << endl;
    }
    for(int i = 0; i!=timeVecB52.size(); i++){
        timeBenchFile4 << timeVecB52[i] << endl;
    }
    timeBenchFile4.close();
    ofstream flagBenchFile4;
    flagBenchFile4.open("flag_bench_n500_W1.txt");
    for(int i = 0; i!=validFlagB5.size(); i++){
        flagBenchFile4 << validFlagB5[i] << endl;
    }
    for(int i = 0; i!=validFlagB52.size(); i++){
        flagBenchFile4 << validFlagB52[i] << endl;
    }
    flagBenchFile4.close();
    ofstream pathBenchFile4;
    pathBenchFile4.open("path_bench_n500_W1.txt");
    for(auto& tup : pathB5){
        pathBenchFile4 << get<0>(tup) << " " << get<1>(tup) << endl;
    }
    for(auto& tup : pathB52){
        pathBenchFile4 << get<0>(tup) << " " << get<1>(tup) << endl;
    }
    pathBenchFile4.close();
    //////////////N=1000 BENCHMARKING/////
    n = 1000;
    r = 1;
    vector<tuple<int,int>> pathB10;
    vector<double> timeVecB10;
    vector<int> validFlagB10;
    PRMPlanner** planArrayB101 = new PRMPlanner*[benchNum];
    for (int i = 0; i!= benchNum; i++){
        auto start = std::chrono::high_resolution_clock::now();
        planArrayB101[i] = new PRMPlanner((CSpace2D &) myNewSpace1,n,r,qstart1,qgoal1);
        auto finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = finish - start;
        timeVecB10.push_back(elapsed.count());
        validFlagB10.push_back(planArrayB101[i]->flag2);
        for(auto& tuple : planArrayB101[i]->cameFrom){
            pathB10.push_back(tuple);
        }
    }
    delete[] planArrayB101;
    r = 2;
    vector<tuple<int,int>> pathB102;
    vector<double> timeVecB102;
    vector<int> validFlagB102;
    PRMPlanner** planArrayB102 = new PRMPlanner*[benchNum];
    for (int i = 0; i!= benchNum; i++){
        auto start = std::chrono::high_resolution_clock::now();
        planArrayB102[i] = new PRMPlanner((CSpace2D &) myNewSpace1,n,r,qstart1,qgoal1);
        auto finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = finish - start;
        timeVecB102.push_back(elapsed.count());
        validFlagB102.push_back(planArrayB102[i]->flag2);
        for(auto& tuple : planArrayB102[i]->cameFrom){
            pathB102.push_back(tuple);
        }
    }
    delete[] planArrayB102;
    ofstream timeBenchFile5;
    timeBenchFile5.open("time_bench_n1000_W1.txt");
    for(int i = 0; i!=timeVecB10.size(); i++){
        timeBenchFile5 << timeVecB10[i] << endl;
    }
    for(int i = 0; i!=timeVecB102.size(); i++){
        timeBenchFile5 << timeVecB102[i] << endl;
    }
    timeBenchFile5.close();
    ofstream flagBenchFile5;
    flagBenchFile5.open("flag_bench_n1000_W1.txt");
    for(int i = 0; i!=validFlagB10.size(); i++){
        flagBenchFile5 << validFlagB10[i] << endl;
    }
    for(int i = 0; i!=validFlagB102.size(); i++){
        flagBenchFile5 << validFlagB102[i] << endl;
    }
    flagBenchFile5.close();
    ofstream pathBenchFile5;
    pathBenchFile5.open("path_bench_n1000_W1.txt");
    for(auto& tup : pathB10){
        pathBenchFile5 << get<0>(tup) << " " << get<1>(tup) << endl;
    }
    for(auto& tup : pathB102){
        pathBenchFile5 << get<0>(tup) << " " << get<1>(tup) << endl;
    }
    pathBenchFile5.close();
    ///////////////////////////////////////////////////////BEGIN WORKSPACE 2 FOR QUESTION 2 B///////////////////////////
    cout << "Beginning Problem 2b Workspace 2" << endl;
    vector<vector<tuple<double, double>>> obsList2;
    vector<tuple<double,double>> obsCoords10;

    obsCoords10.push_back(tuple<double,double>(-6,-6));
    obsCoords10.push_back(tuple<double,double>(-6,-5));
    obsCoords10.push_back(tuple<double,double>(25,-5));
    obsCoords10.push_back(tuple<double,double>(25,-6));


    vector<tuple<double,double>> obsCoords20;
    obsCoords20.push_back(tuple<double,double>(-6,5));
    obsCoords20.push_back(tuple<double,double>(-6,6));
    obsCoords20.push_back(tuple<double,double>(30,6));
    obsCoords20.push_back(tuple<double,double>(30,5));



    vector<tuple<double,double>> obsCoords30;
    obsCoords30.push_back(tuple<double,double>(-6,-5));
    obsCoords30.push_back(tuple<double,double>(-6,5));
    obsCoords30.push_back(tuple<double,double>(-5,5));
    obsCoords30.push_back(tuple<double,double>(-5,-5));



    vector<tuple<double,double>> obsCoords40;
    obsCoords40.push_back(tuple<double,double>(4,-5));
    obsCoords40.push_back(tuple<double,double>(4,1));
    obsCoords40.push_back(tuple<double,double>(5,1));
    obsCoords40.push_back(tuple<double,double>(5,-5));



    vector<tuple<double,double>> obsCoords50;
    obsCoords50.push_back(tuple<double,double>(9,0));
    obsCoords50.push_back(tuple<double,double>(9,5));
    obsCoords50.push_back(tuple<double,double>(10,5));
    obsCoords50.push_back(tuple<double,double>(10,0));



    vector<tuple<double,double>> obsCoords60;
    obsCoords60.push_back(tuple<double,double>(14,-5));
    obsCoords60.push_back(tuple<double,double>(14,1));
    obsCoords60.push_back(tuple<double,double>(15,1));
    obsCoords60.push_back(tuple<double,double>(15,-5));



    vector<tuple<double,double>> obsCoords70;
    obsCoords70.push_back(tuple<double,double>(19,0));
    obsCoords70.push_back(tuple<double,double>(19,5));
    obsCoords70.push_back(tuple<double,double>(20,5));
    obsCoords70.push_back(tuple<double,double>(20,0));



    vector<tuple<double,double>> obsCoords80;
    obsCoords80.push_back(tuple<double,double>(24,-5));
    obsCoords80.push_back(tuple<double,double>(24,1));
    obsCoords80.push_back(tuple<double,double>(25,1));
    obsCoords80.push_back(tuple<double,double>(25,-5));



    vector<tuple<double,double>> obsCoords90;
    obsCoords90.push_back(tuple<double,double>(29,0));
    obsCoords90.push_back(tuple<double,double>(29,5));
    obsCoords90.push_back(tuple<double,double>(30,5));
    obsCoords90.push_back(tuple<double,double>(30,0));

    obsList2.push_back(obsCoords10);
    obsList2.push_back(obsCoords20);
    obsList2.push_back(obsCoords30);
    obsList2.push_back(obsCoords40);
    obsList2.push_back(obsCoords50);
    obsList2.push_back(obsCoords60);
    obsList2.push_back(obsCoords70);
    obsList2.push_back(obsCoords80);
    obsList2.push_back(obsCoords90);

    tuple<int,int> xBounds2(-6,36);
    tuple<int,int> yBounds2(-6,6);
    tuple<double,double> qstart2(0,0);
    tuple<double,double> qgoal2(35, 0);
    r = 2;
    n = 500;
    CSpace2D myNewSpace2(obsList2,xBounds2,yBounds2,resolution);
    PRMPlanner myPRMPlan2((CSpace2D &) myNewSpace2,n,r,qstart2,qgoal2);

    ofstream myfile22;
    myfile22.open("nodeGridLocs3.txt");
    for(auto& tuple : myPRMPlan2.nodeIters){
        myfile22 << get<0>(tuple) << " " << get<1>(tuple) << endl;
    }
    myfile22.close();

    ofstream myfile23;
    myfile23.open("connectionList3.txt");
    myfile23 << myPRMPlan2.nodeMap;
    myfile23.close();

    ofstream myfile33;
    myfile33.open("nodePath3.txt");
    for(auto& tuple : myPRMPlan2.nodePath){
        myfile33 << get<0>(tuple) << " " << get<1>(tuple) << endl;
    }
    myfile33.close();

    ofstream myfile39;
    myfile39.open("camefrom3.txt");
    for(auto& tuple : myPRMPlan2.cameFrom){
        myfile39 << get<0>(tuple) << " " << get<1>(tuple) << endl;
    }
    myfile39.close();
    //////////////////////////BEGIN BENCHMARKING FOR WORKSPACE 2 AT 200 NODES//////////////
    n = 200;
    r = 1;
    vector<tuple<int,int>> pathW2;
    vector<double> timeVecW2;
    vector<int> validFlagW2;
    PRMPlanner** planArrayW21 = new PRMPlanner*[benchNum];
//    PRMPlanner* planArrayA1 = (PRMPlanner*)malloc(sizeof(PRMPlanner) * benchNum);
    for (int i = 0; i!= benchNum; i++){
        auto start = std::chrono::high_resolution_clock::now();
        planArrayW21[i] = new PRMPlanner((CSpace2D &) myNewSpace2,n,r,qstart2,qgoal2);
        auto finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = finish - start;
        timeVecW2.push_back(elapsed.count());
        validFlagW2.push_back(planArrayW21[i]->flag2);
        for(auto& tuple : planArrayW21[i]->cameFrom){
            pathW2.push_back(tuple);
        }
    }
    delete[] planArrayW21;
    r = 2;
    vector<tuple<int,int>> pathW22;
    vector<double> timeVecW22;
    vector<int> validFlagW22;
    PRMPlanner** planArrayW22 = new PRMPlanner*[benchNum];
////    PRMPlanner* planArrayA1 = (PRMPlanner*)malloc(sizeof(PRMPlanner) * benchNum);
    for (int i = 0; i!= benchNum; i++){
        auto start = std::chrono::high_resolution_clock::now();
        planArrayW22[i] = new PRMPlanner((CSpace2D &) myNewSpace2,n,r,qstart2,qgoal2);
        auto finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = finish - start;
        timeVecW22.push_back(elapsed.count());
        validFlagW22.push_back(planArrayW22[i]->flag2);
        for(auto& tuple : planArrayW22[i]->cameFrom){
            pathW22.push_back(tuple);
        }
    }
    delete[] planArrayW22;
    ofstream timeBenchFile6;
    timeBenchFile6.open("time_bench_n200_W2.txt");
    for(int i = 0; i!=timeVecW2.size(); i++){
        timeBenchFile6 << timeVecW22[i] << endl;
    }
    for(int i = 0; i!=timeVecW22.size(); i++){
        timeBenchFile6 << timeVecW22[i] << endl;
    }
    timeBenchFile6.close();
    ofstream flagBenchFile6;
    flagBenchFile6.open("flag_bench_n200_W2.txt");
    for(int i = 0; i!=validFlagW2.size(); i++){
        flagBenchFile6 << validFlagW2[i] << endl;
    }
    for(int i = 0; i!=validFlagW22.size(); i++){
        flagBenchFile6 << validFlagW22[i] << endl;
    }
    flagBenchFile6.close();

    ofstream pathBenchFile6;
    pathBenchFile6.open("path_bench_n200_W2.txt");
    for(auto& tup : pathW2){
        pathBenchFile6 << get<0>(tup) << " " << get<1>(tup) << endl;
    }
    for(auto& tup : pathW22){
        pathBenchFile6 << get<0>(tup) << " " << get<1>(tup) << endl;
    }
    pathBenchFile6.close();
    //////////////N=500 BENCHMARKING/////
    n = 500;
    r = 1;
    vector<tuple<int,int>> pathW25;
    vector<double> timeVecW25;
    vector<int> validFlagW25;
    PRMPlanner** planArrayW251 = new PRMPlanner*[benchNum];
    for (int i = 0; i!= benchNum; i++){
        auto start = std::chrono::high_resolution_clock::now();
        planArrayW251[i] = new PRMPlanner((CSpace2D &) myNewSpace2,n,r,qstart2,qgoal2);
        auto finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = finish - start;
        timeVecW25.push_back(elapsed.count());
        validFlagW25.push_back(planArrayW251[i]->flag2);
        for(auto& tuple : planArrayW251[i]->cameFrom){
            pathW25.push_back(tuple);
        }
    }
    delete[] planArrayW251;
    r = 2;
    vector<tuple<int,int>> pathW252;
    vector<double> timeVecW252;
    vector<int> validFlagW252;
    PRMPlanner** planArrayW252 = new PRMPlanner*[benchNum];
////    PRMPlanner* planArrayA1 = (PRMPlanner*)malloc(sizeof(PRMPlanner) * benchNum);
    for (int i = 0; i!= benchNum; i++){
        auto start = std::chrono::high_resolution_clock::now();
        planArrayW252[i] = new PRMPlanner((CSpace2D &) myNewSpace2,n,r,qstart2,qgoal2);
        auto finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = finish - start;
        timeVecW252.push_back(elapsed.count());
        validFlagW252.push_back(planArrayW252[i]->flag2);
        for(auto& tuple : planArrayW252[i]->cameFrom){
            pathW252.push_back(tuple);
        }
    }
    delete[] planArrayW252;
    ofstream timeBenchFile7;
    timeBenchFile7.open("time_bench_n500_W2.txt");
    for(int i = 0; i!=timeVecW25.size(); i++){
        timeBenchFile7 << timeVecW25[i] << endl;
    }
    for(int i = 0; i!=timeVecW252.size(); i++){
        timeBenchFile7 << timeVecW252[i] << endl;
    }
    timeBenchFile7.close();
    ofstream flagBenchFile7;
    flagBenchFile7.open("flag_bench_n500_W2.txt");
    for(int i = 0; i!=validFlagW25.size(); i++){
        flagBenchFile7 << validFlagW25[i] << endl;
    }
    for(int i = 0; i!=validFlagW252.size(); i++){
        flagBenchFile7 << validFlagW252[i] << endl;
    }
    flagBenchFile7.close();
    ofstream pathBenchFile7;
    pathBenchFile7.open("path_bench_n500_W2.txt");
    for(auto& tup : pathW25){
        pathBenchFile7 << get<0>(tup) << " " << get<1>(tup) << endl;
    }
    for(auto& tup : pathW252){
        pathBenchFile7 << get<0>(tup) << " " << get<1>(tup) << endl;
    }
    pathBenchFile7.close();
    //////////////N=1000 BENCHMARKING/////
    n = 1000;
    r = 1;
    vector<tuple<int,int>> pathW210;
    vector<double> timeVecW210;
    vector<int> validFlagW210;
    PRMPlanner** planArrayW2101 = new PRMPlanner*[benchNum];
    for (int i = 0; i!= benchNum; i++){
        auto start = std::chrono::high_resolution_clock::now();
        planArrayW2101[i] = new PRMPlanner((CSpace2D &) myNewSpace2,n,r,qstart2,qgoal2);
        auto finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = finish - start;
        timeVecW210.push_back(elapsed.count());
        validFlagW210.push_back(planArrayW2101[i]->flag2);
        for(auto& tuple : planArrayW2101[i]->cameFrom){
            pathW210.push_back(tuple);
        }
    }
    delete[] planArrayW2101;
    r = 2;
    vector<tuple<int,int>> pathW2102;
    vector<double> timeVecW2102;
    vector<int> validFlagW2102;
    PRMPlanner** planArrayW2102 = new PRMPlanner*[benchNum];
    for (int i = 0; i!= benchNum; i++){
        auto start = std::chrono::high_resolution_clock::now();
        planArrayW2102[i] = new PRMPlanner((CSpace2D &) myNewSpace2,n,r,qstart2,qgoal2);
        auto finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = finish - start;
        timeVecW2102.push_back(elapsed.count());
        validFlagW2102.push_back(planArrayW2102[i]->flag2);
        for(auto& tuple : planArrayW2102[i]->cameFrom){
            pathW2102.push_back(tuple);
        }
    }
    delete[] planArrayW2102;
    ofstream timeBenchFile8;
    timeBenchFile8.open("time_bench_n1000_W2.txt");
    for(int i = 0; i!=timeVecW210.size(); i++){
        timeBenchFile8 << timeVecW210[i] << endl;
    }
    for(int i = 0; i!=timeVecW2102.size(); i++){
        timeBenchFile8 << timeVecW2102[i] << endl;
    }
    timeBenchFile8.close();
    ofstream flagBenchFile8;
    flagBenchFile8.open("flag_bench_n1000_W2.txt");
    for(int i = 0; i!=validFlagW210.size(); i++){
        flagBenchFile8 << validFlagW210[i] << endl;
    }
    for(int i = 0; i!=validFlagW2102.size(); i++){
        flagBenchFile8 << validFlagW2102[i] << endl;
    }
    flagBenchFile8.close();
    ofstream pathBenchFile8;
    pathBenchFile8.open("path_bench_n1000_W2.txt");
    for(auto& tup : pathW210){
        pathBenchFile8 << get<0>(tup) << " " << get<1>(tup) << endl;
    }
    for(auto& tup : pathW2102){
        pathBenchFile8 << get<0>(tup) << " " << get<1>(tup) << endl;
    }
    pathBenchFile8.close();
    /////////////////////////////////////////BEGIN RRT FOR THE 2 SQUARE WORKSPACE///////////////////////////////////////
    cout<< "Beginning 2 Square RRT" << endl;
    r = 0.5;
    double ep = 0.25;
    n = 5000;
    double pgoal = 0.05;
    GoalRRT realFirst((CSpace2D &) myNewSpace,n,r,qstart,qgoal,pgoal,ep);
    ofstream myfile231;
    myfile231.open("nodeGridLocs27.txt");
    for(auto& tuple : realFirst.nodeIters){
        myfile231 << get<0>(tuple) << " " << get<1>(tuple) << endl;
    }
    myfile231.close();

    ofstream myfile232;
    myfile232.open("connectionList27.txt");
    myfile232 << realFirst.nodeMap;
    myfile232.close();

    ofstream myfile233;
    myfile233.open("nodePath27.txt");
    for(auto& tuple : realFirst.nodePath){
        myfile233 << get<0>(tuple) << " " << get<1>(tuple) << endl;
    }
    myfile233.close();

    ofstream myfile234;
    myfile234.open("camefrom27.txt");
    for(auto& tuple : realFirst.cameFrom){
        myfile234 << get<0>(tuple) << " " << get<1>(tuple) << endl;
    }
    myfile234.close();

    vector<tuple<double,double>> nodeVecRRTA;
    vector<tuple<int,int>> pathRRTA;
    vector<double> timeVecRRTA;
    vector<int> validFlagRRTA;
    GoalRRT** planArrayRRTA = new GoalRRT*[benchNum];
    for (int i = 0; i!= benchNum; i++){
        auto start = std::chrono::high_resolution_clock::now();
        planArrayRRTA[i] = new GoalRRT((CSpace2D &) myNewSpace,n,r,qstart,qgoal,pgoal,ep);
        auto finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = finish - start;
        timeVecRRTA.push_back(elapsed.count());
        validFlagRRTA.push_back(planArrayRRTA[i]->flag2);
        for(auto& tuple : planArrayRRTA[i]->cameFrom){
            pathRRTA.push_back(tuple);
        }
        for(auto& tup : planArrayRRTA[i]->nodeIters){
            nodeVecRRTA.push_back(tup);
        }
    }
    delete[] planArrayRRTA;
    ofstream rrtAtime;
    rrtAtime.open("rrt_a_time.txt");
    for(auto& in : timeVecRRTA){
        rrtAtime << in << endl;
    }
    rrtAtime.close();
    ofstream rrtAflag;
    rrtAflag.open("rrt_a_flag.txt");
    for(auto& in : validFlagRRTA){
        rrtAflag << in << endl;
    }
    rrtAflag.close();
    ofstream rrtApath;
    rrtApath.open("rrt_a_path.txt");
    for(auto& in : pathRRTA){
        rrtApath << get<0>(in) << " " << get<1>(in) << endl;
    }
    rrtApath.close();
    ofstream rrtAnode;
    rrtAnode.open("rrt_a_node.txt");
    for(auto& in : nodeVecRRTA){
        rrtAnode << get<0>(in) << " " << get<1>(in) << endl;
    }
    rrtAnode.close();

    //////////////////////////////////BEGIN QUESTION 3 PART A///////////////////////////////////////////////////////////
    cout << "Beginning Problem 3a" << endl;

    GoalRRT myFirstRRT((CSpace2D &) myNewSpace1,n,r,qstart1,qgoal1,pgoal,ep);

    ofstream myfile01;
    myfile01.open("nodeGridLocs4.txt");
    for(auto& tuple : myFirstRRT.nodeIters){
        myfile01 << get<0>(tuple) << " " << get<1>(tuple) << endl;
    }
    myfile01.close();

    ofstream myfile02;
    myfile02.open("connectionList4.txt");
    myfile02 << myFirstRRT.nodeMap;
    myfile02.close();

    ofstream myfile03;
    myfile03.open("nodePath4.txt");
    for(auto& tuple : myFirstRRT.nodePath){
        myfile03 << get<0>(tuple) << " " << get<1>(tuple) << endl;
    }
    myfile03.close();

    ofstream myfile04;
    myfile04.open("camefrom4.txt");
    for(auto& tuple : myFirstRRT.cameFrom){
        myfile04 << get<0>(tuple) << " " << get<1>(tuple) << endl;
    }
    myfile04.close();

    vector<tuple<double,double>> nodeVecRRTW1;
    vector<tuple<int,int>> pathRRTW1;
    vector<double> timeVecRRTW1;
    vector<int> validFlagRRTW1;
    GoalRRT** planArrayRRTW1 = new GoalRRT*[benchNum];
    for (int i = 0; i!= benchNum; i++){
        auto start = std::chrono::high_resolution_clock::now();
        planArrayRRTW1[i] = new GoalRRT((CSpace2D &) myNewSpace1,n,r,qstart1,qgoal1,pgoal,ep);
        auto finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = finish - start;
        timeVecRRTW1.push_back(elapsed.count());
        validFlagRRTW1.push_back(planArrayRRTW1[i]->flag2);
        for(auto& tuple : planArrayRRTW1[i]->cameFrom){
            pathRRTW1.push_back(tuple);
        }
        for(auto& tup : planArrayRRTW1[i]->nodeIters){
            nodeVecRRTW1.push_back(tup);
        }
    }
    delete[] planArrayRRTW1;
    ofstream rrtW1time;
    rrtW1time.open("rrt_W1_time.txt");
    for(auto& in : timeVecRRTW1){
        rrtW1time << in << endl;
    }
    rrtW1time.close();
    ofstream rrtW1flag;
    rrtW1flag.open("rrt_W1_flag.txt");
    for(auto& in : validFlagRRTW1){
        rrtW1flag << in << endl;
    }
    rrtW1flag.close();
    ofstream rrtW1path;
    rrtW1path.open("rrt_W1_path.txt");
    for(auto& in : pathRRTW1){
        rrtW1path << get<0>(in) << " " << get<1>(in) << endl;
    }
    rrtW1path.close();
    ofstream rrtW1node;
    rrtW1node.open("rrt_W1_node.txt");
    for(auto& in : nodeVecRRTW1){
        rrtW1node << get<0>(in) << " " << get<1>(in) << endl;
    }
    rrtW1node.close();
//////////////////////////////////////BEGIN QUESTION 3 PART B///////////////////////////////////////////////////////////
    cout << "Beginning Problem 3b" << endl;
    GoalRRT myFirstRRT1((CSpace2D &) myNewSpace2,n,r,qstart2,qgoal2,pgoal,ep);

    ofstream myfile91;
    myfile91.open("nodeGridLocs5.txt");
    for(auto& tuple : myFirstRRT1.nodeIters){
        myfile91 << get<0>(tuple) << " " << get<1>(tuple) << endl;
    }
    myfile91.close();

    ofstream myfile92;
    myfile92.open("connectionList5.txt");
    myfile92 << myFirstRRT1.nodeMap;
    myfile92.close();

    ofstream myfile93;
    myfile93.open("nodePath5.txt");
    for(auto& tuple : myFirstRRT1.nodePath){
        myfile93 << get<0>(tuple) << " " << get<1>(tuple) << endl;
    }
    myfile93.close();

    ofstream myfile94;
    myfile94.open("camefrom5.txt");
    for(auto& tuple : myFirstRRT1.cameFrom){
        myfile94 << get<0>(tuple) << " " << get<1>(tuple) << endl;
    }
    myfile94.close();

    vector<tuple<double,double>> nodeVecRRTW2;
    vector<tuple<int,int>> pathRRTW2;
    vector<double> timeVecRRTW2;
    vector<int> validFlagRRTW2;
    GoalRRT** planArrayRRTW2 = new GoalRRT*[benchNum];
    for (int i = 0; i!= benchNum; i++){
        auto start = std::chrono::high_resolution_clock::now();
        planArrayRRTW2[i] = new GoalRRT((CSpace2D &) myNewSpace2,n,r,qstart2,qgoal2,pgoal,ep);
        auto finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = finish - start;
        timeVecRRTW2.push_back(elapsed.count());
        validFlagRRTW2.push_back(planArrayRRTW2[i]->flag2);
        for(auto& tuple : planArrayRRTW2[i]->cameFrom){
            pathRRTW2.push_back(tuple);
        }
        for(auto& tup : planArrayRRTW2[i]->nodeIters){
            nodeVecRRTW2.push_back(tup);
        }
    }
    delete[] planArrayRRTW2;
    ofstream rrtW2time;
    rrtW2time.open("rrt_W2_time.txt");
    for(auto& in : timeVecRRTW2){
        rrtW2time << in << endl;
    }
    rrtW2time.close();
    ofstream rrtW2flag;
    rrtW2flag.open("rrt_W2_flag.txt");
    for(auto& in : validFlagRRTW2){
        rrtW2flag << in << endl;
    }
    rrtW2flag.close();
    ofstream rrtW2path;
    rrtW2path.open("rrt_W2_path.txt");
    for(auto& in : pathRRTW2){
        rrtW2path << get<0>(in) << " " << get<1>(in) << endl;
    }
    rrtW2path.close();
    ofstream rrtW2node;
    rrtW2node.open("rrt_W2_node.txt");
    for(auto& in : nodeVecRRTW2){
        rrtW2node << get<0>(in) << " " << get<1>(in) << endl;
    }
    rrtW2node.close();

    ///////////////////////////////////////////////QUESTION 1///////////////////////////////////////////////////////////
    vector<tuple<double,double>> nodeIters;
    //root
    nodeIters.emplace_back(0,10);
    //A
    nodeIters.emplace_back(-1,9);
    //B
    nodeIters.emplace_back(0,8);
    //C
    nodeIters.emplace_back(1,9);
    //D
    nodeIters.emplace_back(-2,9);
    //E
    nodeIters.emplace_back(-2,5);
    //F
    nodeIters.emplace_back(-1,7);
    //G
    nodeIters.emplace_back(-1,5);
    //H
    nodeIters.emplace_back(0,5);
    //I
    nodeIters.emplace_back(1,5);
    //J
    nodeIters.emplace_back(1,7);
    //K
    nodeIters.emplace_back(2,5);
    //L
    nodeIters.emplace_back(2,9);
    //GOAL
    nodeIters.emplace_back(0,3);
    // Now a heuristic vec
    vector<int> heuristic;
    heuristic.emplace_back(0);
    heuristic.emplace_back(3);
    heuristic.emplace_back(2);
    heuristic.emplace_back(3);
    heuristic.emplace_back(3);
    heuristic.emplace_back(1);
    heuristic.emplace_back(3);
    heuristic.emplace_back(2);
    heuristic.emplace_back(1);
    heuristic.emplace_back(2);
    heuristic.emplace_back(3);
    heuristic.emplace_back(2);
    heuristic.emplace_back(3);

    ArrayXXd nodeMap = ArrayXXd::Zero(14,14);
    ArrayXXd edgeMap = ArrayXXd::Zero(14,14);
    //root connections
    nodeMap(0,1) = 1;
    nodeMap(0,2) = 1;
    nodeMap(0,3) = 1;
    //A connections
    nodeMap(1,0) = 1;
    nodeMap(1,4) = 1;
    nodeMap(1,5) = 1;
    nodeMap(1,6) = 1;
    //B connections
    nodeMap(2,0) = 1;
    nodeMap(2,7) = 1;
    nodeMap(2,8) = 1;
    nodeMap(2,9) = 1;
    //C connections
    nodeMap(3,0) = 1;
    nodeMap(3,10) = 1;
    nodeMap(3,11) = 1;
    nodeMap(3,12) = 1;
    //D connections
    nodeMap(4,1) = 1;
    //E connections
    nodeMap(5,1) = 1;
    nodeMap(5,13) = 1;
    //F connections
    nodeMap(6,1) = 1;
    //G connections
    nodeMap(7,2) = 1;
    nodeMap(7,13) = 1;
    //H connections
    nodeMap(8,2) = 1;
    //I connections
    nodeMap(9,2) = 1;
    nodeMap(9,13) = 1;
    //J connections
    nodeMap(10,3) = 1;
    //K connections
    nodeMap(11,3) = 1;
    nodeMap(11,13) = 1;
    //L connections
    nodeMap(12,3) = 1;
    //Goal connections
    nodeMap(13,5) = 1;
    nodeMap(13,7) = 1;
    nodeMap(13,9) = 1;
    nodeMap(13,11) = 1;

    //root connections
    edgeMap(0,1) = 1;
    edgeMap(0,2) = 1;
    edgeMap(0,3) = 1;
    //A connections
    edgeMap(1,0) = 1;
    edgeMap(1,4) = 1;
    edgeMap(1,5) = 1;
    edgeMap(1,6) = 3;
    //B connections
    edgeMap(2,0) = 1;
    edgeMap(2,7) = 4;
    edgeMap(2,8) = 1;
    edgeMap(2,9) = 2;
    //C connections
    edgeMap(3,0) = 1;
    edgeMap(3,10) = 1;
    edgeMap(3,11) = 1;
    edgeMap(3,12) = 1;
    //D connections
    edgeMap(4,1) = 1;
    //E connections
    edgeMap(5,1) = 1;
    edgeMap(5,13) = 3;
    //F connections
    edgeMap(6,1) = 3;
    //G connections
    edgeMap(7,2) = 4;
    edgeMap(7,13) = 3;
    //H connections
    edgeMap(8,2) = 1;
    //I connections
    edgeMap(9,2) = 2;
    edgeMap(9,13) = 3;
    //J connections
    edgeMap(10,3) = 1;
    //K connections
    edgeMap(11,3) = 1;
    edgeMap(11,13) = 2;
    //L connections
    edgeMap(12,3) = 1;
    //Goal connections
    edgeMap(13,5) = 3;
    edgeMap(13,7) = 3;
    edgeMap(13,9) = 3;
    edgeMap(13,11) = 2;
    // Begin A STAR
    Astar myFirstAstar(nodeMap,edgeMap,nodeIters,heuristic);

    ofstream edgeMap1;
    edgeMap1.open("Astar_edgeMap1.txt");
    edgeMap1 << edgeMap;
    edgeMap1.close();

    ofstream nodeMap1;
    nodeMap1.open("Astar_nodeMap1.txt");
    nodeMap1 << nodeMap;
    nodeMap1.close();

    ofstream nodeIters1;
    nodeIters1.open("Astar_nodeIters.txt");
    for(auto& tup : nodeIters){
        nodeIters1 << get<0>(tup) << " " << get<1>(tup) << endl;
    }
    nodeIters1.close();


    return 0;
}

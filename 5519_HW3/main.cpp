#include <iostream>
#include <fstream>
#include "WavefrontPlanner.h"
#include "TwoLink.h"
#include "TwoDManipulatorCspace.h"
int main() {

    /////////////////////////////////////////////////HW 3 Question 2////////////////////////////////////////////////////
    cout << "==========BEGINNING FIRST GRADIENT SCENARIO==============="<<endl;
    // Create a vector of obstacles for the first workspace
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
    // Create necessary constants
    int resolution = 50;
    tuple<int,int> xBounds(0,10);
    tuple<int,int> yBounds(-3,3);
    // Instantiate cspace
    CSpace2D myNewSpace(obsList,xBounds,yBounds,resolution);
    myNewSpace.createBrushGrid();
    //Print grid to files
    cout << "Printing To File" << endl;
    ofstream myfile;
    myfile.open("BrushFire1.txt");
    //Write the brush grid to a file so we can check it with a MATLAB plot
    for(int i = 0; i <= resolution-1; i ++){
        for(int j = 0; j <= resolution-1; j++) {
            myfile << j << " " << i << " " <<  myNewSpace.brushDistList(j,i) << endl;
        }
    }
    myfile.close();

    ofstream myfile2;
    myfile2.open("DiscGrid1.txt");
    for(auto& tuple: myNewSpace.discCoordList) {
        myfile2 << get<0>(tuple) << " " << get<1>(tuple) << endl;
    }
    myfile2.close();

    cout << "Beginning Gradient Planning" << endl;
    /////Gradient Hyperparameters///////
    vector<double> Qstar;
    Qstar.push_back(2.5);
    Qstar.push_back(100);
    double dstar = 8;
    tuple<double,double> qstart(0,0);
    tuple<double,double> qgoal(10, 0);
    double attGain = 10;
    double repGain = 100;
    ////////////////////////////////////
    // Create gradient planner object
    GradientPlanner myGrad((CSpace2D &) myNewSpace, Qstar,dstar, qstart, qgoal,attGain,repGain);
    myGrad.cVec.emplace_back(4,1);
    myGrad.cVec.emplace_back(7,-1);
    vector<tuple<double,double>> potField;
    potField = myGrad.getPotentialField();
    // Print the interesting data to files
    ofstream myfile4;
    myfile4.open("PotentialField.txt");
    for(auto& tuple : potField){
        myfile4 << get<0>(tuple) << " " << get<1>(tuple) << endl;
    }
    myfile4.close();


    myGrad.gradientDescent();
    vector<tuple<double,double>> pathToGoal = myGrad.pathList;
    ofstream myfile3;
    myfile3.open("GradPath.txt");
    for(auto& tuple : pathToGoal){
        myfile3 << get<0>(tuple) << " " << get<1>(tuple) << endl;
    }
    myfile3.close();
    /////////////////////////////////////////////QUESTION 2 PART B//////////////////////////////////////////////////////
    cout << "==========BEGINNING SECOND GRADIENT SCENARIO==============="<<endl;
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

    tuple<int,int> xBounds1(0,15);
    tuple<int,int> yBounds1(0,14);

    CSpace2D myNewSpace1(obsList1,xBounds1,yBounds1,resolution);

    ofstream myfile21;
    myfile21.open("DiscGrid11.txt");
    for(auto& tuple: myNewSpace1.discCoordList) {
        myfile21 << get<0>(tuple) << " " << get<1>(tuple) << endl;
    }
    myfile21.close();
    /////Gradient Hyperparameters///////
    vector<double> Qstar1;
    Qstar1.push_back(2.85);
    Qstar1.push_back(1.5);
    Qstar1.push_back(1.5);
    Qstar1.push_back(1.5);
    Qstar1.push_back(7.75);
    double dstar1 = 2.79;
    tuple<double,double> qstart1(0,0);
    tuple<double,double> qgoal1(10, 10);
    double attGain1 = 8;
    double repGain1 = 640;
    ///////////////////////////////////
    // Create grad planner object
    GradientPlanner myGrad1((CSpace2D &) myNewSpace1, Qstar1,dstar1, qstart1, qgoal1,attGain1,repGain1);
    myGrad1.cVec.emplace_back(1.5,3);
    myGrad1.cVec.emplace_back(3.5,8.5);
    myGrad1.cVec.emplace_back(8,12.5);
    myGrad1.cVec.emplace_back(12.5,9);
    myGrad1.cVec.emplace_back(9,5.5);
    vector<tuple<double,double>> potField1;
    potField1 = myGrad1.getPotentialField();
    ofstream myfile41;
    myfile41.open("PotentialField1.txt");
    for(auto& tuple : potField1){
        myfile41 << get<0>(tuple) << " " << get<1>(tuple) << endl;
    }
    myfile41.close();

    myGrad1.gradientDescent();
    vector<tuple<double,double>> pathToGoal1 = myGrad1.pathList;
    ofstream myfile31;
    myfile31.open("GradPath1.txt");
    for(auto& tuple : pathToGoal1){
        myfile31 << get<0>(tuple) << " " << get<1>(tuple) << endl;
    }
    myfile31.close();

///////////////////////////////////////////////3rd Gradient Workspace///////////////////////////////////////////////////
    cout << "==========BEGINNING THIRD GRADIENT SCENARIO==============="<<endl;
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

    tuple<int,int> xBounds2(-10,35);
    tuple<int,int> yBounds2(-8,8);

    CSpace2D myNewSpace2(obsList2,xBounds2,yBounds2,resolution);

    ofstream myfile22;
    myfile22.open("DiscGrid2.txt");
    for(auto& tuple: myNewSpace2.discCoordList) {
        myfile22 << get<0>(tuple) << " " << get<1>(tuple) << endl;
    }
    myfile22.close();

    vector<double> Qstar2;
    Qstar2.push_back(8);//Bottom wall
    Qstar2.push_back(8);//Top wall
    Qstar2.push_back(8);//Left wall
    Qstar2.push_back(80);//First bot tooth
    Qstar2.push_back(7);//First top tooth
    Qstar2.push_back(115);//2nd bot tooth
    Qstar2.push_back(7);//2nd top tooth
    Qstar2.push_back(115);//3rd bot tooth
    Qstar2.push_back(7);//3rd top tooth
    double dstar2 = 5;
    tuple<double,double> qstart2(0,0);
    tuple<double,double> qgoal2(35, 0);
    double attGain2 = 9;
    double repGain2 = 843;
    GradientPlanner myGrad2((CSpace2D &) myNewSpace2, Qstar2,dstar2, qstart2, qgoal2,attGain2,repGain2);
    myGrad2.cVec.emplace_back(-6,9);
    myGrad2.cVec.emplace_back(12,6);
    myGrad2.cVec.emplace_back(-5.5,0);
    myGrad2.cVec.emplace_back(4.5,-2);
    myGrad2.cVec.emplace_back(9.5,2.5);
    myGrad2.cVec.emplace_back(14.5,-2);
    myGrad2.cVec.emplace_back(19.5,2.5);
    myGrad2.cVec.emplace_back(24.5,-2);
    myGrad2.cVec.emplace_back(29.5,2.5);
    vector<tuple<double,double>> potField2;
    potField2 = myGrad2.getPotentialField();
    ofstream myfile42;
    myfile42.open("PotentialField2.txt");
    for(auto& tuple : potField2){
        myfile42 << get<0>(tuple) << " " << get<1>(tuple) << endl;
    }
    myfile42.close();

    myGrad2.gradientDescent();
    vector<tuple<double,double>> pathToGoal2 = myGrad2.pathList;
    ofstream myfile32;
    myfile32.open("GradPath2.txt");
    for(auto& tuple : pathToGoal2){
        myfile32 << get<0>(tuple) << " " << get<1>(tuple) << endl;
    }
    myfile32.close();

    //////////////////////////////////////////////////////////QUESTION 3////////////////////////////////////////////////
    cout << "==========BEGINNING FIRST WAVEPLANNER SCENARIO==============="<<endl;
    double dx = 0.25;

    tuple<int,int> xBounds15(-3,15);
    tuple<int,int> yBounds15(-3,15);

    CSpace2D waveCSpace1(dx, obsList1,xBounds15,yBounds15,qgoal1,qstart1);
    ofstream myWavDisc;
    myWavDisc.open("wavDisc1.txt");
    for(auto& tuple: waveCSpace1.discCoordList) {
        myWavDisc << get<0>(tuple) << " " << get<1>(tuple) << endl;
    }
    myWavDisc.close();

    WavefrontPlanner wavPlan1((CSpace2D &) waveCSpace1, qstart1,qgoal1);

    ofstream myWaveFile;
    myWaveFile.open("waveGrid1.txt");
    for(int i = 0; i <= waveCSpace1.resolution-1; i ++){
        for(int j = 0; j <= waveCSpace1.resolution-1; j++) {
            myWaveFile<< j << " " << i << " " <<  wavPlan1.waveGridList(j,i) << endl;
        }
    }
    myWaveFile.close();
    wavPlan1.getWavePath();
    ofstream myWavePathFile;
    myWavePathFile.open("wavePath1.txt");
    for(auto& tuple: wavPlan1.wavePath) {
        myWavePathFile << get<0>(tuple) << " " << get<1>(tuple) << endl;
    }
    myWavePathFile.close();
    ////////////////////////////////////////////////BEGIN SECOND WAVEPLANNER SCENARIO///////////////////////////////////
    cout << "==========BEGINNING SECOND WAVEPLANNER SCENARIO==============="<<endl;
    tuple<int,int> xBounds25(-10,35);
    tuple<int,int> yBounds25(-7,7);

    CSpace2D waveCSpace2(dx, obsList2,xBounds25,yBounds25,qgoal2,qstart2);
    ofstream myWavDisc2;
    myWavDisc2.open("wavDisc2.txt");
    for(auto& tuple: waveCSpace2.discCoordList) {
        myWavDisc2 << get<0>(tuple) << " " << get<1>(tuple) << endl;
    }
    myWavDisc2.close();

    WavefrontPlanner wavPlan2((CSpace2D &) waveCSpace2, qstart2,qgoal2);

    ofstream myWaveFile2;
    myWaveFile2.open("waveGrid2.txt");
    for(int i = 0; i <= waveCSpace2.yres-1; i ++){
        for(int j = 0; j <= waveCSpace2.xres-1; j++) {
            myWaveFile2 << j << " " << i << " " <<  wavPlan2.waveGridList(j,i) << endl;
        }
    }
    myWaveFile2.close();
    wavPlan2.getWavePath();
    ofstream myWavePathFile2;
    myWavePathFile2.open("wavePath2.txt");
    for(auto& tuple: wavPlan2.wavePath) {
        myWavePathFile2 << get<0>(tuple) << " " << get<1>(tuple) << endl;
    }
    myWavePathFile2.close();

    //////////////////////////////////////BEGIN PROBLEM 4///////////////////////////////////////////////////////////////
    cout << "==========BEGINNING THIRD WAVEPLANNER SCENARIO==============="<<endl;
    // Notice that the start in cspace is (0,0)
    // The goal in cspace is (pi,0)
    vector<vector<tuple<double, double>>> obsListc;
    // Create the vectors for each obstacle and fill them with the tuples of coordinates
    vector<tuple<double,double>> obsCoords1c;
    obsCoords1c.emplace_back(-0.25,1.1);
    obsCoords1c.emplace_back(-0.25,2);
    obsCoords1c.emplace_back(0.25,2);
    obsCoords1c.emplace_back(0.25,1.1);

    vector<tuple<double,double>> obsCoords2c;
    obsCoords2c.emplace_back(-2,-0.5);
    obsCoords2c.emplace_back(-2,-0.3);
    obsCoords2c.emplace_back(2,-0.3);
    obsCoords2c.emplace_back(2,-0.5);

    obsListc.push_back(obsCoords1c);
    obsListc.push_back(obsCoords2c);
    //Initialize 2 link class
    TwoLink myManipc(1,1,1);
    //Initialize 2 link manipulator c space class
    TwoDManipulatorCSpace myManipCSpacec(obsListc,(TwoLink &) myManipc);
//    myManipCSpacec.discretizeCSpace();
    myManipCSpacec.createBrushGrid();

    ofstream myWavDisc3;
    myWavDisc3.open("wavDisc3.txt");
    for(auto& tuple: myManipCSpacec.discCoordList) {
        myWavDisc3 << get<0>(tuple) << " " << get<1>(tuple) << endl;
    }
    myWavDisc3.close();

    ofstream myWaveFile3;
    myWaveFile3.open("waveGrid3.txt");
    for(int i = 0; i <=myManipCSpacec.resolution-1; i ++){
        for(int j = 0; j <=myManipCSpacec.resolution-1; j++) {
            myWaveFile3 << j << " " << i << " " <<  myManipCSpacec.brushDistList(j,i) << endl;
        }
    }
    myWaveFile3.close();

    ofstream myWavePathFile3;
    myWavePathFile3.open("wavePath3.txt");
    for(auto& tuple: myManipCSpacec.wavePath) {
        myWavePathFile3 << get<0>(tuple) << " " << get<1>(tuple) << endl;
    }
    myWavePathFile3.close();

    ofstream myWorkPathFile;
    myWorkPathFile.open("workPath1.txt");
    for(auto links : myManipCSpacec.workPath){
        for(auto tups : links){
            myWorkPathFile << get<0>(tups) << " " << get<1>(tups) << endl;
        }
    }
    myWorkPathFile.close();

    return 0;
}

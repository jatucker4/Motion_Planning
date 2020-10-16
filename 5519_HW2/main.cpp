#include <iostream>
#include <fstream>
#include "ThreeLink.h"
#include "TwoLink.h"
#include "TwoDManipulatorCspace.h"
// In this assignment I use the Eigen package

int main() {
    // Define the angles for question 7 part 1
    double theta1 = 0;
    double theta2 = M_PI/4;
    double theta3 = M_PI/4;
    // Create a three link manipulator class
    ThreeLink manipulator1(theta1,theta2,theta3,10,10,10);
    // Get the list of link origins
    vector<tuple<double,double>> originList = manipulator1.forwardKinematics();
    // Get the end effector location
    tuple<double,double> endLoc = manipulator1.getEndEffectorLoc();
    originList.push_back(endLoc);
    // Print link and end effector locations to a file for MATLAB
    ofstream myfile;
    myfile.open("Kinematics_1_LinkLocs.txt");

    for(auto& tuple: originList) {
        myfile << get<0>(tuple) << " " << get<1>(tuple) << endl;
    }
    myfile.close();


    //Begin code block for question 7 part 2
    double length = 10;
    tuple<double,double> effectorLoc(10,20);
    //Create a 3 link manipulator class using the overloaded constructor
    ThreeLink manipulator2(length,10,10, effectorLoc);
    vector<double> angList;
    //Perform inverse kinematics
    manipulator2.inverseKinematics();
    //Get a list of angles using the class variables
    angList.push_back(manipulator2.thetaOne);
    angList.push_back(manipulator2.thetaTwo);
    angList.push_back(manipulator2.thetaThree);

    vector<tuple<double,double>> originList2 = manipulator2.forwardKinematics();
    tuple<double,double> endLoc2 = manipulator2.getEndEffectorLoc();
    originList2.push_back(endLoc2);
    //Print the link origins and angles to two separate files for MATLAB
    ofstream thisfile;
    thisfile.open("Kinematics_2_LinkLocs.txt");

    for(double & i : angList) {
        thisfile << i << endl;
    }
    thisfile.close();

    ofstream thisfile2;
    thisfile2.open("Kinematics_22_LinkLocs.txt");
    for(auto& tuple: originList2) {
        thisfile2 << get<0>(tuple) << " " << get<1>(tuple) << endl;
    }
    thisfile2.close();

    cout << "BEGINNING QUESTION 8 PART A " << "\n";
    // Begin the Problem 8 section
    //Part a with the triangle obstacle
    // Create a vector of obstacles for the first workspace
    vector<vector<tuple<double, double>>> obsList;
    // Create the vectors for each obstacle and fill them with the tuples of coordinates
    vector<tuple<double,double>> obsCoords1;

    obsCoords1.emplace_back(-0.25,0.25);
    obsCoords1.emplace_back(0,0.75);
    obsCoords1.emplace_back(0.25,0.25);

    obsList.push_back(obsCoords1);

    TwoLink myManip(1,1,1);
    TwoDManipulatorCSpace myManipCSpace(obsList,(TwoLink &) myManip);
    myManipCSpace.discretizeCSpace();
    myManipCSpace.getCollisionBools();
    vector<double> myBoolList = myManipCSpace.collBools;
    // Now print the coords and corresponding bool to a file
    ofstream myFile3;
    myFile3.open("CSpace_Part_a.txt");
    for(auto& tuple: myManipCSpace.discCoordList) {
        myFile3 << get<0>(tuple) << " " << get<1>(tuple) << endl;
    }
    for(auto colbool : myBoolList){
        myFile3 << colbool << endl;
    }
    myFile3.close();

    // Q8 Part b
    cout << "BEGINNING QUESTION 8 PART B " << "\n";
    vector<vector<tuple<double, double>>> obsListb;
    // Create the vectors for each obstacle and fill them with the tuples of coordinates
    vector<tuple<double,double>> obsCoords1b;
    obsCoords1b.emplace_back(-0.25,1.1);
    obsCoords1b.emplace_back(-0.25,2);
    obsCoords1b.emplace_back(0.25,2);
    obsCoords1b.emplace_back(0.25,1.1);

    vector<tuple<double,double>> obsCoords2b;
    obsCoords2b.emplace_back(-2,-2);
    obsCoords2b.emplace_back(-2,-1.8);
    obsCoords2b.emplace_back(2,-1.8);
    obsCoords2b.emplace_back(2,-2);

    obsListb.push_back(obsCoords1b);
    obsListb.push_back(obsCoords2b);
    // Create two link manipulator
    TwoLink myManipb(1,1,1);
    // Create 2 link C-space
    TwoDManipulatorCSpace myManipCSpaceb(obsListb,(TwoLink &) myManipb);
    myManipCSpaceb.discretizeCSpace();
    myManipCSpaceb.getCollisionBools();
    vector<double> myBoolListb = myManipCSpaceb.collBools;
    // Now print the coords and corresponding bool to a file
    ofstream myFile3b;
    myFile3b.open("CSpace_Part_b.txt");
    for(auto& tuple: myManipCSpaceb.discCoordList) {
        myFile3b << get<0>(tuple) << " " << get<1>(tuple) << endl;
    }
    for(auto colbool : myBoolListb){
        myFile3b << colbool << endl;
    }
    myFile3b.close();

    // Q8 Part c
    cout << "BEGINNING QUESTION 8 PART C " << "\n";
    vector<vector<tuple<double, double>>> obsListc;
    // Create the vectors for each obstacle and fill them with the tuples of coordinates
    vector<tuple<double,double>> obsCoords1c;
    obsCoords1c.emplace_back(-0.25,0.25);
    obsCoords1c.emplace_back(0,0.75);
    obsCoords1c.emplace_back(0.25,0.25);

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
    myManipCSpacec.discretizeCSpace();
    myManipCSpacec.getCollisionBools();
    vector<double> myBoolListc = myManipCSpacec.collBools;
    // Now print the coords and corresponding bool to a file
    ofstream myFile3c;
    myFile3c.open("CSpace_Part_c.txt");
    for(auto& tuple: myManipCSpacec.discCoordList) {
        myFile3c << get<0>(tuple) << " " << get<1>(tuple) << endl;
    }
    for(auto colbool : myBoolListc){
        myFile3c << colbool << endl;
    }
    myFile3c.close();
    return 0;
}

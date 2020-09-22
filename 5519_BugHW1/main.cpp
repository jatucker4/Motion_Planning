#include <iostream>
#include "Bug.h"
#include <fstream>
int main() {
    // Create a vector of obstacles for the first workspace
    vector<vector<tuple<double, double>>> obsList;
    // Create the vectors for each obstacle and fill them with the tuples of coordinates
    vector<tuple<double,double>> obsCoords1;

    obsCoords1.push_back(tuple<double,double>(1,1));
    obsCoords1.push_back(tuple<double,double>(2,1));
    obsCoords1.push_back(tuple<double,double>(2,5));
    obsCoords1.push_back(tuple<double,double>(1,5));

    vector<tuple<double,double>> obsCoords2;
    obsCoords2.push_back(tuple<double,double>(3,4));
    obsCoords2.push_back(tuple<double,double>(4,4));
    obsCoords2.push_back(tuple<double,double>(4,12));
    obsCoords2.push_back(tuple<double,double>(3,12));

    vector<tuple<double,double>> obsCoords3;
    obsCoords3.push_back(tuple<double,double>(3,12));
    obsCoords3.push_back(tuple<double,double>(12,12));
    obsCoords3.push_back(tuple<double,double>(12,13));
    obsCoords3.push_back(tuple<double,double>(3,13));

    vector<tuple<double,double>> obsCoords4;
    obsCoords4.push_back(tuple<double,double>(12,5));
    obsCoords4.push_back(tuple<double,double>(13,5));
    obsCoords4.push_back(tuple<double,double>(13,13));
    obsCoords4.push_back(tuple<double,double>(12,13));

    vector<tuple<double,double>> obsCoords5;
    obsCoords5.push_back(tuple<double,double>(6,5));
    obsCoords5.push_back(tuple<double,double>(12,5));
    obsCoords5.push_back(tuple<double,double>(12,6));
    obsCoords5.push_back(tuple<double,double>(6,6));

    obsList.push_back(obsCoords1);
    obsList.push_back(obsCoords2);
    obsList.push_back(obsCoords3);
    obsList.push_back(obsCoords4);
    obsList.push_back(obsCoords5);

    // Instantiate workspace object with the obstacle list
    Workspace work(obsList);

    // Creaete necessary variables for the BugBot Constructor
    string bug = "bug1";
    string bug2 = "bug2";
    tuple<double,double> qStart(0,0);
    tuple<double,double> qGoal(10,10);
    // Instantiate bugs one and two with the first workspace
    Bug myBug(bug, (Workspace &) work, qStart, qGoal);
    Bug myBug2(bug2,(Workspace &) work, qStart, qGoal);

    // Display the results to the console
    double dist = myBug.sendToGoal();
    cout << "Bug 1 in workspace 1 in has successfully reached the goal! " << "\n";
    cout << "The total distance traveled was: " << dist << "\n";
    cout << "======================================================================" << "\n";

    double dist2 = myBug2.sendToGoal();
    cout << "Bug 2 in workspace 1 in has successfully reached the goal! " << "\n";
    cout << "The total distance traveled was: " << dist2 << "\n";
    cout << "======================================================================" << "\n";

    // Create vectors for the path of the bugs
    vector<tuple<double,double>> myPath = myBug.bugPath;
    vector<tuple<double,double>> myPath2 = myBug2.bugPath;
    // Write the paths to a file for MATLAB
    ofstream myfile;
    myfile.open("Bug1PathFile.txt");

    for(auto& tuple: myPath) {
        myfile << get<0>(tuple) << " " << get<1>(tuple) << endl;
    }
    myfile.close();

    ofstream myfile2;
    myfile2.open("Bug2PathFile.txt");

    for(auto& tuple: myPath2) {
        myfile2 << get<0>(tuple) << " " << get<1>(tuple) << endl;
    }
    myfile2.close();


    // Begin scenario two section
    vector<vector<tuple<double, double>>> obsList2;
    vector<tuple<double,double>> obsCoords10;

    obsCoords10.push_back(tuple<double,double>(-6,-6));
    obsCoords10.push_back(tuple<double,double>(25,-6));
    obsCoords10.push_back(tuple<double,double>(25,-5));
    obsCoords10.push_back(tuple<double,double>(-6,-5));

    vector<tuple<double,double>> obsCoords20;
    obsCoords20.push_back(tuple<double,double>(-6,5));
    obsCoords20.push_back(tuple<double,double>(30,5));
    obsCoords20.push_back(tuple<double,double>(30,6));
    obsCoords20.push_back(tuple<double,double>(-6,6));

    vector<tuple<double,double>> obsCoords30;
    obsCoords30.push_back(tuple<double,double>(-6,-5));
    obsCoords30.push_back(tuple<double,double>(-5,-5));
    obsCoords30.push_back(tuple<double,double>(-5,5));
    obsCoords30.push_back(tuple<double,double>(-6,5));

    vector<tuple<double,double>> obsCoords40;
    obsCoords40.push_back(tuple<double,double>(4,-5));
    obsCoords40.push_back(tuple<double,double>(5,-5));
    obsCoords40.push_back(tuple<double,double>(5,1));
    obsCoords40.push_back(tuple<double,double>(4,1));

    vector<tuple<double,double>> obsCoords50;
    obsCoords50.push_back(tuple<double,double>(9,0));
    obsCoords50.push_back(tuple<double,double>(10,0));
    obsCoords50.push_back(tuple<double,double>(10,5));
    obsCoords50.push_back(tuple<double,double>(9,5));

    vector<tuple<double,double>> obsCoords60;
    obsCoords60.push_back(tuple<double,double>(14,-5));
    obsCoords60.push_back(tuple<double,double>(15,-5));
    obsCoords60.push_back(tuple<double,double>(15,1));
    obsCoords60.push_back(tuple<double,double>(14,1));

    vector<tuple<double,double>> obsCoords70;
    obsCoords70.push_back(tuple<double,double>(19,0));
    obsCoords70.push_back(tuple<double,double>(20,0));
    obsCoords70.push_back(tuple<double,double>(20,5));
    obsCoords70.push_back(tuple<double,double>(19,5));

    vector<tuple<double,double>> obsCoords80;
    obsCoords80.push_back(tuple<double,double>(24,-5));
    obsCoords80.push_back(tuple<double,double>(25,-5));
    obsCoords80.push_back(tuple<double,double>(25,1));
    obsCoords80.push_back(tuple<double,double>(24,1));

    vector<tuple<double,double>> obsCoords90;
    obsCoords90.push_back(tuple<double,double>(29,0));
    obsCoords90.push_back(tuple<double,double>(30,0));
    obsCoords90.push_back(tuple<double,double>(30,5));
    obsCoords90.push_back(tuple<double,double>(29,5));

    obsList2.push_back(obsCoords10);
    obsList2.push_back(obsCoords20);
    obsList2.push_back(obsCoords30);
    obsList2.push_back(obsCoords40);
    obsList2.push_back(obsCoords50);
    obsList2.push_back(obsCoords60);
    obsList2.push_back(obsCoords70);
    obsList2.push_back(obsCoords80);
    obsList2.push_back(obsCoords90);

    // Instantiate second workspace object
    Workspace work2(obsList2);

    tuple<double,double> qStart2(0,0);
    tuple<double,double> qGoal2(35,0);
    // Instantiate bugs 1 and 2 for the second workspace
    Bug myBug3(bug, (Workspace &) work2, qStart2, qGoal2);
    Bug myBug4(bug2,(Workspace &) work2, qStart2, qGoal2);

    // Display results in the console
    double dist3 = myBug3.sendToGoal();
    cout << "Bug 1 in workspace 2 in has successfully reached the goal! " << "\n";
    cout << "The total distance traveled was: " << dist3 << "\n";
    cout << "======================================================================" << "\n";

    double dist4 = myBug4.sendToGoal();
    cout << "Bug 2 in workspace 2 in has successfully reached the goal! " << "\n";
    cout << "The total distance traveled was: " << dist4 << "\n";
    cout << "======================================================================" << "\n";

    // Write the path to files for MATLAB
    vector<tuple<double,double>> myPath3 = myBug3.bugPath;
    vector<tuple<double,double>> myPath4 = myBug4.bugPath;

    ofstream myfile3;
    myfile3.open("Bug1Path2File.txt");

    for(auto& tuple: myPath3) {
        myfile3 << get<0>(tuple) << " " << get<1>(tuple) << endl;
    }
    myfile3.close();

    ofstream myfile4;
    myfile4.open("Bug2Path2File.txt");

    for(auto& tuple: myPath4) {
        myfile4 << get<0>(tuple) << " " << get<1>(tuple) << endl;
    }
    myfile4.close();

    return 0;
}


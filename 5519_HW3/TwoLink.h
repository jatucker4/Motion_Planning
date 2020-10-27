//
// Created by John on 10/3/2020.
//

#ifndef INC_5519_HW2_TWOLINK_H
#define INC_5519_HW2_TWOLINK_H


#include <string>
#include <cmath>
#include <tuple>
#include <vector>
#include <iostream>
#include "Eigen/Core"
#include "Eigen/Geometry"

using namespace std;
using namespace Eigen;
// Simple class for the two link manipulator
class TwoLink {
public:
    TwoLink( double length,double length2,double length3);

    double jointToEnd;
    double distBetwJoint;


};

// This is the constructor for the TwoLink class
TwoLink::TwoLink( double length,double length2,double length3) {
// I'll need to reassign the input variables to the class ones
    jointToEnd = 0;
    distBetwJoint = length - (2 * jointToEnd);
}
#endif //INC_5519_HW2_TWOLINK_H

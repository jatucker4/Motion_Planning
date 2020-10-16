//
// Created by John on 9/27/2020.
//

#ifndef INC_5519_HW2_THREELINK_H
#define INC_5519_HW2_THREELINK_H
#include <string>
#include <cmath>
#include <tuple>
#include <vector>
#include <iostream>
#include "Eigen/Core"
#include "Eigen/Geometry"

using namespace std;
using namespace Eigen;
// Class for the three link manipulator
class ThreeLink {
public:
    ThreeLink(double theta1, double theta2, double theta3, double length,double length2,double length3);
    ThreeLink(double length,double length2,double length3, tuple<double,double> effectorLoc);
    tuple<double,double> getEndEffectorLoc();
    vector<tuple<double,double>> forwardKinematics();
    void inverseKinematics();
    void calcEndEffectPos(Eigen::Vector3d& guessing, Eigen::Vector3d& endloc);
    void calcJointPos(Eigen::Vector3d& guessing, Eigen::Vector3d& j1, Eigen::Vector3d& j2, Eigen::Vector3d& j3);

    double thetaOne;
    double thetaTwo;
    double thetaThree;
    double jointToEnd;
    double distBetwJoint;
    tuple<double,double> endEffector;

};

// This is the constructor for the Threelink class
ThreeLink::ThreeLink(double theta1, double theta2, double theta3, double length,double length2,double length3) {
// I'll need to reassign the input variables to the class ones
thetaOne = theta1;
thetaTwo = theta2;
thetaThree = theta3;
jointToEnd = 0;
distBetwJoint = length - (2 * jointToEnd);
}

// This is our overloaded constructor for the scenario where only a length and end effector location are provided
ThreeLink::ThreeLink(double length,double length2,double length3, tuple<double, double> effectorLoc) {
// I'll need to reassign the input variables to the class ones
jointToEnd = 0;
distBetwJoint = length - (2 * jointToEnd);
endEffector = effectorLoc;
}

// Simple getter method to obtain the end effector location
tuple<double, double> ThreeLink::getEndEffectorLoc() {
    return endEffector;
}

// This method handles the forward kinematic scenario
vector<tuple<double, double>> ThreeLink::forwardKinematics() {
    // First I need to create the transformation matrix object for each link
    Matrix<double, 3, 3> T1;
    Matrix<double, 3, 3> T2;
    Matrix<double, 3, 3> T3;
    Matrix<double, 3, 3> T4;
    // Now create and initialize the vector to determine the xy of the origin of each links local frame
    Vector3d L1;
    L1 << 0,0,1;
    // Now initialize each matrix
    T1 << cos(thetaOne), -sin(thetaOne), 0,
        sin(thetaOne), cos(thetaOne), 0,
        0, 0, 1;
    T2 << cos(thetaTwo), -sin(thetaTwo), distBetwJoint,
            sin(thetaTwo), cos(thetaTwo), 0,
            0, 0, 1;
    T3 << cos(thetaThree), -sin(thetaThree), distBetwJoint,
            sin(thetaThree), cos(thetaThree), 0,
            0, 0, 1;
    T4 << 1, 0, distBetwJoint,
            0, 1, 0,
            0, 0, 1;
    // Now we can determine the origin of each links local frame by multiplying the T matrices by the L1 vector
    Matrix<double, 3,1> L1Global = T1 * L1;
    Matrix<double, 3,1> L2Global = (T1 * T2) * L1;
    Matrix<double, 3,1> L3Global = (T1 * T2 * T3) * L1;
    Matrix<double, 3,1> EEFGlobal =(T1 * T2 * T3 * T4) * L1;
    // Create the tuples for each link origin
    double EFX = EEFGlobal[0];
    double EFY = EEFGlobal[1];
    endEffector = make_tuple(EFX, EFY);
    tuple<double, double> L1Origin (L1Global[0],L1Global[1]);
    tuple<double, double> L2Origin (L2Global[0],L2Global[1]);
    tuple<double, double> L3Origin (L3Global[0],L3Global[1]);
    vector<tuple<double, double>> originList;
    originList.push_back(L1Origin);
    originList.push_back(L2Origin);
    originList.push_back(L3Origin);
    // Return the vector of tuples with link origins
    return originList;
}

void ThreeLink::inverseKinematics() {
    // Iterate on the initial guess using the Jacobian to minimize error between the
    // position of the end effector derived from the guess and the actual position
    // Our initial guess for the angles
    double step = 0.000001;
    Vector3d guess;
    guess << 0, M_PI/4, M_PI/4;
    Vector3d endTrue;
    endTrue << get<0>(endEffector),get<1>(endEffector),0;
    Vector3d endEffectorPosition;
    endEffectorPosition << 0, 0, 0;
    Vector3d RotAxis;
    RotAxis << 0,0,1;

    Vector3d joint1;
    joint1 << 0, 0, 0;
    Vector3d joint2;
    joint2 << 0, 0, 0;
    Vector3d joint3;
    joint3 << 0, 0, 0;

    Vector3d J_1;
    J_1 << 0, 0, 0;
    Vector3d J_2;
    J_2 << 0, 0, 0;
    Vector3d J_3;
    J_3 << 0, 0, 0;

    Matrix<double, 3,3> Jacobian;
    Jacobian << 0,0,0,
                0,0,0,
                0,0,0;

    Vector3d delta;
    delta << 0, 0, 0;

    double error = 0.0000000001;
    calcEndEffectPos(guess, endEffectorPosition);

        while( (endEffectorPosition - endTrue).norm() > error ) {
            // Get the Jacobian
            calcJointPos(guess, joint1, joint2, joint3);

            J_1 = RotAxis.cross(endEffectorPosition - joint1);
            J_2 = RotAxis.cross(endEffectorPosition - joint2);
            J_3 = RotAxis.cross(endEffectorPosition - joint3);

            Jacobian << J_1, J_2, J_3;
            Jacobian.transpose();

            delta = Jacobian.transpose() * (endTrue - endEffectorPosition);

            guess += (delta * step); // T=O+dO*h
            calcEndEffectPos(guess, endEffectorPosition);
        }
    thetaOne = guess(0);
    thetaTwo = guess(1);
    thetaThree = guess(2);
}


void ThreeLink::calcEndEffectPos(Vector3d &guessing, Vector3d &endloc) {
    Matrix<double, 3, 3> T1;
    Matrix<double, 3, 3> T2;
    Matrix<double, 3, 3> T3;
    Matrix<double, 3, 3> T4;
    double theta1 = guessing(0);
    double theta2 = guessing(1);
    double theta3 = guessing(2);
    // Now create and initialize the vector to determine the xy of the origin of each links local frame
    Vector3d L1;
    L1 << 0,0,1;
    // Now initialize each matrix
    T1 << cos(theta1), -sin(theta1), 0,
            sin(theta1), cos(theta1), 0,
            0, 0, 1;
    T2 << cos(theta2), -sin(theta2), distBetwJoint,
            sin(theta2), cos(theta2), 0,
            0, 0, 1;
    T3 << cos(theta3), -sin(theta3), distBetwJoint,
            sin(theta3), cos(theta3), 0,
            0, 0, 1;
    T4 << 1, 0, distBetwJoint,
            0, 1, 0,
            0, 0, 1;
    // Now we can determine the origin of each links local frame by multiplying the T matrices by the L1 vector
    endloc =(T1 * T2 * T3 * T4) * L1;
    endloc(2) = 0;
}

void ThreeLink::calcJointPos(Vector3d &guessing, Vector3d &j1, Vector3d &j2, Vector3d &j3) {
    // First I need to create the transformation matrix object for each link
    Matrix<double, 3, 3> T1;
    Matrix<double, 3, 3> T2;
    Matrix<double, 3, 3> T3;
    Matrix<double, 3, 3> T4;
    double theta1 = guessing(0);
    double theta2 = guessing(1);
    double theta3 = guessing(2);
    // Now create and initialize the vector to determine the xy of the origin of each links local frame
    Vector3d L1;
    L1 << 0, 0,1;

    // Now initialize each matrix
    T1 << cos(theta1), -sin(theta1), 0,
            sin(theta1), cos(theta1), 0,
            0 , 0, 1;
    T2 << cos(theta2), -sin(theta2), distBetwJoint,
            sin(theta2), cos(theta2), 0,
            0, 0, 1;
    T3 << cos(theta3), -sin(theta3), distBetwJoint,
            sin(theta3), cos(theta3), 0,
            0, 0, 1;
    T4 << 1, 0, distBetwJoint,
            0, 1, 0,
            0, 0, 1;
    // Now we can determine the origin of each links local frame by multiplying the T matrices by the L1 vector
    j1 = T1 * L1;
    j1(2) = 0;

    j2 = (T1 * T2) * L1;
    j2(2) = 0;

    j3 = (T1 * T2 * T3) * L1;
    j3(2) = 0;
}


#endif //INC_5519_HW2_THREELINK_H

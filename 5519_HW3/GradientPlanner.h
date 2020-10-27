//
// Created by John on 10/16/2020.
//

#ifndef INC_5519_HW3_GRADIENTPLANNER_H
#define INC_5519_HW3_GRADIENTPLANNER_H

#include "CSpace2D.h"
class GradientPlanner {
public:
    GradientPlanner(CSpace2D &myspace,vector<double> QStar, double DStar, tuple<double,double> qStart, tuple<double,double> qGoal, double attgain, double repgain);
    tuple<double,double> attPotential();
    tuple<double,double> repPotential();
    vector<tuple<double,double>> getPotentialField();
    int getIndex(double xVal, double yVal);
    static tuple<double,double> vectorDiff(tuple<double,double>vec1, tuple<double,double>vec2);
    static double vectorMag(tuple<double,double> vec);
    double distance(tuple<double, double> target);
//    double interpolateGrad(double xVal, double yVal);
    void gradientDescent();
    vector<double> getGradient(vector<double> potentialField);
    tuple<double,double> tupSum(tuple<double,double> tupOne, tuple<double,double> tupTwo);
    bool nearGoal();
    //Class variables
    double dstar;
    vector<double> Qstar;
    CSpace2D *mySpace;
    vector<tuple<double,double>> pathList;
    tuple<double,double> currentLoc;
    tuple<double, double> qgoal;
    tuple<double, double> qstart;
    double attGain;
    double repGain;
    double goalRegion;
    vector<tuple<double,double>> potField;
    vector<tuple<double,double>> cVec;
};
GradientPlanner::GradientPlanner(CSpace2D &myspace, vector<double> QStar, double DStar, tuple<double, double> qStart,
                                 tuple<double, double> qGoal, double attgain, double repgain) {
    qgoal = qGoal;
    qstart = qStart;
    dstar = DStar;
    Qstar = QStar;
    mySpace = &myspace;
    attGain = attgain;
    repGain = repgain;
    goalRegion = 0.25;

//    potField = getPotentialField();
}

double GradientPlanner::distance(tuple<double, double> target) {
    tuple<double,double> vecDiff = vectorDiff(target, currentLoc);
    double distToLocation = vectorMag(vecDiff);

    return distToLocation;
}

tuple<double, double> GradientPlanner::vectorDiff(tuple<double, double> vec1, tuple<double, double> vec2) {
    double diffX = get<0>(vec2) - get<0>(vec1);
    double diffY = get<1>(vec2) - get<1>(vec1);
    tuple<double, double> vecDiff(diffX, diffY);

    return vecDiff;
}

double GradientPlanner::vectorMag(tuple<double, double> vec) {
    double x = get<0>(vec);
    double y = get<1>(vec);
    double exponent = 2.0;
    double magnitude = sqrt( pow(x, exponent) + pow(y,exponent));
    return magnitude;
}
tuple<double,double> GradientPlanner::attPotential() {
    // INITIAL IMPLEMENTATION
//    double distToGoal = distance(qgoal);
//    double Uatt;
//    if(distToGoal < dstar){
//        Uatt = 0.5 * attGain * (distToGoal * distToGoal);
//    }else{
//        Uatt = (dstar * attGain * distToGoal) - 0.5 * (attGain * distToGoal * distToGoal);
//    }
//    return Uatt;

    tuple<double,double> Uatt;
    double Uattx;
    double Uatty;
    double distToGoal = distance(qgoal);
    if(distToGoal <= dstar){
        Uattx = attGain * (get<0>(currentLoc) - get<0>(qgoal));
        Uatty = attGain * (get<1>(currentLoc) - get<1>(qgoal));
    }else{
        Uattx = (dstar * attGain * (get<0>(currentLoc) - get<0>(qgoal))) /
                distToGoal;
        Uatty = (dstar * attGain * (get<1>(currentLoc) - get<1>(qgoal))) /
                distToGoal;
    }
    Uatt = tuple<double,double>(Uattx,Uatty);
    return Uatt;
}

tuple<double,double> GradientPlanner::repPotential() {
    //Initial implementation
//    double Urep = 0;
//    double tempVar;
//    vector<double> distToEachObst = mySpace->distToObst(currentLoc);
//    for(int iter = 0; iter<= Qstar.size(); iter++){
//        if(distToEachObst[iter] <= Qstar[iter]){
//            tempVar = 0.5 * repGain * (( (1 / distToEachObst[iter]) - (1 / Qstar[iter]) ) *
//                    ( (1 / distToEachObst[iter]) - (1 / Qstar[iter]) ));
//        }else{
//            tempVar = 0;
//        }
//        Urep = Urep + tempVar;
//    }
//    return Urep;
    tuple<double,double> Urep;
    double Urepx;
    double Urepy;
    double tempvar;
    double distToObst;
    double Urepxsum = 0;
    double Urepysum = 0;
    for(int iter = 0; iter<= Qstar.size()-1; iter++){
        distToObst = distance(cVec[iter]);
        tempvar = repGain * ((  (1 / Qstar[iter]) - (1 / distToObst) )) /
                    ( distToObst * distToObst * distToObst);
        if(distToObst <= Qstar[iter]){
            Urepx = tempvar * (get<0>(currentLoc) - get<0>(cVec[iter]));
            Urepy = tempvar * (get<1>(currentLoc) - get<1>(cVec[iter]));
        }else{
            Urepx = 0;
            Urepy = 0;
        }
        Urepxsum = Urepxsum + Urepx;
        Urepysum = Urepysum + Urepy;
    }
    Urep = tuple<double,double>(Urepxsum,Urepysum);
    return Urep;
}

vector<tuple<double,double>> GradientPlanner::getPotentialField() {
    // Need to iterate through every grid point and calculate the potential
    vector<tuple<double,double>> grid = mySpace->discCoordList;
    vector<tuple<double,double>> potential;
    tuple<double,double> repPot;
    tuple<double,double> attPot;
    tuple<double,double> totalPot;
    for(auto& iter : grid){
        currentLoc = iter;
        repPot = repPotential();
        attPot = attPotential();
        totalPot = tupSum(repPot,attPot);

        potential.push_back(totalPot);
    }
    return potential;
}

void GradientPlanner::gradientDescent() {
    double alpha = 0.00001;
    pathList.push_back(qstart);
    double xCurr = get<0>(qstart);
    double yCurr = get<1>(qstart);
    double xNext;
    double yNext;
    double gradX;
    double gradY;
    int index;
    tuple<double,double> gradVals;
    tuple<double,double> gradAtt;
    tuple<double,double> gradRep;
//    vector<double> gradVals = getGradient(potentialField);
    currentLoc = qstart;
    double gradVal;
    while(!nearGoal()){
        // Calculate the gradient from the potential vector
//        gradRep = repPotential();
//        gradAtt = attPotential();
//        index = getIndex(xCurr,yCurr);
//        gradVals = tupleSum(gradRep,gradAtt);
        gradVals = tupSum(repPotential(),attPotential());
        gradX = get<0>(gradVals);
        gradY = get<1>(gradVals);
        xNext = xCurr - alpha * gradX;
        yNext = yCurr - alpha * gradY;

        pathList.emplace_back(xNext,yNext);
        xCurr = xNext;
        yCurr = yNext;

        currentLoc = tuple<double,double>(xCurr,yCurr);
//        cout << "(" << get<0>(currentLoc) << "," << get<1>(currentLoc) << ")" << endl;
    }
}

bool GradientPlanner::nearGoal() {
    double dist = distance(qgoal);
    return (dist <= goalRegion);
}

vector<double> GradientPlanner::getGradient(vector<double> potentialField) {
    vector<double> gradientVals;
    gradientVals.push_back(potentialField[1] - potentialField[0]);
    for(int i = 1; i<= (potentialField.size() - 1); i++){
        double temp;
        temp = (potentialField[i+1] - potentialField[i-1]) / 2;
        gradientVals.push_back(temp);
    }
    gradientVals.push_back(potentialField.end()[-1] - potentialField.end()[-2]);
    return gradientVals;
}

int GradientPlanner::getIndex(double xVal, double yVal) {
    int index = 0;
    double error = 0.03;
    vector<tuple<double,double>> discList = mySpace->discCoordList;
    reverse(discList.begin(),discList.end());
    for(auto& iter : discList){
        if((abs( xVal - get<0>(iter)) <= error) && (abs(  yVal - get<1>(iter)) <= error)){
            break;
        }
        index++;
    }
    return (discList.size() - index - 1);
}


tuple<double, double> GradientPlanner::tupSum(tuple<double, double> tupOne, tuple<double, double> tupTwo) {
    tuple<double,double> tupSumOne;
    double x1 = get<0>(tupOne);
    double x2 = get<0>(tupTwo);
    double y1 = get<1>(tupOne);
    double y2 = get<1>(tupTwo);
    double xsum = x1 + x2;
    double ysum = y1 + y2;

    tupSumOne = tuple<double,double>(xsum,ysum);

    return tupSumOne;

}

#endif //INC_5519_HW3_GRADIENTPLANNER_H

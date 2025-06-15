#pragma once
#include "Quaternion.h"
#include "Constants.h"
#include "VectorMath.h"


#include <vector>
#include <string>
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>


using std::vector;
using std::string;

class Frame {
public:
    vector<double> _anglesWrtInertial;
    vector<vector<double>> _IB;
    double _dt;
    vector<vector<double>> _xB;
    vector<vector<double>> _xI;
    string _name;
    vector<vector<vector<double>>> _stateHistInI;
    vector<vector<vector<double>>> _stateHistInB;
    vector<vector<double>> _anglesWrtInertialHist;
    vector<double> _qIB;
    vector<double> _qBI;
    vector<vector<double>> _qIBhist;
    vector<vector<double>> _xI0;

    Frame(const vector<vector<double>>& x0,
          const vector<double>& anglesWrtInertial,
          double dt,
          const string& name = "");
    virtual void update(const vector<double>& accelInBody,
                        const vector<double>& anglesLosWrtI,
                        const vector<double>& angleRatesWrtLos) 
                {
                    
                }

    void updateStates(const vector<double>& accelInBody);

    void storeStatesInEnglishUnits(const vector<double>& r,
                                   const vector<double>& v,
                                   const vector<double>& a,
                                   const string& frame);

    const vector<double>& getAnglesWrtInertial() const ;
    vector<vector<double>> getInertialStates() const ;
    vector<vector<double>> getBodyStates() const ;
    //static json parseConfig(const string& cfgFile) ;

    vector<vector<double>> getStates() const ;

    void printAngles() const ;

    void printStates() const ;
    static vector<double> clipTheta(const vector<double>& thetaVec);

    const vector<double>& getqIB() const;
    const vector<double>& getqBI() const ;
    const vector<vector<double>>& getIB() const ;
    void writeStateHistoryDictCsv(const std::string& filename) const;
};

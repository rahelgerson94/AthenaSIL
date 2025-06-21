#pragma once
#include "Frame.h"
#include "Quaternion.h"
#include "Constants.h"
#include <cstring>


class Vehicle : public Frame 
{
    public:
    
    vector<double> _qLB;
    vector<vector<double>> _LB;
    vector<double> _anglesWrtLos ;
    vector<vector<double>> anglesWrtInertialHist;
    vector<vector<double>>  _anglesWrtLosHist;
    vector<vector<double>> qIBhist;
    double _vB;
    Vehicle(const vector<vector<double>>& x0,
            const vector<double>& anglesWrtInertial,
            double dt,
            const std::string& name = ""); 

    Vehicle(const vector<vector<double>>& x0,
            const vector<double>& anglesWrtInertial,
            double dt,
            const vector<double>& aInB,
            const std::string& name = "" );
    

    vector<vector<double>> initInertialStates(const vector<double>& rInI,
                                                  const vector<double>& vInB);

    vector<vector<double>> initInertialStates(const vector<double>& rInI,
                                                  const vector<double>& vInB,
                                                  const vector<double>& aInB);
    void update(const vector<double>& accelInFrame,
                const vector<double>& anglesLosInertial,
                const std::string& accelFrame = "b",
                const vector<double>* yprCmd = nullptr);

    vector<double> computeAngleRatesWrtInertial(const vector<double>& a);

    vector<double> computeAnglesWrtLos(const vector<double>& anglesLosInertial);

    const vector<double>& getAnglesWrtLos() const;

    void storeStatesInEnglishUnits() ;

    void setPos(const vector<double>& r, const std::string& frame = "i") ;
    void setVel(const vector<double>& v, const std::string& frame);

    void setAccel(const vector<double>& a, const std::string& frame) ;

    void setAnglesWrtInertial(const vector<double>& ea) ;
    vector<vector<double>> getInertialStates() const ;
    void writeStateHistoryDictCsv(const std::string& filename) const;
    
    static void computeCurStates(
        const vector<vector<double>>& xIprev,
        double vB,
        const vector<double>& anglesWrtInertialPrev,
        const vector<double>& anglesLosInertial,
        const vector<double>& aCmdInLos,
        vector<vector<double>>& xI,
        vector<double>& anglesWrtInertial,
        vector<double>& anglesWrtLos);


    static vector<double> computeAnglesWrtInertial(
        const vector<double>& aCmdInBodyCur,
        const vector<double>& anglesWrtInertialPrev,
        double vB);

    static vector<double> computeAnglesWrtLos(
        const vector<vector<double>>& xIcur,
        const vector<double>& anglesLosInertial);

};

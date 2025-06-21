// Header file: EngagementManager.h
#pragma once
#include <string>
#include <map>
#include "Constants.h"
#include "Quaternion.h"
#include "VectorMath.h"
#include "Frame.h"
class Waypoint
{
    public:
        vector<vector<double>> _xI;
        vector<double> _anglesWrtInertial;
    Waypoint(vector<vector<double>>xI,
            vector<double> anglesWrtInertial):
            _xI(xI),
            _anglesWrtInertial(anglesWrtInertial)
            {

            }
    Waypoint(){}
    Waypoint convert2EnglishUnits() const 
    {
        vector<std::vector<double>> xI 
        = {
        _xI[R] * M2FT,
        _xI[V] * M2FT,
        _xI[A] * M2FT
        };

        vector<double> angles = _anglesWrtInertial * RAD2DEG;
        return Waypoint(xI, angles);
    }
};

class EngagementManager : public Frame {
public:
    vector<double> _w;
    vector<double> _wGhose;
    vector<vector<double>> _wGhoseHist;
    vector<vector<double>> _wWeissHist;
    vector<vector<double>> _anglesWrtInertialHist;
    vector<double> _angleRates321;
    double _accelMax;
    double _accelMin;
    double _timeToIntercept;
    Waypoint _waypoint;
    vector<Waypoint> _waypointHist;
    EngagementManager(const vector<vector<double>>& xTi,
                      const vector<vector<double>>& xPi,
                      double dt,
                      double accelMaxVal,
                      double accelMinVal);

    vector<double> initAngles();

    void update(const vector<vector<double>>& xTi,
                const vector<double>& tAnglesWrtLos,
                const vector<vector<double>>& xPi,
                const vector<double>& pAnglesWrtLos);

    vector<double> computeAccelCmdInLos(const vector<vector<double>>& xPi,
                                             const vector<vector<double>>& xTi);
    

    vector<double> computeAccelCmdInLos(const vector<vector<double>>& xPi);
    vector<double> computeAccelCmdInBody(const vector<vector<double>>& xPi,
                                              const vector<double>& anglesPandLos);

    void storeStatesInEnglishUnits();

    void updateAnglesWrtInertial();

    vector<double> computeOmegaGhose(const vector<vector<double>>& xTi,
                                          const vector<double>& anglesWrtLosT,
                                          const vector<vector<double>>& xPi,
                                          const vector<double>& anglesWrtLosP);

    void setTimeToIntercept(double tf);
    double getTimeToIntercept() const;

    void computeWaypoint(
        vector<vector<double>> xTi,            // 3x3: [r, v, a]
        double vBt,
        vector<double> tAnglesWrtLos,               // 3x1
        vector<double> tAnglesWrtInertial,          // 3x1
        vector<vector<double>> xPi,            // 3x3: [r, v, a]
        double vBp,
        vector<double> pAnglesWrtLos,               // 3x1
        vector<double> pAnglesWrtInertial,          // 3x1
        int numDts2lookahead,
        vector<double> aCmdInLos,
        int Nprime);


    void computeCurStates(
        const vector<vector<double>>& xTi,
        const vector<double>& tAnglesWrtLos,
        const vector<vector<double>>& xPi,
        const vector<double>& pAnglesWrtLos,
        int Nprime,
        vector<double>& aCmdInLos, //output
        vector<double>& anglesWrtInertial //output
    );
    
    vector<double> computeAccelCmdInLos(
        const vector<double>& anglesWrtInertial,
        const vector<double>& vP,
        const vector<double>& losRate,
        int Nprime);

    vector<double> computeLosRate(
        const vector<vector<double>>& xTi,
        const vector<double>& tAnglesWrtLos,
        const vector<vector<double>>& xPi,
        const vector<double>& pAnglesWrtLos);
    void writeStateHistoryDictCsv(const string& filename) const;
};

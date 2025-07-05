// Header file: Guidance.h
#pragma once
#include <string>
#include <map>
#include "Constants.h"
#include "Quaternion.h"
#include "VectorMath.h"
#include "Frame.h"
#include "Vehicle.h"
namespace GC = GuidanceConstants;

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
        _xI[GC::POS] * GC::M2FT,
        _xI[GC::VEL] * GC::M2FT,
        _xI[GC::ACCEL] * GC::M2FT
        };

        vector<double> angles = _anglesWrtInertial * GC::RAD2DEG;
        return Waypoint(xI, angles);
    }
};

class Guidance : public Frame {
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
    int _Nprime;
    int _numDts2lookahead;
    vector<double> _aCmdInLos;
    Guidance(const vector<vector<double>>& xTi,
                const vector<vector<double>>& xPi,
                double dt,
                double accelMaxVal,
                double accelMinVal,
                const int Nprime,
                int _numDtsToLookahead);
    
    Guidance(const vector<vector<double>>& xTi,
                const vector<double>& tAnglesWrtLos,
                const vector<vector<double>>& xPi,
                const vector<double>& pAnglesWrtLos,
                double dt,
                double accelMaxVal,
                double accelMinVal,
                const int Nprime,
                 int _numDts2lookahead);

    vector<double> initAngles();

    vector<vector<double>> computeRelStates();
    void update(const vector<vector<double>>& xTi,
                const vector<double>& tAnglesWrtLos,
                const vector<vector<double>>& xPi,
                const vector<double>& pAnglesWrtLos);
    void update(void);
    void computeAccelCmdInLos(const vector<vector<double>>& xPi,
                              const vector<vector<double>>& xTi);
    
    void computeAccelCmdInLos(const vector<vector<double>>& xTi);

    /* this method version is used in propagate states*/
    vector<double> computeAccelCmdInLos(
        const vector<double>& anglesWrtInertial,
        const vector<double>& vP,
        const vector<double>& losRate);

    void computeAccelCmdInLos(void);
    vector<double> computeAccelCmdInLosTmp(const vector<vector<double>>& xPi) ;

    vector<double> computeAccelCmdInBody(const vector<vector<double>>& xPi,
                                        const vector<double>& anglesPandLos);
    void storeStatesInEnglishUnits();
    void storeStatesInEnglishUnitsArdupilot();
    void updateAnglesWrtInertial();

    vector<double> computeOmegaGhose(const vector<vector<double>>& xTi,
                                          const vector<double>& anglesWrtLosT,
                                          const vector<vector<double>>& xPi,
                                          const vector<double>& anglesWrtLosP);
    void computeOmegaGhose(void);
    
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
        vector<double> aCmdInLos);

    void computeWaypoint(void);

    void computeCurStates(
        const vector<vector<double>>& xTi,
        const vector<double>& tAnglesWrtLos,
        const vector<vector<double>>& xPi,
        const vector<double>& pAnglesWrtLos,
        vector<double>& aCmdInLos, //output
        vector<double>& anglesWrtInertial //output
    );
    
    void computeCurStates(
        vector<double>& aCmdInLos, //output
        vector<double>& anglesWrtInertial //output
    );

    vector<double> computeLosRate(
        const vector<vector<double>>& xTi,
        const vector<double>& tAnglesWrtLos,
        const vector<vector<double>>& xPi,
        const vector<double>& pAnglesWrtLos);
    
    void writeStateHistoryDictCsv(void) const;
    void writeStateHistoryDictCsvArdupilot(void) const;
    
    /* ---------- getters and setters ----------*/
    void setTimeToIntercept(double tf);
    double getTimeToIntercept() const;
    vector<double> getOmegaGhose();
    vector<double> getAccelCmdInLos();
    Waypoint getWaypoint();
    //private:
    Vehicle _pursuer;
    Vehicle _target;
};

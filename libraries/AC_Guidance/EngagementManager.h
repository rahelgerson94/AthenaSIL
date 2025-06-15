// Header file: EngagementManager.h
#pragma once
#include <vector>
#include <string>
#include <map>
#include "Constants.h"
#include "Quaternion.h"
#include "VectorMath.h"
#include "Frame.h"
//#include "json.hpp"

class EngagementManager : public Frame {
public:
    std::vector<std::vector<double>> _xI;
    double _dt;
    std::vector<double> _w;
    std::vector<double> _wGhose;
    std::vector<std::vector<double>> _wGhoseHist;
    std::vector<std::vector<double>> _wWeissHist;
    std::vector<std::vector<double>> _anglesWrtInertialHist;
    std::vector<double> _angleRates321;
    double _accelMax;
    double _accelMin;
    double _timeToIntercept;

    EngagementManager(const std::vector<std::vector<double>>& xTi,
                      const std::vector<std::vector<double>>& xPi,
                      double dt,
                      double accelMaxVal,
                      double accelMinVal);

    std::vector<double> initAngles();

    void update(const std::vector<std::vector<double>>& xTi,
                const std::vector<double>& tAnglesWrtLos,
                const std::vector<std::vector<double>>& xPi,
                const std::vector<double>& pAnglesWrtLos);

    std::vector<double> computeAccelCmdInLos(const std::vector<std::vector<double>>& xPi,
                                             const std::vector<std::vector<double>>& xTi);
    

    std::vector<double> computeAccelCmdInLos(const std::vector<std::vector<double>>& xPi);
    std::vector<double> computeAccelCmdInBody(const std::vector<std::vector<double>>& xPi,
                                              const std::vector<double>& anglesPandLos);

    void storeStatesInEnglishUnits();

    //static std::map<std::string, std::vector<double>> parseConfig(const std::string& cfgFile);

    void updateAnglesWrtInertial();

    std::vector<double> computeOmegaGhose(const std::vector<std::vector<double>>& xTi,
                                          const std::vector<double>& anglesWrtLosT,
                                          const std::vector<std::vector<double>>& xPi,
                                          const std::vector<double>& anglesWrtLosP);

    void setTimeToIntercept(double tf);
    double getTimeToIntercept() const;
};

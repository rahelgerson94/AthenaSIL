// Header + Implementation: EngagementManager.cpp
#include "EngagementManager.h"

using std::vector;
using std::string;
using std::cout;
using std::endl;
EngagementManager::EngagementManager(const vector<vector<double>>& xTi,
                                     const vector<vector<double>>& xPi,
                                     double dt,
                                     double accelMaxVal,
                                     double accelMinVal)
    : Frame({{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}, {0.0, 0.0, 0.0}, dt, "los"),
      _dt(dt),
      _w(3, 0.0),
      _wGhose(3, 0.0),
      _angleRates321(3, 0.0),
      _accelMax(accelMaxVal * FT2M),
      _accelMin(accelMinVal * FT2M),
      _timeToIntercept(nan(""))
{
    _xI = vector<vector<double>>(3, vector<double>(3, 0.0));
    _xI[R] = (xTi[R] - xPi[R]) * FT2M;
    _xI[V] = (xTi[V] - xPi[V]) * FT2M;
    _anglesWrtInertial = initAngles();
    _IB = Quaternion::dcmFromEulerAngles32(_anglesWrtInertial);
    _qIB = Quaternion::quaternionFromEulerAngles321(_anglesWrtInertial);
    _qBI = Quaternion::conjugate(_qIB);
}

vector<double> EngagementManager::initAngles() {
    vector<double> angles(3, 0.0);
    const auto& r = _xI[R];
    angles[Z] = atan2(r[Y], r[X]);
    angles[Y] = atan2(r[Z], sqrt(r[X]*r[X] + r[Y]*r[Y]));
    return angles;
}

void EngagementManager::update(const vector<vector<double>>& xTi,
                               const vector<double>& tAnglesWrtLos,
                               const vector<vector<double>>& xPi,
                               const vector<double>& pAnglesWrtLos) {

    _xI[R] =  xTi[R]- xPi[R];
    _xI[V] =  xTi[V]- xPi[V];
    double r2 = norm(_xI[R])*norm(_xI[R]);
    _w =   cross(_xI[R], _xI[V])/r2;
    _wGhose = computeOmegaGhose(xTi, tAnglesWrtLos, xPi, pAnglesWrtLos);
    updateAnglesWrtInertial();
    _IB = Quaternion::dcmFromEulerAngles32(_anglesWrtInertial);
    auto dq = Quaternion::computeDerivative(_qIB, _wGhose);
    _qIB = _qIB + _dt * dq;
    _qIB = _qIB/norm(_qIB);
    _qBI = Quaternion::conjugate(_qIB);
}

vector<double> EngagementManager::computeAccelCmdInLos(const vector<vector<double>>& xPi,
                                                       const vector<vector<double>>& xTi) {
    vector<double> vPinL =  Quaternion::rotateVectorByQuaternion(_qIB, xPi[V]);
    vector<double> aCmdInLos =  cross(_wGhose, vPinL);
    vector<double> r1 = (_xI[R])/norm(_xI[R]);
    vector<double> atPerp =  cross(r1, (xTi)[A]);
    aCmdInLos = aCmdInLos + 0.5*atPerp;
    return aCmdInLos;
}

vector<double> EngagementManager::computeAccelCmdInLos(const vector<vector<double>>& xPi) 
{
    vector<double> vPinL =  Quaternion::rotateVectorByQuaternion(_qIB, xPi[V]);
    vector<double> aCmdInLos =  cross(_wGhose, vPinL);
    return clipVec(aCmdInLos, _accelMin, _accelMax);
}

vector<double> EngagementManager::computeAccelCmdInBody(const vector<vector<double>>& xPi,
                                                        const vector<double>& anglesPandLos) {
    vector<double> aCmdInB(3, 0.0);
    double Vm = norm(xPi[V]);
    double theta = anglesPandLos[Y];
    double psi = anglesPandLos[Z];
    aCmdInB[Y] = Vm * (-_wGhose[Y]*sin(theta)*sin(psi) + _wGhose[Z]*cos(theta));
    aCmdInB[Z] = Vm * (-_wGhose[Y]*cos(psi));
    return clipVec(aCmdInB, _accelMin, _accelMax);
}

void EngagementManager::storeStatesInEnglishUnits() 
{
    _wGhoseHist.push_back(_wGhose*RAD2DEG);
    _wWeissHist.push_back(_w*RAD2DEG);
}


void EngagementManager::updateAnglesWrtInertial() {
    const auto& r = _xI[R];
    _anglesWrtInertial[Z] = atan2(r[Y], r[X]);
    _anglesWrtInertial[Y] = atan2(r[Z], sqrt(r[X]*r[X] + r[Y]*r[Y]));
}

vector<double> EngagementManager::computeOmegaGhose(const vector<vector<double>>& xTi,
                                                    const vector<double>& anglesWrtLosT,
                                                    const vector<vector<double>>& xPi,
                                                    const vector<double>& anglesWrtLosP) {
    vector<double> angleRates(3, 0.0);
    double vt = norm(xTi[V]);
    double vp = norm(xPi[V]);
    double rMag = norm(_xI[R]);
    double thetaT = anglesWrtLosT[Y], psiT = anglesWrtLosT[Z];
    double thetaP = anglesWrtLosP[Y], psiP = anglesWrtLosP[Z];
    angleRates[Y] = (1/rMag) * (vp * sin(thetaP) - vt * sin(thetaT));
    angleRates[Z] = (1/rMag) * (vt * cos(thetaT) * sin(psiT) - vp * cos(thetaP) * sin(psiP));
    return angleRates;
}

void EngagementManager::setTimeToIntercept(double tf) {
    cout << "tf: " << tf << endl;
    _timeToIntercept = tf;
}

double EngagementManager::getTimeToIntercept() const {
    return _timeToIntercept;
}

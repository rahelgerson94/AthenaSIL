// Header + Implementation: EngagementManager.cpp
#include "EngagementManager.h"
#include "Vehicle.h"

EngagementManager::EngagementManager(const vector<vector<double>>& xTi,
                                     const vector<vector<double>>& xPi,
                                     double dt,
                                     double accelMaxVal,
                                     double accelMinVal)
    : Frame({{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}, {0.0, 0.0, 0.0}, dt, "los"),
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
    _waypoint = Waypoint(_xI,
                          {0,0,0});
    _dt = dt;
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
    _waypointHist.push_back(_waypoint.convert2EnglishUnits());
    _qIBhist.push_back(_qIB);
    _anglesWrtInertialHist.push_back(_anglesWrtInertial * RAD2DEG);
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

double EngagementManager::getTimeToIntercept() const 
{
    return _timeToIntercept;
}


void EngagementManager::computeCurStates(
    const vector<vector<double>>& xTi,
    const vector<double>& tAnglesWrtLos,
    const vector<vector<double>>& xPi,
    const vector<double>& pAnglesWrtLos,
    int Nprime,
    vector<double>& aCmdInLos, //output
    vector<double>& anglesWrtInertialNew //output
)
{
    vector<double> rtp = xTi[R] - xPi[R];
    vector<double> lambdaDot = computeLosRate(xTi, tAnglesWrtLos, xPi, pAnglesWrtLos);

    anglesWrtInertialNew[Z] = atan2(rtp[Y], rtp[X]);
    anglesWrtInertialNew[Y] = atan2(rtp[Z], sqrt(rtp[X]*rtp[X] + rtp[Y]*rtp[Y]));

    aCmdInLos = computeAccelCmdInLos(
        anglesWrtInertialNew, xPi[V], lambdaDot, Nprime);
}

vector<double> EngagementManager::computeAccelCmdInLos(
    const vector<double>& anglesWrtInertial,
    const vector<double>& vP,
    const vector<double>& losRate,
    int Nprime)
{
    vector<double> qIL = Quaternion::quaternionFromEulerAngles321(anglesWrtInertial);
    vector<double> vPinL = Quaternion::rotateVectorByQuaternion(qIL, vP);
    return Nprime * cross(losRate, vPinL);
}

vector<double> EngagementManager::computeLosRate(
    const vector<vector<double>>& xTi,
    const vector<double>& tAnglesWrtLos,
    const vector<vector<double>>& xPi,
    const vector<double>& pAnglesWrtLos)
{
    vector<double> angleRates(3, 0.0);
    double vt = norm(xTi[V]);
    double vp = norm(xPi[V]);

    vector<double> rtp = xTi[R] - xPi[R];
    vector<double> vtp = xTi[V] - xPi[V];

    double r = norm(rtp);

    double thetaT = tAnglesWrtLos[1], psiT = tAnglesWrtLos[2];
    double thetaP = pAnglesWrtLos[1], psiP = pAnglesWrtLos[2];

    angleRates[X] = 0.0;
    angleRates[Y] = (1/r) * (vp * sin(thetaP) - vt * sin(thetaT));
    angleRates[Z] = (1/r) * (vt * cos(thetaT) * sin(psiT) - vp * cos(thetaP) * sin(psiP));

    return angleRates;
}

void EngagementManager::writeStateHistoryDictCsv(const std::string& filename) const 
{
    std::ofstream file("output/" + filename + ".csv");
    if (!file.is_open()) {
        throw std::runtime_error("Unable to open file: " + filename);
    }

    // Ensure output directory exists (optional, but safe)
    std::filesystem::create_directories("output");

    //  CSV header
    file << "yaw,pitch,roll,"
         << "qIBw,qIBx,qIBy,qIBz,"
         << "RxWp,RyWp,RzWp,"
         << "VxWp,VyWp,VzWp,"
         << "AxWp,AyWp,AzWp,"
         << "rollWp,pitchWp,yawWp\n";

    size_t n = _anglesWrtInertialHist.size();
    printf("len(hist): %zu\n", n);

    for (size_t i = 0; i < n; ++i) {
        const auto& angles = _anglesWrtInertialHist[i];   // [yaw, pitch, roll]
        const auto& q = _qIBhist[i];                      // quaternion [w, x, y, z]

        // Write yaw, pitch, roll
        file << std::fixed << std::setprecision(8)
             << angles[2] << "," << angles[1] << "," << angles[0] << ",";

        // Write quaternion
        file << q[0] << "," << q[1] << "," << q[2] << "," << q[3] << ",";

        // Write waypoint state (assume _waypointHist.size() >= n)
        const auto& wp = _waypointHist[i];
        const auto& rw = wp._xI[R];
        const auto& vw = wp._xI[V];
        const auto& aw = wp._xI[A];
        const auto& angWp = wp._anglesWrtInertial;

        file << rw[0] << "," << rw[1] << "," << rw[2] << ","
             << vw[0] << "," << vw[1] << "," << vw[2] << ","
             << aw[0] << "," << aw[1] << "," << aw[2] << ","
             << angWp[0] << "," << angWp[1] << "," << angWp[2] << "\n";  // roll, pitch, yaw
    }

    file.close();
}


void EngagementManager::computeWaypoint(
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
    int Nprime)
{
    
    vector<double> losAnglesWrtInertial = _anglesWrtInertial; 
    vector<vector<double>> xPiNew = {{0,0,0}, {0,0,0}, {0,0,0}};
    vector<double>pAnglesWrtInertialNew = {0,0,0};
    vector<double> pAnglesWrtLosNew= {0,0,0};

    vector<vector<double>> xTiNew = {{0,0,0}, {0,0,0}, {0,0,0}};
    vector<double>tAnglesWrtInertialNew= {0,0,0};
    vector<double> tAnglesWrtLosNew = {0,0,0};

    vector<double> aCmdInLosNew = aCmdInLos;
    vector<double> losAnglesWrtInertialNew = _anglesWrtInertial;
    for (int i = 0; i < numDts2lookahead; ++i) 
    {
        
        Vehicle::computeCurStates(
            xPi,
            vBp,
            pAnglesWrtInertial,
            losAnglesWrtInertialNew,
            aCmdInLosNew,
            xPiNew, //output
            pAnglesWrtInertialNew,//output
            pAnglesWrtLosNew//output
            );

        // Predict target state (no acceleration)
        Vehicle::computeCurStates(
            xTi,
            vBt,
            tAnglesWrtInertial,
            losAnglesWrtInertialNew,
            vector<double>{0.0, 0.0, 0.0},
            xTiNew,//output
            tAnglesWrtInertialNew,//output
            tAnglesWrtLosNew//output
        );

        // update LOS dynamics with updated pursuer, tgt states
        EngagementManager::computeCurStates(
            xTiNew,
            tAnglesWrtLosNew,
            xPiNew,
            pAnglesWrtLosNew,
            Nprime,
            aCmdInLosNew,//output
            losAnglesWrtInertialNew//output
            );
    }//end for loop

    // Rotate LOS acceleration into inertial frame
    vector<double> qIB = Quaternion::quaternionFromEulerAngles321(losAnglesWrtInertial);
    vector<double> qBI = Quaternion::conjugate(qIB);
    vector<double> aCmdInI = Quaternion::rotateVectorByQuaternion(qBI, aCmdInLos);

    // Append acceleration to pursuer state
    xPi.push_back(aCmdInI); // Now xPi = [r, v, a]

    _waypoint = Waypoint(xPi, pAnglesWrtInertial); // Set waypoint
}
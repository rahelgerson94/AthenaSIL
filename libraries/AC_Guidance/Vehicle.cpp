#include "Vehicle.h"
#include <cmath>
Vehicle::Vehicle(const vector<vector<double>>& x0,
                 const vector<double>& anglesWrtInertial,
                 double dt,
                 const std::string& name)
    : Frame(x0, anglesWrtInertial, dt, name)
{
    _xI = initInertialStates(x0[R]*FT2M, 
                            x0[V]*FT2M);
    _qLB = {0,0,0,0};
    _anglesWrtLos= {0,0,0};
    
}

vector<vector<double>> Vehicle::initInertialStates(const vector<double>& rInI,
                                                    const vector<double>& vInB)
{
    vector<double> vInI =Quaternion::rotateVectorByQuaternion(_qBI, vInB);
    vector<double> aInI = {0.0, 0.0, 0.0};
    return {rInI, vInI, aInI};
}

void Vehicle::update(const vector<double>& accelInFrame,
                     const vector<double>& anglesLosInertial,
                     const std::string& accelFrame,
                     const vector<double>* yprCmd)
{
    vector<double> accelInBody, accelInLos;
    if (accelFrame == "l" || accelFrame == "L") 
    {
        accelInLos =  accelInFrame;
    } else 
    {
        accelInBody = accelInFrame;
    }

    double gamma = _anglesWrtInertial[Y];
    double phi = _anglesWrtInertial[Z];
    double vB = _xB[V][X];
    vector<double> vInIprev = _xI[V];
    _xI[V] = 
    {
        vB * cos(phi) * cos(gamma),
        vB * sin(phi) * cos(gamma),
        vB * sin(gamma)
    };

    _xI[R] = _xI[R] + _xI[V] * _dt;
    _xI[A] = (_xI[V] - vInIprev)/_dt;
    _anglesWrtLos = computeAnglesWrtLos(anglesLosInertial);
    _qLB = Quaternion::quaternionFromEulerAngles321(_anglesWrtLos);
    if (accelFrame ==  "l" || accelFrame == "L")
    {
        accelInBody = Quaternion::rotateVectorByQuaternion(_qLB, accelInLos);
    }
    vector<double> angleRatesWrtInertial = computeAngleRatesWrtInertial(accelInBody);
    _anglesWrtInertial = _anglesWrtInertial + angleRatesWrtInertial* _dt;

    _qIB =Quaternion::quaternionFromEulerAngles321(_anglesWrtInertial);

    _qBI =Quaternion::conjugate(_qIB);
    _xB[A] = accelInBody;
    _xB[V] = _xB[V] + _xB[A] * _dt;
    _xB[R] = vector<double>(3,0);
}

vector<double> Vehicle::computeAngleRatesWrtInertial(const vector<double>& a)
{
    double vx = _xB[V][X];
    double theta = _anglesWrtInertial[Y];
    double thetaDot = (a[Z] == 0 && vx == 0) ? 0 : a[Z] / vx;
    double psiDot = (a[Y] == 0 && vx == 0) ? 0 : a[Y] / (vx * cos(theta));
    return {0.0, thetaDot, psiDot};
}

vector<double> Vehicle::computeAnglesWrtLos(const vector<double>& anglesLosInertial)
{
    vector<double> qIL = Quaternion::quaternionFromEulerAngles321(anglesLosInertial);
    auto vLos = Quaternion::rotateVectorByQuaternion(qIL, _xI[V]);
    vector<double> angles = {0.0, 0.0, 0.0};
    angles[Y] = atan2(vLos[Z], std::sqrt(vLos[Y]*vLos[Y] + vLos[X]*vLos[X]));
    angles[Z] = atan2(vLos[Y], vLos[X]);
    return angles;
}

const vector<double>& Vehicle::getAnglesWrtLos() const {
    return _anglesWrtLos;
}

void Vehicle::storeStatesInEnglishUnits() {
    auto statesI = Frame::getInertialStates();
    Frame::storeStatesInEnglishUnits(statesI[R], statesI[V], statesI[A], "I");
    auto statesB = Frame::getBodyStates();
    Frame::storeStatesInEnglishUnits(statesB[R], statesB[V], statesB[A], "B");
    _anglesWrtInertialHist.push_back(_anglesWrtInertial * RAD2DEG);
    _anglesWrtLosHist.push_back(_anglesWrtLos * RAD2DEG);
    _qIBhist.push_back(_qIB);
}

void Vehicle::setPos(const vector<double>& r, const std::string& frame) {
    if (frame == "b") _xB[R] = r;
    else _xI[R] = r;
}

void Vehicle::setVel(const vector<double>& v, const std::string& frame) {
    if (frame == "b") _xB[V] = v;
    else _xI[V] = v;
}

void Vehicle::setAccel(const vector<double>& a, const std::string& frame) {
    if (frame == "b") _xB[A] = a;
    else _xI[A] = a;
}

void Vehicle::setAnglesWrtInertial(const vector<double>& ea) {
    _anglesWrtInertial = ea;
}

vector<vector<double>> Vehicle::getInertialStates() const {
    return _xI;
}

void Vehicle::writeStateHistoryDictCsv(const std::string& filename) const 
{
    std::ofstream file("output/" + filename + ".csv");
    if (!file.is_open()) {
        throw std::runtime_error("Unable to open file: " + filename);
    }

    file << "Rx,Ry,Rz,"
         << "Vx,Vy,Vz,"
         << "Ax,Ay,Az,"
         << "yaw,pitch,roll,"
         << "rollLosBody,pitchLosBody,yawLosBody,"
         << "qIBw,qIBx,qIBy,qIBz\n";

    size_t n = _stateHistInI.size();
    for (size_t i = 0; i < n; ++i) {
        const auto& r = _stateHistInI[i][R];
        const auto& v = _stateHistInI[i][V];
        const auto& a = _stateHistInI[i][A];
        const auto& angles = _anglesWrtInertialHist[i];   // [yaw, pitch, roll] in deg
        const auto& losAngles = _anglesWrtLosHist[i];     // [yaw, pitch, roll] in deg
        const auto& q = _qIBhist[i];                      // quaternion [w, x, y, z]

        file << std::fixed << std::setprecision(8)
             << r[0] << "," << r[1] << "," << r[2] << ","
             << v[0] << "," << v[1] << "," << v[2] << ","
             << a[0] << "," << a[1] << "," << a[2] << ","
             << angles[2] << "," << angles[1] << "," << angles[0] << ","
             << losAngles[2] << "," << losAngles[1] << "," << losAngles[0] << ","
             << q[0] << "," << q[1] << "," << q[2] << "," << q[3] << "\n";
    }

    file.close();
}
#include "Vehicle.h"

Vehicle::Vehicle(){}

Vehicle::Vehicle(const vector<vector<double>>& x0, //{rInI, vInB}
                 const vector<double>& anglesWrtInertial,
                 double dt,
                 const std::string& name)
    : Frame(x0, anglesWrtInertial, dt, name)
{
    _xI = initInertialStates(x0[POS]* FT2M, 
                            x0[VEL]* FT2M);
    _qLB = {0,0,0,0};
    _anglesWrtLos= {0,0,0};  
    _vB = x0[VEL][X]* FT2M;
    _dt = dt;
    
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
                     const std::string& accelFrame)
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
    vector<double> vInIprev = _xI[VEL];
    double vB = _xB[VEL][X];
    _xI[VEL] = 
    {
        vB * cos(phi) * cos(gamma),
        vB * sin(phi) * cos(gamma),
        vB * sin(gamma)
    };

    _xI[POS] = _xI[POS] + _xI[VEL] * _dt;
    _xI[ACCEL] = (_xI[VEL] - vInIprev)/_dt;
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
    _xB[ACCEL] = accelInBody;
    _xB[VEL] = _xB[VEL] + _xB[ACCEL] * _dt;
    _xB[POS] = vector<double>(3,0);
}

vector<double> Vehicle::computeAngleRatesWrtInertial(const vector<double>& a)
{
    double vx = _xB[VEL][X];
    double theta = _anglesWrtInertial[Y];
    double thetaDot = (a[Z] == 0 && vx == 0) ? 0 : a[Z] / vx;
    double psiDot = (a[Y] == 0 && vx == 0) ? 0 : a[Y] / (vx * cos(theta));
    return {0.0, thetaDot, psiDot};
}

vector<double> Vehicle::computeAnglesWrtLos(const vector<double>& anglesLosInertial)
{
    vector<double> qIL = Quaternion::quaternionFromEulerAngles321(anglesLosInertial);
    auto vLos = Quaternion::rotateVectorByQuaternion(qIL, _xI[VEL]);
    vector<double> angles = {0.0, 0.0, 0.0};
    angles[Y] = atan2(vLos[Z], std::sqrt(vLos[Y]*vLos[Y] + vLos[X]*vLos[X]));
    angles[Z] = atan2(vLos[Y], vLos[X]);
    return angles;
}

const vector<double>& Vehicle::getAnglesWrtLos() 
{
    return _anglesWrtLos;
}

void Vehicle::storeStatesInEnglishUnits() {
    auto statesI = Frame::getInertialStates();
    Frame::storeStatesInEnglishUnits(statesI[POS], statesI[VEL], statesI[ACCEL], "I");
    auto statesB = Frame::getBodyStates();
    Frame::storeStatesInEnglishUnits(statesB[POS], statesB[VEL], statesB[ACCEL], "B");
    _anglesWrtInertialHist.push_back(_anglesWrtInertial * RAD2DEG);
    _anglesWrtLosHist.push_back(_anglesWrtLos * RAD2DEG);
    _qIBhist.push_back(_qIB);
}

void Vehicle::setPos(const vector<double>& r, const std::string& frame) {
    if (frame == "b") _xB[POS] = r;
    else _xI[POS] = r;
}

void Vehicle::setVel(const vector<double>& v, const std::string& frame) {
    if (frame == "b") _xB[VEL] = v;
    else _xI[VEL] = v;
}

void Vehicle::setAccel(const vector<double>& a, const std::string& frame) {
    if (frame == "b") _xB[ACCEL] = a;
    else _xI[ACCEL] = a;
}

void Vehicle::setAnglesWrtInertial(const vector<double>& ea) {
    _anglesWrtInertial = ea;
}

vector<vector<double>> Vehicle::getInertialStates() 
{
    return _xI;
}

void Vehicle::writeStateHistoryDictCsv( ) const 
{
    string PROJ_ROOT_PATH = std::getenv("PROJ_ROOT_PATH") ? std::getenv("PROJ_ROOT_PATH") : "";
    string filename = PROJ_ROOT_PATH + "/AthenaSimCpp/output/" + _name + ".csv";
    std::ofstream file(filename);
    
    if (!file.is_open()) {
        throw std::runtime_error("Unable to open file: " + _name);
    }

    file << "Rx,Ry,Rz,"
         << "Vx,Vy,Vz,"
         << "Ax,Ay,Az,"
         << "roll,pitch,yaw,"
         << "rollLosBody,pitchLosBody,yawLosBody,"
         << "qIBw,qIBx,qIBy,qIBz\n";

    size_t n = _stateHistInI.size();
    for (size_t i = 0; i < n; ++i) {
        const auto& r = _stateHistInI[i][POS];
        const auto& v = _stateHistInI[i][VEL];
        const auto& a = _stateHistInI[i][ACCEL];
        const auto& angles = _anglesWrtInertialHist[i];   // [yaw, pitch, roll] in deg
        const auto& losAngles = _anglesWrtLosHist[i];     // [yaw, pitch, roll] in deg
        const auto& q = _qIBhist[i];                      // quaternion [w, x, y, z]

        file << std::fixed << std::setprecision(8)
             << r[0] << "," << r[1] << "," << r[2] << ","
             << v[0] << "," << v[1] << "," << v[2] << ","
             << a[0] << "," << a[1] << "," << a[2] << ","
             << angles[0] << "," << angles[1] << "," << angles[2] << ","
             << losAngles[0] << "," << losAngles[1] << "," << losAngles[2] << ","
             << q[0] << "," << q[1] << "," << q[2] << "," << q[3] << "\n";
    }

    file.close();
}

/*
compute the new states (pos, vel, orientation) that result 
from an incoming guidance command. this function is repeatedly called by 
Guidance::computeWaypoint to propagate the vehcile states some time into future
*/
void Vehicle::computeCurStates(
    const vector<vector<double>>& xIprev,
    double vB,
    const vector<double>& anglesWrtInertialPrev,
    const vector<double>& anglesLosInertial,
    const vector<double>& aCmdInLos,
    vector<vector<double>>& xI, //output
    vector<double>& anglesWrtInertial, //output
    vector<double>& anglesWrtLos)  //output. does not need the previous value 
{

    double gamma = anglesWrtInertialPrev[Y];
    double phi = anglesWrtInertialPrev[Z];

    xI[VEL][X] = vB * std::cos(phi) * std::cos(gamma);
    xI[VEL][Y] = vB * std::sin(phi) * std::cos(gamma);
    xI[VEL][Z] = vB * std::sin(gamma);

    
    xI[POS] = xIprev[POS]+ xI[VEL] * _dt;

    anglesWrtLos = computeAnglesWrtLos(xI, anglesLosInertial);
    vector<double> qLB = Quaternion::quaternionFromEulerAngles321(anglesWrtLos);
    vector<double> aCmdInB = Quaternion::rotateVectorByQuaternion(qLB, aCmdInLos);

    vector<double> angleRates = computeAnglesWrtInertial(aCmdInB, anglesWrtInertialPrev, vB);
    anglesWrtInertial = anglesWrtInertialPrev + angleRates * _dt; 
}

vector<double> Vehicle::computeAnglesWrtInertial(
    const vector<double>& aCmdInBodyCur,
    const vector<double>& anglesWrtInertialPrev,
    double vB)
{
    
    vector<double> angleRatesWrtInertial(3, 0.0);

    if (aCmdInBodyCur[Z] == 0 && vB == 0) {
        angleRatesWrtInertial[Y] = 0;
    } else {
        angleRatesWrtInertial[Y] = aCmdInBodyCur[Z] / vB;
    }

    if (aCmdInBodyCur[Y] == 0 && vB == 0) {
        angleRatesWrtInertial[Z] = 0;
    } else {
        double theta = anglesWrtInertialPrev[Y];
        angleRatesWrtInertial[Z] = aCmdInBodyCur[Y] / (vB * std::cos(theta));
    }

    return angleRatesWrtInertial;
}

vector<double> Vehicle::computeAnglesWrtLos(
    const vector<vector<double>>& xIcur,
    const vector<double>& anglesLosInertial)
{
    vector<double> qIL = Quaternion::quaternionFromEulerAngles321(anglesLosInertial);
    vector<double> vLos = Quaternion::rotateVectorByQuaternion(qIL, xIcur[VEL]);

    vector<double> angles(3, 0.0);
    angles[Y] = std::atan2(vLos[Z], std::sqrt(vLos[X]*vLos[X] + vLos[Y]*vLos[Y]));
    angles[Z] = std::atan2(vLos[Y], vLos[X]);
    return angles;
}

// Header + Implementation: Guidance.cpp
#include "Guidance.h"
#include "Vehicle.h"
namespace GC = GuidanceConstants;
namespace GU = GuidanceUtils;

Guidance::Guidance(const vector<vector<double>>& xTi,
                                     const vector<vector<double>>& xPi,
                                     double dt,
                                     double accelMaxVal,
                                     double accelMinVal,
                                     int Nprime,
                                 int numDts2lookahead=10)
    : Frame({{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}, {0.0, 0.0, 0.0}, dt, "guide"),
      _w(3, 0.0),
      _wGhose(3, 0.0),
      _angleRates321(3, 0.0),
      _accelMax(accelMaxVal * FT2M),
      _accelMin(accelMinVal * FT2M),
      _timeToIntercept(nan(""))
{
    _xI = vector<vector<double>>(3, vector<double>(3, 0.0));
    _xI[POS] = (xTi[POS] - xPi[POS]) * FT2M;
    _xI[VEL] = (xTi[VEL] - xPi[VEL]) * FT2M;
    _anglesWrtInertial = initAngles();
    _IB = GU::Quaternion::dcmFromEulerAngles32(_anglesWrtInertial);
    _qIB = GU::Quaternion::quaternionFromEulerAngles321(_anglesWrtInertial);
    _qBI = GU::Quaternion::conjugate(_qIB);
    _waypoint = Waypoint(_xI,
                          {0,0,0});
    _dt = dt;
    _Nprime = Nprime;
    _numDts2lookahead = numDts2lookahead;
    _aCmdInLos = {0,0,0};
}

/*
constructor for the object that has tgt, pursuer objects as properties
*/
Guidance::Guidance(const vector<vector<double>>& xTi,
                    const vector<double>& tAnglesWrtInertial,
                    const vector<vector<double>>& xPi,
                    const vector<double>& pAnglesWrtInertial,
                    double dt,
                    double accelMaxVal,
                    double accelMinVal,
                     int Nprime,
                     int numDts2lookahead=10)
    : Frame({{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}, {0.0, 0.0, 0.0}, dt, "guide"),
      _w(3, 0.0),
      _wGhose(3, 0.0),
      _angleRates321(3, 0.0),
      _accelMax(accelMaxVal * FT2M),
      _accelMin(accelMinVal * FT2M),
      _timeToIntercept(nan("")),

    _target(Vehicle(xTi,
                    tAnglesWrtInertial,
                    dt, 
                    "target")),
    _pursuer(Vehicle(xPi, 
            pAnglesWrtInertial,
            dt, 
            "pursuer"))
    {

    auto relPosVel = computeRelStates();
    _xI[POS] = _target._xI[POS] - _pursuer._xI[POS];
    _xI[VEL] = _target._xI[VEL] - _pursuer._xI[VEL];
    _anglesWrtInertial = initAngles();
    _IB = GU::Quaternion::dcmFromEulerAngles32(_anglesWrtInertial);
    _qIB = GU::Quaternion::quaternionFromEulerAngles321(_anglesWrtInertial);
    _qBI = GU::Quaternion::conjugate(_qIB);
    _waypoint = Waypoint(_xI,
                          {0,0,0});
    _dt = dt;
    _Nprime = Nprime;
    _numDts2lookahead = numDts2lookahead;
}

vector<double> Guidance::initAngles() {
    vector<double> angles(3, 0.0);
    const auto& r = _xI[POS];
    angles[GC::Z] = atan2(r[GC::Y], r[GC::X]);
    angles[GC::Y] = atan2(r[GC::Z], sqrt(r[GC::X]*r[GC::X] + r[GC::Y]*r[GC::Y]));
    return angles;
}

vector<vector<double>> Guidance::computeRelStates()
{
    vector<double> r = _target._xI[POS] - _pursuer._xI[POS];
    vector<double> v = _target._xI[VEL] - _pursuer._xI[VEL];
    return {r,v};
}

void Guidance::update(void) 
{
    computeAccelCmdInLos(); //updates _aCmdInLos
    _pursuer.update(_aCmdInLos, _anglesWrtInertial, "l");
    _target.update({0.0, 0.0, 0.0}, _anglesWrtInertial, "b");
    _xI[POS] =  _target._xI[POS] -  _pursuer._xI[POS];
    _xI[VEL] =  _target._xI[VEL] -  _pursuer._xI[VEL];
    double r2 = norm(_xI[POS])*norm(_xI[POS]);
    _w =   cross(_xI[POS], _xI[VEL])/r2;
    computeOmegaGhose(); //updates _wGhose
    updateAnglesWrtInertial();
    _IB = GU::Quaternion::dcmFromEulerAngles32(_anglesWrtInertial);
    auto dq = GU::Quaternion::computeDerivative(_qIB, _wGhose);
    _qIB = _qIB + _dt * dq;
    _qIB = _qIB/norm(_qIB);
    _qBI = GU::Quaternion::conjugate(_qIB);
}

/*
 Guidance::computeOmegaGhose

 TODO: Decide whether to keep this method or the static computeLosRate.
 They are equivalent, but the static method is usable to compute waypoints.

 Computes omegaGhose, which is nearly equivalent to λ̇ (weiss) = (r x v)/r²
 
 The difference is that λ̇_y (ghose) == -(λ̇_y) (weiss)
 This is because Ghose uses the convention of negative 
 (clockwise) rotation about the y-axis, whereas Weiss uses 
 a positive (counterclockwise) rotation about the y-axis.
*/
void Guidance::computeOmegaGhose(void) 
{
    vector<double> losRate = {0,0,0};
    vector<vector<double>>& xTi      = _target._xI;
    vector<double>& anglesWrtLosT    = _target._anglesWrtLos;
    vector<vector<double>>& xPi      = _pursuer._xI;
    vector<double>& anglesWrtLosP    = _pursuer._anglesWrtLos;
    vector<double> angleRates(3, 0.0);
    double vt = norm(xTi[VEL]);
    double vp = norm(xPi[VEL]);
    double rMag = norm(_xI[POS]);
    double thetaT = anglesWrtLosT[GC::Y], psiT = anglesWrtLosT[GC::Z];
    double thetaP = anglesWrtLosP[GC::Y], psiP = anglesWrtLosP[GC::Z];
    _wGhose[GC::Y] = (1/rMag) * (vp * sin(thetaP) - vt * sin(thetaT));
    _wGhose[GC::Z] = (1/rMag) * (vt * cos(thetaT) * sin(psiT) - vp * cos(thetaP) * sin(psiP));
}

/*
compute acceleration command and new orientation that would
result from applying it.
this function is repeatedly called by computeWaypoint to
propagate guidance states some time into future
*/
void Guidance::computeCurStates(vector<double>& aCmdInLos, //output
                                        vector<double>& anglesWrtInertialNew //output
)
{
    vector<vector<double>>& xTi      = _target._xI;
    vector<double>& tAnglesWrtLos    = _target._anglesWrtLos;
    vector<vector<double>>& xPi      = _pursuer._xI;
    vector<double>& pAnglesWrtLos    = _pursuer._anglesWrtLos;

    vector<double> rtp = xTi[POS] - xPi[POS];
    vector<double> lambdaDot = computeLosRate(xTi, tAnglesWrtLos, xPi, pAnglesWrtLos);

    anglesWrtInertialNew[GC::Z] = atan2(rtp[GC::Y], rtp[GC::X]);
    anglesWrtInertialNew[GC::Y] = atan2(rtp[GC::Z], sqrt(rtp[GC::X]*rtp[GC::X] + rtp[GC::Y]*rtp[GC::Y]));

    aCmdInLos = computeAccelCmdInLos(
        anglesWrtInertialNew, xPi[VEL], lambdaDot);
}

/*
Biased ProNav
*/
void Guidance::computeAccelCmdInLos(const vector<vector<double>>& xTi) {
    auto xPi = _pursuer._xI;
    vector<double> vPinL =  GU::Quaternion::rotateVectorByQuaternion(_qIB, xPi[VEL]);
    vector<double> aCmdInLos =  cross(_wGhose, vPinL);
    vector<double> r1 = (_xI[POS])/norm(_xI[POS]);
    vector<double> atPerp =  cross(r1, (xTi)[ACCEL]);
    aCmdInLos = aCmdInLos + 0.5*atPerp;
    _aCmdInLos= _Nprime* aCmdInLos;
}

/*
standard ProNav
*/
void Guidance::computeAccelCmdInLos(void) 
{
    auto xPi = _pursuer._xI;
    vector<double> vPinL =  GU::Quaternion::rotateVectorByQuaternion(_qIB, xPi[VEL]);
    vector<double> aCmdInLos =  _Nprime*cross(_wGhose, vPinL);
    _aCmdInLos= clipVec(aCmdInLos, _accelMin, _accelMax);
}

/* called temp because as of right now, I am in the intermediate state 
of making methods, classes compatible with Ardupilot
 */
vector<double> Guidance::computeAccelCmdInLosTmp(const vector<vector<double>>& xPi) 
{
    
    vector<double> vPinL =  GU::Quaternion::rotateVectorByQuaternion(_qIB, xPi[VEL]);
    vector<double> aCmdInLos =  _Nprime*cross(_wGhose, vPinL);
    return  clipVec(aCmdInLos, _accelMin, _accelMax);
}


/*
standard ProNav, same as above, but used for
waypoint generation in computeCurStates
*/
vector<double> Guidance::computeAccelCmdInLos(
    const vector<double>& anglesWrtInertial,
    const vector<double>& vP,
    const vector<double>& losRate)
{
    vector<double> qIL = GU::Quaternion::quaternionFromEulerAngles321(anglesWrtInertial);
    vector<double> vPinL = GU::Quaternion::rotateVectorByQuaternion(qIL, vP);
    vector<double>  aCmdInLos = cross(losRate, vPinL);
    return _Nprime * aCmdInLos;
}

void Guidance::storeStatesInEnglishUnits(void) 
{
    _wGhoseHist.push_back(_wGhose*RAD2DEG);
    _waypointHist.push_back(_waypoint.convert2EnglishUnits());
    _qIBhist.push_back(_qIB);
    _anglesWrtInertialHist.push_back(_anglesWrtInertial * RAD2DEG);
}

/* called ardupilot because this is a method that would be 
compatible with Ardupilot, and as of right now, I am in the intermediate state 
of making methods, classes
*/

void Guidance::storeStatesInEnglishUnitsArdupilot(void) 
{
    _wGhoseHist.push_back(_wGhose*RAD2DEG);
    _waypointHist.push_back(_waypoint.convert2EnglishUnits());
    _qIBhist.push_back(_qIB);
    _anglesWrtInertialHist.push_back(_anglesWrtInertial * RAD2DEG);
    _pursuer.storeStatesInEnglishUnits();
    _target.storeStatesInEnglishUnits();
}


void Guidance::updateAnglesWrtInertial() 
{
    const auto& r = _xI[POS];
    _anglesWrtInertial[GC::Z] = atan2(r[GC::Y], r[GC::X]);
    _anglesWrtInertial[GC::Y] = atan2(r[GC::Z], sqrt(r[GC::X]*r[GC::X] + r[GC::Y]*r[GC::Y]));
}

void Guidance::computeWaypoint(void )
{
    
    vector<double> losAnglesWrtInertial = _anglesWrtInertial; 
    vector<vector<double>> xPiNew = _pursuer._xI;
    vector<double>pAnglesWrtInertialNew = _pursuer._anglesWrtInertial;
    vector<double> pAnglesWrtLosNew= _pursuer._anglesWrtLos;

    vector<vector<double>> xTiNew =  _target._xI;
    vector<double>tAnglesWrtInertialNew= _target._anglesWrtInertial;
    vector<double> tAnglesWrtLosNew =  _target._anglesWrtLos;
    /*
    _aCmdInLos is the accel cmd guidance passed to the pursuer
    in the inner (higher rate) loop. 
    computeWaypoint propagates the Guidance, Pursuer, and Target
    states forward, with _aCmdInLos as the starting point
    */
    vector<double> aCmdInLos = _aCmdInLos; 
    vector<double> aCmdInLosNew = aCmdInLos;
    vector<double> losAnglesWrtInertialNew = _anglesWrtInertial;

    /*
    initialize the vectors that will be propagated forard in time
    */
    auto xPi = _pursuer._xI;
    auto pAnglesWrtInertial = _pursuer._anglesWrtInertial;
    auto xTi = _target._xI;
    auto tAnglesWrtInertial = _target._anglesWrtInertial;

    for (int i = 0; i < _numDts2lookahead; ++i) 
    {
        
        Vehicle::computeCurStates(
            xPi, //xIprev
            _pursuer._vB, //vb
            pAnglesWrtInertial, //anglesWrtInertialPrev
            losAnglesWrtInertialNew, //anglesLosInertial
            aCmdInLosNew, //input, output of Guidance::computeWaypoint
            xPiNew, //output
            pAnglesWrtInertialNew,//output
            pAnglesWrtLosNew//output, not integrated
            );

        // Predict target state (no acceleration)
        Vehicle::computeCurStates(
            xTi,
            _target._vB,
            tAnglesWrtInertial,
            losAnglesWrtInertialNew,//anglesLosInertial
            vector<double>{0.0, 0.0, 0.0},
            xTiNew,//output
            tAnglesWrtInertialNew,//output
            tAnglesWrtLosNew//output, not integrated
        );

        // update LOS dynamics with updated pursuer, tgt states
        Guidance::computeCurStates(
            xTiNew,
            tAnglesWrtLosNew,
            xPiNew,
            pAnglesWrtLosNew,
            aCmdInLosNew,//output
            losAnglesWrtInertialNew//output, not intergated
            );

        /* update the integrated quantieis*/
        xPi = xPiNew;
        xTi = xTiNew;
        pAnglesWrtInertial = pAnglesWrtInertialNew;
        tAnglesWrtInertial = tAnglesWrtInertialNew;
            
    }//end for loop

    // Rotate LOS acceleration into inertial frame
    vector<double> qIB = GU::Quaternion::quaternionFromEulerAngles321(losAnglesWrtInertial);
    vector<double> qBI = GU::Quaternion::conjugate(qIB);
    vector<double> aCmdInI = GU::Quaternion::rotateVectorByQuaternion(qBI, aCmdInLos);

    // Append acceleration to pursuer state
    xPi.push_back(aCmdInI); // Now xPi = [r, v, a]

    _waypoint = Waypoint(xPi, pAnglesWrtInertial); // Set waypoint
}

void Guidance::setTimeToIntercept(double tf) {
    //cout << "tf: " << tf << endl;
    _timeToIntercept = tf;
}

double Guidance::getTimeToIntercept() const 
{
    return _timeToIntercept;
}


void Guidance::writeStateHistoryDictCsv(void) const 
{
    string PROJ_ROOT_PATH = std::getenv("PROJ_ROOT_PATH") ? std::getenv("PROJ_ROOT_PATH") : "";
    string filename = PROJ_ROOT_PATH + "/AthenaSimCpp/output/" + _name + ".csv";
    std::ofstream file(filename);

    if (!file.is_open()) {
        throw std::runtime_error("Unable to open file: " + _name);
    }

    // Ensure output directory exists (optional, but safe)
    std::filesystem::create_directories("output");

    //  CSV header
    file << "roll,pitch,yaw,"
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
             << angles[0] << "," << angles[1] << "," << angles[2] << ",";

        // Write quaternion
        file << q[0] << "," << q[1] << "," << q[2] << "," << q[3] << ",";

        // Write waypoint state (assume _waypointHist.size() >= n)
        const auto& wp = _waypointHist[i];
        const auto& rw = wp._xI[POS];
        const auto& vw = wp._xI[VEL];
        const auto& aw = wp._xI[ACCEL];
        const auto& angWp = wp._anglesWrtInertial;

        file << rw[0] << "," << rw[1] << "," << rw[2] << ","
             << vw[0] << "," << vw[1] << "," << vw[2] << ","
             << aw[0] << "," << aw[1] << "," << aw[2] << ","
             << angWp[0] << "," << angWp[1] << "," << angWp[2] << "\n";  // roll, pitch, yaw
    }

    file.close();
}

void Guidance::writeStateHistoryDictCsvArdupilot(void) const 
{
    string PROJ_ROOT_PATH = std::getenv("PROJ_ROOT_PATH") ? std::getenv("PROJ_ROOT_PATH") : "";
    string filename = PROJ_ROOT_PATH + "/AthenaSimCpp/output/" + _name + ".csv";
    std::ofstream file(filename);

    if (!file.is_open()) {
        throw std::runtime_error("Unable to open file: " + _name);
    }

    // Ensure output directory exists (optional, but safe)
    std::filesystem::create_directories("output");

    //  CSV header
    file << "roll,pitch,yaw,"
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
             << angles[0] << "," << angles[1] << "," << angles[2] << ",";

        // Write quaternion
        file << q[0] << "," << q[1] << "," << q[2] << "," << q[3] << ",";

        // Write waypoint state (assume _waypointHist.size() >= n)
        const auto& wp = _waypointHist[i];
        const auto& rw = wp._xI[POS];
        const auto& vw = wp._xI[VEL];
        const auto& aw = wp._xI[ACCEL];
        const auto& angWp = wp._anglesWrtInertial;

        file << rw[0] << "," << rw[1] << "," << rw[2] << ","
             << vw[0] << "," << vw[1] << "," << vw[2] << ","
             << aw[0] << "," << aw[1] << "," << aw[2] << ","
             << angWp[0] << "," << angWp[1] << "," << angWp[2] << "\n";  // roll, pitch, yaw
    }

    file.close();

    _target.writeStateHistoryDictCsv();
    _pursuer.writeStateHistoryDictCsv();
}

vector<double> Guidance::getOmegaGhose()    { return _wGhose;}
vector<double> Guidance::getAccelCmdInLos()  { return _aCmdInLos;}
Waypoint Guidance::getWaypoint()            { return _waypoint;}

/*-------------------- non void methods -------------------- */
vector<double> Guidance::computeOmegaGhose(const vector<vector<double>>& xTi,
                                                    const vector<double>& anglesWrtLosT,
                                                    const vector<vector<double>>& xPi,
                                                    const vector<double>& anglesWrtLosP) {
    vector<double> angleRates(3, 0.0);
    double vt = norm(xTi[VEL]);
    double vp = norm(xPi[VEL]);
    double rMag = norm(_xI[POS]);
    double thetaT = anglesWrtLosT[GC::Y], psiT = anglesWrtLosT[GC::Z];
    double thetaP = anglesWrtLosP[GC::Y], psiP = anglesWrtLosP[GC::Z];
    angleRates[GC::Y] = (1/rMag) * (vp * sin(thetaP) - vt * sin(thetaT));
    angleRates[GC::Z] = (1/rMag) * (vt * cos(thetaT) * sin(psiT) - vp * cos(thetaP) * sin(psiP));
    return angleRates;
}

void Guidance::update(const vector<vector<double>>& xTi,
                    const vector<double>& tAnglesWrtLos,
                    const vector<vector<double>>& xPi,
                    const vector<double>& pAnglesWrtLos) {

    _xI[POS] =  xTi[POS]- xPi[POS];
    _xI[VEL] =  xTi[VEL]- xPi[VEL];
    double r2 = norm(_xI[POS])*norm(_xI[POS]);
    _w =   cross(_xI[POS], _xI[VEL])/r2;
    _wGhose = computeOmegaGhose(xTi, tAnglesWrtLos, xPi, pAnglesWrtLos);
    updateAnglesWrtInertial();
    _IB = GU::Quaternion::dcmFromEulerAngles32(_anglesWrtInertial);
    auto dq = GU::Quaternion::computeDerivative(_qIB, _wGhose);
    _qIB = _qIB + _dt * dq;
    _qIB = _qIB/norm(_qIB);
    _qBI = GU::Quaternion::conjugate(_qIB);
    
}


void Guidance::computeCurStates(
    const vector<vector<double>>& xTi,
    const vector<double>& tAnglesWrtLos,
    const vector<vector<double>>& xPi,
    const vector<double>& pAnglesWrtLos,
    vector<double>& aCmdInLos, //output
    vector<double>& anglesWrtInertialNew //output
)
{
    vector<double> rtp = xTi[POS] - xPi[POS];
    vector<double> lambdaDot = computeLosRate(xTi, tAnglesWrtLos, xPi, pAnglesWrtLos);

    anglesWrtInertialNew[GC::Z] = atan2(rtp[GC::Y], rtp[GC::X]);
    anglesWrtInertialNew[GC::Y] = atan2(rtp[GC::Z], sqrt(rtp[GC::X]*rtp[GC::X] + rtp[GC::Y]*rtp[GC::Y]));

    aCmdInLos = computeAccelCmdInLos(
        anglesWrtInertialNew, 
        xPi[VEL], 
        lambdaDot);
}

/*
 Guidance::computeLosRate

 Function to compute the LOS rate as described in
 Ghose's paper, Differential Evolution...
 
 Note, w_ghose is nearly equivalent to λ̇ (weiss) = (r x v)/r²
 
 The difference is that λ̇_y (ghose) == -(λ̇_y) (weiss)
 This is because Ghose uses the convention of negative 
 (clockwise) rotation about the y-axis, whereas Weiss uses 
 a positive (counterclockwise) rotation about the y-axis.
*/
vector<double> Guidance::computeLosRate(
    const vector<vector<double>>& xTi,
    const vector<double>& tAnglesWrtLos,
    const vector<vector<double>>& xPi,
    const vector<double>& pAnglesWrtLos)
{
    vector<double> angleRates(3, 0.0);
    double vt = norm(xTi[VEL]);
    double vp = norm(xPi[VEL]);

    vector<double> rtp = xTi[POS] - xPi[POS];
    vector<double> vtp = xTi[VEL] - xPi[VEL];

    double r = norm(rtp);

    double thetaT = tAnglesWrtLos[1], psiT = tAnglesWrtLos[2];
    double thetaP = pAnglesWrtLos[1], psiP = pAnglesWrtLos[2];

    angleRates[GC::X] = 0.0;
    angleRates[GC::Y] = (1/r) * (vp * sin(thetaP) - vt * sin(thetaT));
    angleRates[GC::Z] = (1/r) * (vt * cos(thetaT) * sin(psiT) - vp * cos(thetaP) * sin(psiP));

    return angleRates;
}

vector<double> Guidance::computeAccelCmdInBody(const vector<vector<double>>& xPi,
                                                const vector<double>& anglesPandLos) {
    vector<double> aCmdInB(3, 0.0);
    double Vm = norm(xPi[VEL]);
    double theta = anglesPandLos[GC::Y];
    double psi = anglesPandLos[GC::Z];
    aCmdInB[GC::Y] = _Nprime * Vm * (-_wGhose[GC::Y]*sin(theta)*sin(psi) + _wGhose[GC::Z]*cos(theta));
    aCmdInB[GC::Z] = _Nprime * Vm * (-_wGhose[GC::Y]*cos(psi));
    return clipVec(aCmdInB, _accelMin, _accelMax);
}


void Guidance::computeWaypoint(
    vector<vector<double>> xTi,            // 3x3: [r, v, a]
    double vBt,
    vector<double> tAnglesWrtLos,               // 3x1
    vector<double> tAnglesWrtInertial,          // 3x1
    vector<vector<double>> xPi,            // 3x3: [r, v, a]
    double vBp,
    vector<double> pAnglesWrtLos,               // 3x1
    vector<double> pAnglesWrtInertial,          // 3x1
    int numDts2lookahead,
    vector<double> aCmdInLos )
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
        Guidance::computeCurStates(
            xTiNew,
            tAnglesWrtLosNew,
            xPiNew,
            pAnglesWrtLosNew,
            aCmdInLosNew,//output
            losAnglesWrtInertialNew//output
            );
    }//end for loop

    // Rotate LOS acceleration into inertial frame
    vector<double> qIB = GU::Quaternion::quaternionFromEulerAngles321(losAnglesWrtInertial);
    vector<double> qBI = GU::Quaternion::conjugate(qIB);
    vector<double> aCmdInI = GU::Quaternion::rotateVectorByQuaternion(qBI, aCmdInLos);

    // Append acceleration to pursuer state
    xPi.push_back(aCmdInI); // Now xPi = [r, v, a]

    _waypoint = Waypoint(xPi, pAnglesWrtInertial); // Set waypoint
}
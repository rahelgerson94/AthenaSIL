#include "Frame.h"


using std::vector;
using std::string;
using std::array;

double Frame::_dt = 0.0;
Frame::Frame(){}

Frame::Frame(const vector<vector<double>>& x0,
             const vector<double>& anglesWrtInertial,
             double dt,
             const string& name)
{
    _dt = 0;
    _name = name;
    _anglesWrtInertial = anglesWrtInertial * DEG2RAD;
    _IB =Quaternion::dcmFromEulerAngles321(_anglesWrtInertial);
    vector<double> rInI = x0[POS] * FT2M;
    vector<double> vInB = x0[VEL] * FT2M;
    
    _qIB =Quaternion::quaternionFromEulerAngles321(_anglesWrtInertial);
    _qBI =Quaternion::conjugate(_qIB);
    _xB = {Quaternion::rotateVectorByQuaternion(_qIB, rInI), vInB, {0.0, 0.0, 0.0} };
    _xI = { rInI,Quaternion::rotateVectorByQuaternion(_qBI, vInB), {0.0, 0.0, 0.0} };
    _xI0 = _xI;
}

void Frame::updateStates(const vector<double>& accelInBody) {
    _xB[ACCEL] = accelInBody;
    _xB[VEL] = accelInBody * _dt;
    vector<double> xI0VRot =Quaternion::rotateVectorByQuaternion(_qIB, _xI0[VEL]);
    _xB[POS] = _xB[POS] + xI0VRot * _dt + accelInBody * (0.5f * _dt * _dt);
}


void Frame::storeStatesInEnglishUnits(const vector<double>& r,
                                      const vector<double>& v,
                                      const vector<double>& a,
                                      const string& frame) {
    vector<vector<double>> states = { r * M2FT, v * M2FT, a * M2FT };
    if (frame == "I") _stateHistInI.push_back(states);
    else if (frame == "B") _stateHistInB.push_back(states);
}

const vector<double>& Frame::getAnglesWrtInertial() const { return _anglesWrtInertial; }

vector<vector<double>> Frame::getInertialStates() const {
    vector<double> aInI =Quaternion::rotateVectorByQuaternion(_qBI, _xB[ACCEL]);
    return { _xI[POS], _xI[VEL], aInI };
}

vector<vector<double>> Frame::getBodyStates() const {
    vector<double> rB = _xB[POS] + _xB[VEL] * _dt;
    return { rB, _xB[VEL], _xB[ACCEL] };
}

// json Frame::parseConfig(const string& cfgFile) {
//     std::ifstream f(cfgFile);
//     json config;
//     f >> config;
//     json result;
//     result["anglesWrtInertial"] = config["anglesWrtInertialDeg"];
//     result["name"] = config.value("name", "");
//     result["x"] = { config["rInI"], config["vInB"] * config["V"] };
//     return result;
// }

vector<vector<double>> Frame::getStates() const { return _xI; }

void Frame::printAngles() const {
    std::cout << "\u03A8: " << RAD2DEG * _anglesWrtInertial[Z] << " deg\n";
    std::cout << "\u03B8: " << RAD2DEG * _anglesWrtInertial[Y] << " deg\n";
    std::cout << "\u03D5: " << RAD2DEG * _anglesWrtInertial[X] << " deg\n\n";
}

void Frame::printStates() const {
    std::cout << "---------" << (_name.empty() ? "vehicleObj" : _name) << "---------\n";

    const auto& statesI = getInertialStates();
    const auto& r = statesI[POS];
    // const auto& v = statesI[VEL];
    // const auto& a = statesI[ACCEL];

    const auto& statesB = getBodyStates();
    const auto& rB = statesB[POS];
    const auto& vB = statesB[VEL];
    const auto& aB = statesB[ACCEL];

    std::cout << "\nrInI:\t";
    for (double val : r * M2FT) std::cout << val << "\t";

    std::cout << "\nvInB:\t";
    for (double val : vB * M2FT) std::cout << val << "\t";

    std::cout << "\naInB:\t";
    for (double val : aB * M2FT) std::cout << val << "\t";

    std::cout << "\n";
    const char* names[3] = {"\u03D5", "\u03B8", "\u03A8"};
    for (int i = 1; i < 3; ++i)
        std::cout << names[i] << " (deg): " << RAD2DEG * _anglesWrtInertial[i] << "\n";
}



const vector<double>& Frame::getqIB() const { return _qIB; }
const vector<double>& Frame::getqBI() const { return _qBI; }
const vector<vector<double>>& Frame::getIB() const { return _IB; }



void Frame::writeStateHistoryDictCsv(void) const 
{
    string PROJ_ROOT_PATH = std::getenv("PROJ_ROOT_PATH") ? std::getenv("PROJ_ROOT_PATH") : "";
    string filename = PROJ_ROOT_PATH + "/AthenaSimCpp/output/" + _name + ".csv";
    std::ofstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Unable to open file: " + _name);
    }

    // Header
    file << "Rx,Ry,Rz,"
         << "Vx,Vy,Vz,"
         << "Ax,Ay,Az,"
         << "roll,pitch,yaw,"
         << "qIBw,qIBx,qIBy,qIBz\n";

    size_t n = _stateHistInI.size();
    for (size_t i = 0; i < n; ++i) {
        const auto& r = _stateHistInI[i][POS];
        const auto& v = _stateHistInI[i][VEL];
        const auto& a = _stateHistInI[i][ACCEL];
        const auto& angles = _anglesWrtInertialHist[i];   // [yaw, pitch, roll] in deg
        const auto& q = _qIBhist[i];                      // quaternion [w, x, y, z]

        file << std::fixed << std::setprecision(8)
             << r[0] << "," << r[1] << "," << r[2] << ","
             << v[0] << "," << v[1] << "," << v[2] << ","
             << a[0] << "," << a[1] << "," << a[2] << ","
             << angles[0] << "," << angles[1] << "," << angles[2] << ","   
             << q[0] << "," << q[1] << "," << q[2] << "," << q[3] << "\n";
    }

    file.close();
}

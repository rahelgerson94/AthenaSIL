#include "Frame.h"

namespace GC = GuidanceConstants;
namespace GU = GuidanceUtils;
double Frame::_dt = 0.0;
Frame::Frame(){}

Frame::Frame(const vector<vector<double>>& x0,
             const vector<double>& anglesWrtInertial,
             double dt,
             const string& name)
{
    _dt = 0;
    _name = name;
    _anglesWrtInertial = anglesWrtInertial * GC::DEG2RAD;
    _IB =GU::Quaternion::dcmFromEulerAngles321(_anglesWrtInertial);
    vector<double> rInI = x0[GC::POS] * GC::FT2M;
    vector<double> vInB = x0[GC::VEL] * GC::FT2M;
    
    _qIB =GU::Quaternion::quaternionFromEulerAngles321(_anglesWrtInertial);
    _qBI =GU::Quaternion::conjugate(_qIB);
    _xB = {GU::Quaternion::rotateVectorByQuaternion(_qIB, rInI), vInB, {0.0, 0.0, 0.0} };
    _xI = { rInI,GU::Quaternion::rotateVectorByQuaternion(_qBI, vInB), {0.0, 0.0, 0.0} };
    _xI0 = _xI;
}

void Frame::updateStates(const vector<double>& accelInBody) {
    _xB[GC::ACCEL] = accelInBody;
    _xB[GC::VEL] = accelInBody * _dt;
    vector<double> xI0VRot =GU::Quaternion::rotateVectorByQuaternion(_qIB, _xI0[GC::VEL]);
    _xB[GC::POS] = _xB[GC::POS] + xI0VRot * _dt + accelInBody * (0.5f * _dt * _dt);
}


void Frame::storeStatesInEnglishUnits(const vector<double>& r,
                                      const vector<double>& v,
                                      const vector<double>& a,
                                      const string& frame) {
    vector<vector<double>> states = { r * GC::M2FT, v * GC::M2FT, a * GC::M2FT };
    if (frame == "I") _stateHistInI.push_back(states);
    else if (frame == "B") _stateHistInB.push_back(states);
}

const vector<double>& Frame::getAnglesWrtInertial() const { return _anglesWrtInertial; }

vector<vector<double>> Frame::getInertialStates() const {
    vector<double> aInI =GU::Quaternion::rotateVectorByQuaternion(_qBI, _xB[GC::ACCEL]);
    return { _xI[GC::POS], _xI[GC::VEL], aInI };
}

vector<vector<double>> Frame::getBodyStates() const {
    vector<double> rB = _xB[GC::POS] + _xB[GC::VEL] * _dt;
    return { rB, _xB[GC::VEL], _xB[GC::ACCEL] };
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
    std::cout << "\u03A8: " << GC::RAD2DEG * _anglesWrtInertial[GC::Z] << " deg\n";
    std::cout << "\u03B8: " << GC::RAD2DEG * _anglesWrtInertial[GC::Y] << " deg\n";
    std::cout << "\u03D5: " << GC::RAD2DEG * _anglesWrtInertial[GC::X] << " deg\n\n";
}

void Frame::printStates() const {
    std::cout << "---------" << (_name.empty() ? "vehicleObj" : _name) << "---------\n";

    const auto& statesI = getInertialStates();
    const auto& r = statesI[GC::POS];
    // const auto& v = statesI[GC::VEL];
    // const auto& a = statesI[GC::ACCEL];

    const auto& statesB = getBodyStates();
    const auto& rB = statesB[GC::POS];
    const auto& vB = statesB[GC::VEL];
    const auto& aB = statesB[GC::ACCEL];

    std::cout << "\nrInI:\t";
    for (double val : r * GC::M2FT) std::cout << val << "\t";

    std::cout << "\nvInB:\t";
    for (double val : vB * GC::M2FT) std::cout << val << "\t";

    std::cout << "\naInB:\t";
    for (double val : aB * GC::M2FT) std::cout << val << "\t";

    std::cout << "\n";
    const char* names[3] = {"\u03D5", "\u03B8", "\u03A8"};
    for (int i = 1; i < 3; ++i)
        std::cout << names[i] << " (deg): " << GC::RAD2DEG * _anglesWrtInertial[i] << "\n";
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
        const auto& r = _stateHistInI[i][GC::POS];
        const auto& v = _stateHistInI[i][GC::VEL];
        const auto& a = _stateHistInI[i][GC::ACCEL];
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

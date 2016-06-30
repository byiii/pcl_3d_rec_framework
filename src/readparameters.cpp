#include <utils/readparameters.h>

//template void axisAngleToRotationMatrix(
//const Eigen::Matrix<double,3,1> &axis,
//double angle,
//Eigen::Matrix<double,3,3> &rotmat);

template void readParameters::get(const char* key, float& value);
template void readParameters::get(const char* key, int& value);
template void readParameters::get(const char* key, bool& value);
template void readParameters::get(const char* key, double& value);
template void readParameters::get(const char* key, std::string& value);



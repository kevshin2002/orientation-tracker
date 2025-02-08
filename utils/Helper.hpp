/*
* Author: Kevin Shin
* Project: IMU Estimation
* Copyright @ Kevin Shin, 2025
*
*
*/

#ifndef HELPER_HPP
#define HELPER_HPP

#include <iostream>
#include <cmath>
#include <iomanip>
#include <string>
#include <vector>
#include <array>
#include <Eigen/Dense>
#include "Data.hpp"

namespace ECE276A {
  constexpr double g = 9.81;

  enum class Value {
    SUCCESS = 0,
    ERROR = 1,
    TIMEOUT = 2
};

  struct Helper {
  static Quaternion multiplyQuat(const Quaternion aQ1, const Quaternion aQ2) {
    Quaternion result;
    
    result[0] = aQ1[0] * aQ2[0] - aQ1[1] * aQ2[1] - aQ1[2] * aQ2[2] - aQ1[3] * aQ2[3]; // w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    result[1] = aQ1[0] * aQ2[1] + aQ1[1] * aQ2[0] + aQ1[2] * aQ2[3] - aQ1[3] * aQ2[2]; // w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    result[2] = aQ1[0] * aQ2[2] - aQ1[1] * aQ2[3] + aQ1[2] * aQ2[0] + aQ1[3] * aQ2[1]; // w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    result[3] = aQ1[0] * aQ2[3] + aQ1[1] * aQ2[2] - aQ1[2] * aQ2[1] + aQ1[3] * aQ2[0]; // w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    
    return result;
}
  static Quaternion expQuat(const Quaternion aQuaternion){
    double w, x, y, z;
    w = aQuaternion[0];
    x = aQuaternion[1];
    y = aQuaternion[2];
    z = aQuaternion[3];

    double v_norm = sqrt(x * x + y * y + z * z);
    double e_w = std::exp(w);

    double cos_v = cos(v_norm);
    double sin_v = (v_norm > 1e-8) ? (sin(v_norm) / v_norm) : 1.0;
    
    return {e_w * cos_v, e_w * x * sin_v, e_w * y * sin_v, e_w * z * sin_v};
  }
  static Eulers toEuler(const Quaternions aQ) {
    Eulers angles;
    for(const auto& a : aQ){
    Euler angle;
    auto q = a.second;
    // Roll (x-axis rotation)
    double sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3]);
    double cosr_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2]);
    angle[0] = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    double sinp = 2 * (q[0] * q[2] - q[3] * q[1]);
    if (std::abs(sinp) >= 1) {
        angle[1] = std::copysign(M_PI / 2, sinp); // Use 90 degrees if out of range
    } else {
        angle[1] = std::asin(sinp);
    }

    // Yaw (z-axis rotation)
    double siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2]);
    double cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3]);
    angle[2] = std::atan2(siny_cosp, cosy_cosp);

    // Convert to degrees
    for (double& angle_component : angle) {
        angle_component = angle_component * 180 / M_PI;
    }
    angles.push_back(angle);
    }
    return angles;
}

  static Quaternion toInverse(Quaternion aQ) {
    double w = aQ[0];
    double x = aQ[1];
    double y = aQ[2];
    double z = aQ[3];

    double theNormSquared = (w * w) + (x * x) + (y * y) + (z * z);
  
    if(theNormSquared <= 0.0) {
      throw std::invalid_argument("Quaternion has a nonpositive norm, you screwed up somewhere.");
    }

    // Initialized with conjugate
    Quaternion theInverse = {w, -x, -y, -z}; 

    theInverse[0] = theInverse[0] / theNormSquared;
    theInverse[1] = theInverse[1] / theNormSquared;
    theInverse[2] = theInverse[2] / theNormSquared;
    theInverse[3] = theInverse[3] / theNormSquared;

    return theInverse;

  }

};

}
#endif // Helper.hpp


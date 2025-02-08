/*
* 
* Author: Kevin Shin
* Project: IMU Estimation
* Copyright @ Kevin Shin, 2025
*
*/


#ifndef DATA_HPP
#define DATA_HPP

#include <vector>
#include <array>

// retrospectively speaking, some of these should be structs and overloaded.
namespace ECE276A{

// For Vicon Dataset
using RotationRow = std::array<double, 3>;
using RotationMatrix = std::array<RotationRow, 3>;
using Rotation = std::pair<double, RotationMatrix>;
using Rotations = std::vector<Rotation>;
using Quaternion = std::array<double, 4>;
using Quaternions = std::vector<std::pair<double, Quaternion>>;

// For IMU Dataset
using IMUData = std::array<double, 6>;
using IMU = std::pair<double, IMUData>;
using IMUs = std::vector<IMU>;
using Acceleration = std::array<double, 3>;
using Accelerations = std::vector<Acceleration>;
using Gyro = std::array<double, 3>;
using Gyros = std::vector<Gyro>;


// Combined Dataset
using VIMU = std::pair<double, std::pair<IMUData, RotationMatrix>>;
using VIMUs = std::vector<VIMU>;

// Euler Angles
using Angle = std::vector<double>;
using Euler = std::array<double, 3>;
using Eulers = std::vector<Euler>;
}

#endif // Data.hpp

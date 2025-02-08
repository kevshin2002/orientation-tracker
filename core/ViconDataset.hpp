/*
* Author: Kevin Shin
* Project: IMU Estimation
* Copyright @ Kevin Shin, 2025
*
*
*/

#ifndef VICONDATASET_HPP
#define VICONDATASET_HPP

#include <string>
#include <vector>
#include <array>
#include <stdexcept>
#include "Data.hpp"
#include "csv2/csv2.hpp"

namespace ECE276A {
class ViconDataset {
  public:
    ViconDataset(const std::string& aFilePath);
    ~ViconDataset();
   
    Rotation&                         operator[](size_t theIndex);
    
    size_t                            getNumTimeSteps() const;
    Rotation                          getRotation(size_t aTimeStep) const;
    Rotations                         getRotations() const { return rotations;}
    Quaternions                       getQuaternions() const { return quaternions; }
    
    void                              viewRotation(size_t aTimeStep) const;

    Rotation                          toRotation(const std::pair<double, Quaternion>& aQuaternions) const;
    std::pair<double, Quaternion>     toQuaternion(const Rotation& aRotation) const;

  protected:
    bool                              isRotation(const Rotation& aRotation) const;

  private:
    Rotations                         rotations;
    Quaternions                       quaternions;
  };
}

#endif // ViconDataset.hpp

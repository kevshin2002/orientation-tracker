
#include "ViconDataset.hpp"
#include <fstream>
#include <iostream>
#include <cmath>
#include <iomanip>

namespace ECE276A {

ViconDataset::ViconDataset(const std::string& filePath) {
  csv2::Reader<csv2::delimiter<','>, 
               csv2::quote_character<'"'>, 
               csv2::first_row_is_header<true>,
               csv2::trim_policy::trim_whitespace> theCSV;

  if (!theCSV.mmap(filePath)) {
    return;
  }

  for (const auto& theRow : theCSV) {
    Rotation rotation;
    size_t theIndex = 0;

    for (const auto& theCell : theRow) {         
      std::string theValue;
      theCell.read_value(theValue);
      
      if (theIndex == 0) {
        rotation.first = std::stod(theValue);  // First column is the timestamp
      } else {
        size_t rowIndex = (theIndex - 1) / 3;
        size_t colIndex = (theIndex - 1) % 3;
        rotation.second[rowIndex][colIndex] = std::stod(theValue);
      }
      theIndex++;
    }

    if (!rotation.second.empty()) {
      rotations.push_back(rotation);
      if (isRotation(rotation)) {
        quaternions.push_back(toQuaternion(rotation));
      }
    }
  }

  if (!rotations.empty()) {
    rotations.pop_back();  // Last entry is empty row.
  }
}

ViconDataset::~ViconDataset(){}

Rotation& ViconDataset::operator[](size_t theIndex) {
  if (theIndex >= 10) throw std::out_of_range("Index out of range");
  return rotations[theIndex];
}
  
Rotation ViconDataset::getRotation(size_t aTimeStep) const {
  if (aTimeStep >= rotations.size()) {
      throw std::out_of_range("Time step " + std::to_string(aTimeStep) + 
                              " is out of range. Total time steps: " + 
                              std::to_string(rotations.size()));
    }

    return rotations[aTimeStep];  
}

void ViconDataset::viewRotation(size_t aTimeStep) const {
    auto theRotation = getRotation(aTimeStep);
    auto theTimeStep = theRotation.first;
    auto theRotationMatrix = theRotation.second;

    std::cout << "Rotation Matrix at time: " + std::to_string(theTimeStep) << "\n";

    for (const auto& theRow : theRotationMatrix) {
      for (double theValue : theRow) {
        std::cout << std::fixed << std::setprecision(17) << std::setw(25) << theValue << " ";
      }
      std::cout << "\n";
    }
  }

size_t ViconDataset::getNumTimeSteps() const {
    return rotations.size() - 1;
}

std::pair<double, Quaternion> ViconDataset::toQuaternion(const Rotation& aRotation) const {
    if (!isRotation(aRotation)) {
        throw std::runtime_error("Input matrix is not a valid rotation matrix.");
    }

    double w, x, y, z;
    double theTimeStep = aRotation.first;
    auto aRotationMatrix = aRotation.second;
    double trace = aRotationMatrix[0][0] + aRotationMatrix[1][1] + aRotationMatrix[2][2];

    if (trace > 0) {
        double s = 0.5 / std::sqrt(trace + 1.0);
        w = 0.25 / s;
        x = (aRotationMatrix[2][1] - aRotationMatrix[1][2]) * s;
        y = (aRotationMatrix[0][2] - aRotationMatrix[2][0]) * s;
        z = (aRotationMatrix[1][0] - aRotationMatrix[0][1]) * s;
    } else if (aRotationMatrix[0][0] > aRotationMatrix[1][1] && aRotationMatrix[0][0] > aRotationMatrix[2][2]) {
        double s = 2.0 * std::sqrt(1.0 + aRotationMatrix[0][0] - aRotationMatrix[1][1] - aRotationMatrix[2][2]);
        w = (aRotationMatrix[2][1] - aRotationMatrix[1][2]) / s;
        x = 0.25 * s;
        y = (aRotationMatrix[0][1] + aRotationMatrix[1][0]) / s;
        z = (aRotationMatrix[0][2] + aRotationMatrix[2][0]) / s;
    } else if (aRotationMatrix[1][1] > aRotationMatrix[2][2]) {
        double s = 2.0 * std::sqrt(1.0 + aRotationMatrix[1][1] - aRotationMatrix[0][0] - aRotationMatrix[2][2]);
        w = (aRotationMatrix[0][2] - aRotationMatrix[2][0]) / s;
        x = (aRotationMatrix[0][1] + aRotationMatrix[1][0]) / s;
        y = 0.25 * s;
        z = (aRotationMatrix[1][2] + aRotationMatrix[2][1]) / s;
    } else {
        double s = 2.0 * std::sqrt(1.0 + aRotationMatrix[2][2] - aRotationMatrix[0][0] - aRotationMatrix[1][1]);
        w = (aRotationMatrix[1][0] - aRotationMatrix[0][1]) / s;
        x = (aRotationMatrix[0][2] + aRotationMatrix[2][0]) / s;
        y = (aRotationMatrix[1][2] + aRotationMatrix[2][1]) / s;
        z = 0.25 * s;
    }

    // Normalize the quaternion (optional, but recommended for robustness)
    double norm = std::sqrt(w * w + x * x + y * y + z * z);
    w /= norm;
    x /= norm;
    y /= norm;
    z /= norm;

    std::array<double, 4> theQuaternion{ w, x, y, z };

    return std::make_pair(theTimeStep, theQuaternion);
}

  Rotation ViconDataset::toRotation(const std::pair<double, Quaternion>& aQuaternion) const {
    double w = aQuaternion.second[0];
    double x = aQuaternion.second[1];
    double y = aQuaternion.second[2];
    double z = aQuaternion.second[3];
    
    double theTime = aQuaternion.first;
    RotationMatrix theMatrix;
    theMatrix[0][0] = 1 - 2 * (y * y + z * z);
    theMatrix[0][1] = 2 * (x * y - z * w);
    theMatrix[0][2] = 2 * (x * z + y * w);

    theMatrix[1][0] = 2 * (x * y + z * w);
    theMatrix[1][1] = 1 - 2 * (x * x + z * z);
    theMatrix[1][2] = 2 * (y * z - x * w);

    theMatrix[2][0] = 2 * (x * z - y * w);
    theMatrix[2][1] = 2 * (y * z + x * w);
    theMatrix[2][2] = 1 - 2 * (x * x + y * y);

    return std::make_pair(theTime, theMatrix);
  }

  bool ViconDataset::isRotation(const Rotation& aRotation) const {
    constexpr double theEpsilon = 1e-6;
    
    auto aMatrix = aRotation.second;
    for (int i = 0; i < 3; ++i) {
      double theNorm = 0.0;
      for (int k = 0; k < 3; ++k) {
        theNorm += aMatrix[k][i] * aMatrix[k][i];
      }
      
      if (std::abs(theNorm - 1.0) > theEpsilon) return false; 

      for (int j = i + 1; j < 3; ++j) {
        double theDot = 0.0;
        for (int k = 0; k < 3; ++k) {
          theDot += aMatrix[k][i] * aMatrix[k][j];
        }
        
        if (std::abs(theDot) > theEpsilon) return false;
      
      }
    }

    double theDet = aMatrix[0][0] * (aMatrix[1][1] * aMatrix[2][2] - aMatrix[1][2] * aMatrix[2][1]) -
                 aMatrix[0][1] * (aMatrix[1][0] * aMatrix[2][2] - aMatrix[1][2] * aMatrix[2][0]) +
                 aMatrix[0][2] * (aMatrix[1][0] * aMatrix[2][1] - aMatrix[1][1] * aMatrix[2][0]);

    return std::abs(theDet - 1.0) <= theEpsilon;
  
  }


}




/*
* Author: Kevin Shin
* Project: IMU Estimation
* Copyright @ Kevin Shin, 2025
*/

#include "IMUDataset.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <cmath>

namespace ECE276A {
IMUDataset::IMUDataset(const std::string& aFilePath) : file(aFilePath) {

  csv2::Reader<csv2::delimiter<','>, 
                 csv2::quote_character<'"'>, 
                 csv2::first_row_is_header<true>,
                 csv2::trim_policy::trim_whitespace> theCSV;

  bool theFirst = true;
  std::string theValue = "";
  size_t theCount = 0;

  if (theCSV.mmap(file)) {
    for (const auto& theRow : theCSV) {
      bool theFirst = true;
      IMU theIMU;

      for(const auto& theCell : theRow) {         
        std::string theValue;
        theCell.read_value(theValue);

        if(theFirst){
          theFirst = false;
          theIMU.first = std::stod(theValue);
          continue;
        }
                
        if(theCount == 6){
          theCount = 0;
        }
        theIMU.second[theCount] = std::stod(theValue);
        theCount++;
      }
      imus.push_back(theIMU);
    }
  }

  calculateBiases().calculateSF().process();

}

IMUDataset::~IMUDataset(){}

IMU IMUDataset::getReadings(size_t aTimeStep) const {
  if (aTimeStep >= imus.size()) {
    throw std::out_of_range("Time step " + std::to_string(aTimeStep) + 
                            " is out of range. Total time steps: " + 
                            std::to_string(imus.size()));
  }

  return imus[aTimeStep];  
}

void IMUDataset::viewReadings(size_t aTimeStep) const {
  auto theIMU = getReadings(aTimeStep);
  auto theTimeStep = theIMU.first;
  auto theData = theIMU.second;
  
  std::cout << std::fixed << std::setprecision(6);
  std::cout << std::left << std::setw(10) << "Time" << " : " << std::right << theTimeStep << "\n";
  std::cout << std::left << std::setw(10) << "Ax" << " : " << std::right << theData[0] << "\n";
  std::cout << std::left << std::setw(10) << "Ay" << " : " << std::right << theData[1] << "\n";
  std::cout << std::left << std::setw(10) << "Az" << " : " << std::right << theData[2] << "\n";
  std::cout << std::left << std::setw(10) << "Wx" << " : " << std::right << theData[3] << "\n";
  std::cout << std::left << std::setw(10) << "Wy" << " : " << std::right << theData[4] << "\n";
  std::cout << std::left << std::setw(10) << "Wz" << " : " << std::right << theData[5] << "\n";

}

size_t IMUDataset::getNumTimeSteps() const {
    return imus.size() - 1;
}

IMUDataset& IMUDataset::calculateBiases(){
  size_t theStaticCnt = getStaticPoints();
  std::array<double, 6> theSums = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  IMUData theData;

  for (size_t theCount = 0; theCount < theStaticCnt; ++theCount) {
    theData = imus[theCount].second;
    // acceleration
    theSums[0] += theData[0];
    theSums[1] += theData[1];
    theSums[2] += theData[2];

    // gyroscope
    theSums[3] += theData[3];
    theSums[4] += theData[4];
    theSums[5] += theData[5];
  }

  biases[0] = (theSums[0] / theStaticCnt);
  biases[1] = (theSums[1] / theStaticCnt);
  biases[2] = (theSums[2] / theStaticCnt) - 100;
  biases[3] = (theSums[3] / theStaticCnt);
  biases[4] = (theSums[4] / theStaticCnt);
  biases[5] = (theSums[5] / theStaticCnt);

  //std::cout << biases[0] << " " << biases[1] << " " << biases[2] << " " << biases[3] << " " << biases[4] << " " << biases[5];

  return *this;
}

IMUDataset& IMUDataset::calculateSF() {
  double gyro_sens = 3.330 * (180 / M_PI); // mV/ degree / s (4x)
  double accel_sens = 300; // sensitivity (typical) * g

  scales[0] = (3300 / 1023) / accel_sens;
  scales[1] = (3300 / 1023) / gyro_sens;

   return *this;
}

IMUDataset& IMUDataset::process() {
   for(auto& theIMU : imus){
    // accel
    theIMU.second[0] = ((theIMU.second[0] - biases[0]) * scales[0]);
    theIMU.second[1] = ((theIMU.second[1] - biases[1]) * scales[0]);
    theIMU.second[2] = ((theIMU.second[2] - biases[2]) * scales[0]);
    acceleration.push_back({theIMU.second[0], theIMU.second[1], theIMU.second[2]});
    // gyros
    theIMU.second[3] = ((theIMU.second[3] - biases[3]) * scales[1]);
    theIMU.second[4] = ((theIMU.second[4] - biases[4]) * scales[1]);
    theIMU.second[5] = ((theIMU.second[5] - biases[5]) * scales[1]);
    gyros.push_back({theIMU.second[3], theIMU.second[4], theIMU.second[5]});
  }
  
  return *this;
}

size_t IMUDataset::getStaticPoints() {
  size_t theCount = 1;
  double rawBias = (1.23 / 3.3 ) * 1023; // Zoff / Zreff * 2^10 - 1 (10 bits for ADC)
  double wx, wy, wz;
  size_t theSize = imus.size();
   
  for(const auto& theReading : imus) {
    wx = theReading.second[3] - rawBias;
    wy = theReading.second[4] - rawBias;
    wz = theReading.second[5] - rawBias;

    if (std::abs(wx) < 10 && std::abs(wy) < 10 && std::abs(wz) < 10) {
      theCount++;
    } else {
      break;
    }
  }
  return theCount;
}
}

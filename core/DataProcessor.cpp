
/*
* Author: Kevin Shin
* Project: IMU Estimation
* Copyright @ Kevin Shin, 2025
*/

#include "DataProcessor.hpp"

namespace ECE276A {

DataProcessor::DataProcessor(IMUDataset& anIMUDataset, ViconDataset& aViconDataset) : imu(anIMUDataset), vicon(aViconDataset) {
auto theRotations = vicon.getRotations();
auto theIMUs = imu.getIMUs();

std::sort(theRotations.begin(), theRotations.end(), [](const Rotation& a, const Rotation& b) {
  return a.first < b.first;
});

combine(theIMUs, theRotations);

}

DataProcessor::~DataProcessor() {}

DataProcessor& DataProcessor::toModel() {

  size_t theSize = data.size();
  VIMU theCurrent, theNext;
  IMUData theCurrentData, theNextData;
  
  double theDifference = 0;
  Quaternion theQuaternion{1,0,0,0};
  Quaternion theExponential{0,0,0,0};

  for(size_t theIndex = 0; theIndex <= theSize - 1; theIndex++){
      theCurrent = data[theIndex];
      theNext = data[theIndex + 1];
      
      theCurrentData = theCurrent.second.first;
      theNextData = theNext.second.first;

      theDifference = (theNext.first - theCurrent.first) / 2;
      theExponential[0] = 0;
      theExponential[1] = theDifference * theCurrentData[3];
      theExponential[2] = theDifference * theCurrentData[4];
      theExponential[3] = theDifference * theCurrentData[5];
      
      theExponential = Helper::expQuat(theExponential);
      theQuaternion = Helper::multiplyQuat(theQuaternion, theExponential); // motion model
      model.push_back(std::make_pair(theDifference, theQuaternion));
  }
  
  return *this;
}

DataProcessor& DataProcessor::toObserve() {
  Quaternion theQuaternion, theInverse;
  Quaternion theRotation = {0, 0, 0, 1};
  for(const auto theMotions : model){
    theInverse = Helper::toInverse(theMotions.second);
    theQuaternion = Helper::multiplyQuat(theInverse, theRotation);
    theQuaternion = Helper::multiplyQuat(theQuaternion, theMotions.second);
    observation.push_back({theQuaternion[1], theQuaternion[2], theQuaternion[3]});
  
  }
  return *this;
}
void DataProcessor::viewData() {
  size_t theSize = data.size();
  size_t theCount = 0;  

  while(theCount != theSize){
    viewData(theCount);
    theCount++;
  }
}
  
void DataProcessor::viewData(size_t aTime) {
  auto vimu = data[aTime];
  std::cout << "Timestamp: " << std::fixed << std::setprecision(6) << vimu.first << "\n";
  std::cout << "IMU Data: ";
  for (const auto& val : vimu.second.first) {
    std::cout << std::setw(10) << std::fixed << std::setprecision(6) << val << " ";
  }
  std::cout << "\n";
    
  std::cout << "Rotation Matrix: \n";
  for (const auto& row : vimu.second.second) {
    for (const auto& val : row) {
      std::cout << std::setw(10) << std::fixed << std::setprecision(6) << val << " ";
    }
    std::cout << "\n";
  }
  std::cout << "\n";
}

const Rotation& DataProcessor::findVICON(const Rotations& aRotations, double aTimeStamp) {
  auto it = std::lower_bound(aRotations.begin(), aRotations.end(), aTimeStamp,
                             [](const Rotation& theRotation, double theTimeStamp) {
                             return theRotation.first < theTimeStamp;
                             });
  
  if (it == aRotations.begin()) {
    return *it;
  } else if (it == aRotations.end()) {
    return *(it - 1);
  } else {
    double diff1 = std::abs(it->first - aTimeStamp);
    double diff2 = std::abs((it - 1)->first - aTimeStamp);
    return (diff1 < diff2) ? *it : *(it - 1);
  }
}

DataProcessor&  DataProcessor::combine(const IMUs& aIMUs, const Rotations& aRotations) {
  data.reserve(aIMUs.size());

  for (const auto& imu : aIMUs) {
    const Rotation& closestVICON = findVICON(aRotations, imu.first);
    data.emplace_back(imu.first, std::make_pair(imu.second, closestVICON.second));
  }
  return *this;
}
  
}

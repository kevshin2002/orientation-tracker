
/*
* Author: Kevin Shin
* Project: IMU Estmiation
* Copyright @ Kevin Shin, 2025
*/

#ifndef DATAPROCESSOR_HPP
#define DATAPROCESSOR_HPP

#include "Data.hpp"
#include "ViconDataset.hpp"
#include "IMUDataset.hpp"
#include "Helper.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>

namespace ECE276A{

  class DataProcessor{
    public:
      DataProcessor(IMUDataset& anIMUDataset, ViconDataset& aViconDataset);
      ~DataProcessor();
   
      DataProcessor&        toModel();
      DataProcessor&        toObserve();
      void                  viewData();
      void                  viewData(size_t aTime);

      Quaternions           getModel() const { return model; }
      Accelerations         getObservation() const { return observation; }
    protected:
      const Rotation&       findVICON(const Rotations& aRotations, double aTimeStamp);
            DataProcessor&  combine(const IMUs& aIMUs, const Rotations& aRotations);
    
    private:
      IMUDataset            imu;
      ViconDataset          vicon;
      VIMUs                 data; 

      Quaternions           model;
      Accelerations         observation;
  };

}

#endif // DataProcessor.hpp


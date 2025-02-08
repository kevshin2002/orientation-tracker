
/*
* Author: Kevin Shin
* Project: IMU Estimation
* Copyright @ Kevin Shin, 2025
*
*
*/

#ifndef IMUDATASET_HPP
#define IMUDATASET_HPP

#include <string>
#include <vector>
#include <array>
#include "Data.hpp"
#include "csv2/csv2.hpp"

namespace ECE276A {

class IMUDataset {
  public:
    IMUDataset(const std::string& aFilePath);
    ~IMUDataset();
  
  /*
  * IMUDataset is calibrated when instantiated.
  * getReadings returns std::pair<double, std::array<double, 6>>
  * getNumTimeSteps returns the number of data
  * viewReadings prints out the above in a nicely format
  */

    IMUs                    getIMUs() const { return imus;}
    Accelerations           getAccelerations() const { return acceleration; }
    Gyros                   getGyros() const { return gyros; }

    IMU                     getReadings(size_t aTimeStep) const;
    size_t                  getNumTimeSteps() const;

    void                    viewReadings(size_t aTimeStep) const;

  protected:
    IMUDataset&             process();
    IMUDataset&             calculateBiases();
    IMUDataset&             calculateSF();

    size_t                  getStaticPoints();

  private:
    std::string             file;  
    IMUs                    imus;
    Accelerations           acceleration;
    Gyros                   gyros;

    std::array<double, 6>   biases;
    std::array<double, 2>   scales;

  };
}

#endif // IMUDataset.hpp



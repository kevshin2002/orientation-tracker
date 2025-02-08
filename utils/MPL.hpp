/*
 * Author: Kevin Shin
 * Project: IMU Estimation
 * Copyright @ Kevin Shin, 2025
 */

#ifndef MPL_HPP
#define MPL_HPP

#include "matplotlibcpp.h"
#include "Helper.hpp"
#include <vector>

namespace plt = matplotlibcpp;

namespace ECE276A {

struct MPL {
  static void normalize(const Quaternions& quats, Angle& roll, Angle& pitch, Angle& yaw, bool state) {
    double prev_roll = 0, prev_pitch = 0, prev_yaw = 0;
    double current_roll = 0, current_pitch = 0, current_yaw = 0;
    auto euler_angles = Helper::toEuler(quats);
    if(state){
      for (const auto& angles : euler_angles) {
        current_roll = angles[0];
        current_pitch = angles[1];
        current_yaw = angles[2];

        roll.push_back(current_roll);
        pitch.push_back(current_pitch);
        yaw.push_back(current_yaw);

        prev_roll = current_roll;
        prev_pitch = current_pitch;
        prev_yaw = current_yaw;
      }
    }
    else{
      for (const auto& angles : euler_angles) {
        current_roll = angles[0];
        current_pitch = angles[1];
        current_yaw = angles[2];

        roll.push_back(current_roll);
        pitch.push_back(current_pitch);
        yaw.push_back(current_yaw);
      }
    }
  }

static void plotAngles(const std::vector<double>& x, const Angle aEstimate, const Angle aTruth, const std::string& aTitle, const std::string& aLabel) {
    plt::figure();
    plt::named_plot("Estimation", x, aEstimate);
    plt::named_plot("Ground Truth", x, aTruth);
    plt::xlabel("Time Step");
    plt::ylabel(aLabel);
    plt::title(aTitle + " Comparison");
    plt::legend();
  }


static void plot(const Quaternions& aQ1, const Quaternions& aQ2) {
    std::vector<double> roll1, pitch1, yaw1;
    std::vector<double> roll2, pitch2, yaw2;

    normalize(aQ1, roll1, pitch1, yaw1, true);
    normalize(aQ2, roll2, pitch2, yaw2, true);
    
    std::vector<double> x(aQ2.size());
    size_t theSize = x.size();
    for (size_t i = 0; i < theSize; ++i) {
      x[i] = i;
    }
    
    auto resize_vectors = [theSize](auto&... vectors) {
      (vectors.resize(theSize), ...);
    };

    resize_vectors(roll1, roll2, pitch1, pitch2, yaw1, yaw2); 
    
    plotAngles(x, roll1, roll2, "Roll", "Angle (radians)");
    plotAngles(x, pitch1, pitch2, "Pitch", "Angle (radians)");
    plotAngles(x, yaw1, yaw2, "Yaw", "Angle (radians)");
    
    plt::show();
  }

  static void plotAngles(const std::vector<double>& x, const Angle aEstimate, const std::string& aTitle, const std::string& aLabel) {
    plt::figure();
    plt::named_plot("Estimation", x, aEstimate);
    plt::xlabel("Time Step");
    plt::ylabel(aLabel);
    plt::title(aTitle);
    plt::legend();
  }

  static void plot(const Quaternions& aQ1) {
    std::vector<double> roll1, pitch1, yaw1;
            
    normalize(aQ1, roll1, pitch1, yaw1, true);

    std::vector<double> x(aQ1.size());
    size_t theSize = x.size();
    for (size_t i = 0; i < theSize; ++i) {
      x[i] = i;
    }
    
    plotAngles(x, roll1, "Roll", "Angle (radians)");
    plotAngles(x, pitch1, "Pitch", "Angle (radians)");
    plotAngles(x, yaw1, "Yaw", "Angle (radians)");
    plt::show();
  }

  static void plotIMUs(const std::vector<double>& x, const std::vector<double>& aReadings, const std::vector<double>& anObservation, const std::string& aTitle, const std::string& aLabel) {
    plt::figure();
    plt::named_plot("Acceleration Readings", x, aReadings);
    plt::named_plot("Observation Model", x, anObservation);
    plt::xlabel("Time Step");
    plt::ylabel(aLabel);
    plt::title(aTitle + " Comparison");
    plt::legend();
  }

   static void plotIMUs(const std::vector<double>& x, const std::vector<double>& aReadings, const std::string& aTitle, const std::string& aLabel) {
    plt::figure();
    plt::named_plot("Acceleration Readings", x, aReadings);
    plt::xlabel("Time Step");
    plt::ylabel(aLabel);
    plt::title(aTitle);
    plt::legend();
  }


  static void plotGyros(const std::vector<double>& x, const std::vector<double>& aReadings, const std::string& aTitle, const std::string& aLabel) {
    plt::figure();
    plt::named_plot("Gyro Readings", x, aReadings);
    plt::xlabel("Time Step");
    plt::ylabel(aLabel);
    plt::title(aTitle);
    plt::legend();
  }

  static void plotG(const Gyros& anG){
    std::vector<double> gyros_x, gyros_y, gyros_z;
    
    std::vector<double> x(anG.size());
    size_t theSize = x.size();
    for(size_t i = 0; i < theSize; ++i){
      x[i] = i;
    }

    for(const auto& theReadings : anG){
      gyros_x.push_back(theReadings[0]);
      gyros_y.push_back(theReadings[1]);
      gyros_z.push_back(theReadings[2]);
    }

    plotGyros(x, gyros_x, "X Angular Velocity", "Angles (Degrees) ");
    plotGyros(x, gyros_y, "Y Angular Velocity", "Angles (Degrees) ");
    plotGyros(x, gyros_z, "Z Angular Velocity", "Angles (Degrees) ");

  }
  static void plot(const Accelerations& anA1, const Accelerations& anA2){
    std::vector<double> accel_x_1, accel_y_1, accel_z_1;
    std::vector<double> accel_x_2, accel_y_2, accel_z_2;
  
    std::vector<double> x(anA2.size());
    size_t theSize = x.size();

    for(size_t i = 0; i < theSize; ++i){
      x[i] = i;
    }

    for(const auto& theReadings : anA1){
      accel_x_1.push_back(theReadings[0]);
      accel_y_1.push_back(theReadings[1]);
      accel_z_1.push_back(theReadings[2]);
    }

    for(const auto& theObservations : anA2){
      accel_x_2.push_back(theObservations[0]);
      accel_y_2.push_back(theObservations[1]);
      accel_z_2.push_back(theObservations[2]);
    }
    
    auto resize_vectors = [theSize](auto&... vectors) {
            (vectors.resize(theSize), ...);
    };

    resize_vectors(accel_x_1, accel_x_2, accel_y_1, accel_y_2, accel_z_1, accel_z_2);

    plotIMUs(x, accel_x_1, accel_x_2, "X Acceleration", "mV/g");
    plotIMUs(x, accel_y_1, accel_y_2, "Y Acceleration", "mV/g");
    plotIMUs(x, accel_z_1, accel_z_2, "Z Acceleration", "mV/g");

    plt::show();
  }

  static void plotA(const Accelerations& anA1){
    std::vector<double> accel_x_1, accel_y_1, accel_z_1;
    std::vector<double> x(anA1.size());
    size_t theSize = x.size();

    for(size_t i = 0; i < theSize; ++i){
      x[i] = i;
    }

     for(const auto& theReadings : anA1){
      accel_x_1.push_back(theReadings[0]);
      accel_y_1.push_back(theReadings[1]);
      accel_z_1.push_back(theReadings[2]);
    }

    plotIMUs(x, accel_x_1, "X Acceleration", "mV/g");
    plotIMUs(x, accel_y_1, "Y Acceleration", "mV/g");
    plotIMUs(x, accel_z_1, "Z Acceleration", "mV/g");

    plt::show();
  }
};

}

#endif // MPL_HPP

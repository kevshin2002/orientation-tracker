/*
* Author: Kevin Shin
* Project: IMU Estmiation
* Copyright @ Kevin Shin, 2025
*/

#ifndef BRIDGE_HPP
#define BRIDGE_HPP

#include "Data.hpp"
#include "Helper.hpp"
#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <pybind11/embed.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>   

namespace py = pybind11;

namespace ECE276A {

    class __attribute__((visibility("hidden"))) Bridge {
    public:
        Bridge(Quaternions& aEstimate, Quaternions& aTruth, Gyros& aGyro, Accelerations& anAccelerations, std::string& aCam, double aStep, py::module& jax_module);
        ~Bridge();
        
        Bridge& train();
        Bridge& sketch(bool aVicon); 
    private:
        double step_size;
        
        Accelerations&  theAccelerations;
        Accelerations&  theObservation;
  
        Quaternions&   theTruth;
        Quaternions&   theEstimate;
        
      
        std::string theCam; 
        py::module jax_module;
    };

}

#endif // Bridge_HPP

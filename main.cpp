
#include "ViconDataset.hpp"
#include "IMUDataset.hpp"
#include "Bridge.hpp"
#include "DataProcessor.hpp"
#include "MPL.hpp"
#include "Helper.hpp"
#include <iostream>
#include <string>

int main() {
    py::scoped_interpreter guard{};
    py::module sys = py::module::import("sys");
    sys.attr("path").attr("append")("core");
    py::module jax_module = py::module::import("bridge");

    // Data files, could be abstracted away, but it works.
    std::string vicon_file  = "data/csv/vicon/viconRot1.csv";
    std::string imu_file    = "data/csv/imu/imuRaw1.csv";
    std::string theCam      = "data/cam/cam1.p";

    ECE276A::ViconDataset   theVICONDataset(vicon_file);
    ECE276A::IMUDataset     theIMUDataset(imu_file);
    ECE276A::DataProcessor  theProcessor(theIMUDataset, theVICONDataset); 
    
    theProcessor.toModel().toObserve();

    ECE276A::Quaternions   theTruth         =   theVICONDataset.getQuaternions();
    ECE276A::Quaternions   theEstimate      =   theProcessor.getModel();
    ECE276A::Accelerations theObservation   =   theProcessor.getObservation();
    ECE276A::Accelerations theAccelerations =   theIMUDataset.getAccelerations();
    ECE276A::Gyros         theGyros         =   theIMUDataset.getGyros();
     
    ECE276A::MPL::plotG(theGyros); // Angular Velocities
    ECE276A::MPL::plotA(theAccelerations); // Accelerations
    ECE276A::MPL::plot(theObservation, theAccelerations); // Observation Model
    ECE276A::MPL::plot(theEstimate, theTruth); // Before optimizations  
    ECE276A::Bridge theBridge(theEstimate, theTruth, theAccelerations, theObservation, theCam, 1e-3, jax_module);
    theBridge.train();
    
    ECE276A::MPL::plot(theEstimate, theTruth); // After Optimization
    // ECE276A::MPL::plot(theEstimate); For no training set and only test set.
    //theBridge.sketch(true); // For panorama, but not implemented.

    return 0;
}

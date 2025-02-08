#include "Bridge.hpp"
#include <iostream>
#include <vector>

namespace py = pybind11;

std::unique_ptr<py::scoped_interpreter> guard;

namespace ECE276A {
Bridge::Bridge(Quaternions& anEstimate, Quaternions& aTruth, Accelerations& anAccelerations, Accelerations& aObservation, std::string& aCam, double aStep, py::module& jax_module) 
    : theEstimate(anEstimate), theTruth(aTruth), theAccelerations(anAccelerations), theObservation(aObservation), theCam(aCam), step_size(aStep), jax_module(jax_module) {}
Bridge::~Bridge() {}
Bridge& Bridge::train() {

  size_t min_size = std::min({
    theEstimate.size(),
    theTruth.size(),
    theAccelerations.size(),
    theObservation.size()
});

std::vector<double> q_data;
for (size_t i = 0; i < min_size; ++i) {
    q_data.insert(q_data.end(), theEstimate[i].second.begin(), theEstimate[i].second.end());
}

std::vector<size_t> q_shape = {min_size, 4};
py::array_t<double> q_array(q_shape, q_data.data());

std::vector<double> f_data;
for (size_t i = 0; i < min_size; ++i) {
    f_data.insert(f_data.end(), theTruth[i].second.begin(), theTruth[i].second.end());
}

std::vector<size_t> f_shape = {min_size, 4};
py::array_t<double> f_array(f_shape, f_data.data());


std::vector<double> a_data;
for (size_t i = 0; i < min_size; ++i) {
    a_data.insert(a_data.end(), theAccelerations[i].begin(), theAccelerations[i].end());
}

std::vector<size_t> a_shape = {min_size, 3};
py::array_t<double> a_array(a_shape, a_data.data());

std::vector<double> h_data;
for (size_t i = 0; i < min_size; ++i) {
    h_data.insert(h_data.end(), theObservation[i].begin(), theObservation[i].end());
}

std::vector<size_t> h_shape = {min_size, 3};
py::array_t<double> h_array(h_shape, h_data.data());


  
    try {
        py::array_t<double> updated_array = jax_module.attr("train")(f_array, q_array, a_array, h_array, step_size);
        auto buf = updated_array.request();
        double* ptr = static_cast<double*>(buf.ptr);
        std::vector<double> updated_data(ptr, ptr + buf.size);

        if (updated_data.size() != min_size * 4) {
            std::cerr << "Error: Updated data size mismatch. Expected " << min_size * 4 << " but got " << updated_data.size() << std::endl;
            return *this;
        }
        for (size_t i = 0; i < min_size; ++i) {
            std::copy(updated_data.begin() + i * 4, updated_data.begin() + (i + 1) * 4, theEstimate[i].second.begin());
        }
        
        std::cout << "Training completed." << std::endl;
    }
    catch (const py::error_already_set& e) {
        std::cerr << "Python error occurred during train execution: " << e.what() << std::endl;
    }

    return *this;
}

Bridge& Bridge::sketch(bool aVicon = false) {
  if(aVicon){
      jax_module.attr("sketch")(theCam, theTruth);
  }
  else {
      jax_module.attr("sketch")(theCam, theEstimate);
  }
    return *this;
}

}


#include "neblib/inertial_wrapper.hpp"

neblib::IMUWrapper::IMUWrapper(): IMU(vex::inertial(vex::PORT1)), exists(false) {}

neblib::IMUWrapper::IMUWrapper(vex::inertial &&IMU): IMU(IMU), exists(true) {}
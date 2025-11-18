#include "pros/motors.hpp" // IWYU pragma: keep
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp" // IWYU pragma: keep
#include "liblvgl/widgets/lv_label.h" // IWYU pragma: keep
#include "pros/abstract_motor.hpp" // IWYU pragma: keep
#include "pros/adi.h" // IWYU pragma: keep
#include "pros/adi.hpp" // IWYU pragma: keep
#include "liblvgl/llemu.hpp" // IWYU pragma: keep
#include "pros/apix.h" // IWYU pragma: keep
#include "pros/colors.hpp" // IWYU pragma: keep
#include "pros/device.hpp" // IWYU pragma: keep
#include "pros/misc.h" // IWYU pragma: keep
#include "pros/misc.hpp" // IWYU pragma: keep
#include "pros/motors.h" // IWYU pragma: keep
#include "pros/optical.h" // IWYU pragma: keep
#include "pros/optical.hpp" // IWYU pragma: keep
#include <algorithm> // IWYU pragma: keep
#include <cmath> // IWYU pragma: keep
#include <thread> // IWYU pragma: keep
#pragma once

extern pros::Controller Controller;

extern pros::MotorGroup left_mg;
extern pros::MotorGroup right_mg;
extern pros::Motor intake;
extern pros::Motor unstore;
extern pros::Motor store;
extern pros::adi::DigitalOut matchLoad;
extern pros::adi::DigitalOut midScore;
extern pros::adi::DigitalOut storage;

extern pros::Imu imu;
extern pros::Rotation vertical_sensor;
extern pros::Optical colorSensor;
extern lemlib::TrackingWheel vertical_tracker;
extern lemlib::OdomSensors sensors;

extern lemlib::ControllerSettings lateral_controller;
extern lemlib::ControllerSettings angular_controller;

extern lemlib::Chassis chassis;

extern bool oppStore;
extern bool middleScore ;
extern bool matchLoadDown;
extern bool sortToggle;
extern bool sortColorToggle;
extern bool scoring;
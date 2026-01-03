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
#include "pros/misc.h" // IWYU pragma: keep
#include "pros/rtos.hpp" // IWYU pragma: keep
#include <algorithm> // IWYU pragma: keep
#include <cmath> // IWYU pragma: keep
#include <thread> // IWYU pragma: keep
#pragma once

extern pros::Controller Controller;

extern pros::MotorGroup left_mg;
extern pros::MotorGroup right_mg;
extern pros::Motor intake;
extern pros::Motor lever;
extern pros::adi::DigitalOut matchLoad;
extern pros::adi::DigitalOut highMid;
extern pros::adi::DigitalOut hood;
extern pros::adi::DigitalOut wing;

extern pros::Imu imu;
extern pros::Rotation vertical_sensor;
extern lemlib::TrackingWheel vertical_tracker;
extern lemlib::OdomSensors sensors;

extern lemlib::ControllerSettings lateral_controller;
extern lemlib::ControllerSettings angular_controller;

extern lemlib::Chassis chassis;

extern bool matchLoadDown;
extern bool autoLoad;
extern bool up;
extern bool autoTube;
extern bool wingUp;
extern bool autoWing;
extern bool hoodOpen;
extern bool autoHood;
extern int autoLever;
extern float downSpeed;
extern int upSpeed;
extern int skillsUpSpeed;
extern int backSpeed;

#ifndef SLEW
#define SLEW

class slew {
public:
    slew(int accelRate = 8, int decelRate = 8);
    int update(int target);

private:
    int accelRate;
    int decelRate;
    int output;
};

#endif
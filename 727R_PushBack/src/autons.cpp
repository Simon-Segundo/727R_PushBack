#include "main.cpp" // IWYU pragma: keep
#include "main.h" // IWYU pragma: keep
#include "autons.hpp" // IWYU pragma: keep

/* This file contains functions used during autonomous to control the robot
in order to score points without user input. Code here should be optimized
to run as efficiently as possible to maximize the number of points scored in
the autonomous period of a match. */

// Autonomous function covering full alliance side of the field starting on the left side
void fullLeftAuto() {

}

// Autonomous function covering half of the left side of the alliance side of the field
void halfLeftAuto() {
    chassis.setPose(0, 0, 90);
    opposingStorage();
    intake.move(127);
    chassis.moveToPoint(-8.209, 34.832, 5000);
    chassis.moveToPoint(-32.836, 7.765, 5000);
    chassis.moveToPoint(-33.501, 29.286, 5000);
    chassis.moveToPoint(-0.887, 47.257, 5000);

}

// Autonomous function that moves the robot forward a set distance
void moveFwdAuto() {
    chassis.moveToPoint(0, 6, 1000);
}

// Autonomous function covering full alliance side of the field starting on the right side
void fullRightAuto() {

}

// Autonomous function covering half of the right side of the alliance side of the field
void halfRightAuto() {

}

// Autonomous function for skills challenge
void skillsAuto() {

}
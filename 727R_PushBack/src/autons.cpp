#include "main.cpp" // IWYU pragma: keep
#include "main.h" // IWYU pragma: keep
#include "autons.hpp" // IWYU pragma: keep
#include "autons.h"
#include "globals.hpp"

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

    // Transfer
    chassis.moveToPoint(0, 16, 250);
    // Set of 3 Blocks
    chassis.moveToPoint(-12, 53, 1500);
    // Transfer
    chassis.moveToPoint(-8, 37.5, 750, {.forwards = false});
    unstore.move(-127);
    pros::delay(500);
    unstore.brake();
    // Transfer
    chassis.turnToHeading(-135, 500);
    chassis.moveToPoint(-29, 20, 1250, {.maxSpeed = 80});
    chassis.turnToHeading(180, 250);
    // Match Load
    matchLoad.set_value(true);
    chassis.moveToPoint(-34.5, -5, 1000, {.maxSpeed = 50});
    pros::delay(2500);
    matchLoad.set_value(false);
    // Transfer
    chassis.moveToPoint(-32, 15, 1000, {.forwards = false});
    chassis.turnToHeading(-20, 1250, {.maxSpeed = 80});
    // High Goal
    chassis.moveToPoint(-34, 34, 5000, {}, true);
    pros::delay(500);
    unstore.move(127);
    storage.set_value(false);
    oppStore = false;
}

void matchAutoRight () {
    
}

void winPointAuto () {

}

void skillsAuto () {
    storage.set_value(true);
    oppStore = true;
    intake.move(127);
    // Transfer
    chassis.moveToPoint(0, 16, 500);
    chassis.turnToHeading(-90, 500);
    chassis.moveToPoint(-34, 16, 1000);
    matchLoad.set_value(true);
    chassis.turnToHeading(180, 500);
    // Match Load Left Red
    chassis.moveToPoint(-34, 0, 1000);
    pros::delay(500);
    intake.brake();
}
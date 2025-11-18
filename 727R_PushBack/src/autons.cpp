#include "autons.h"

void matchAutoLeft () {
    storage.set_value(true);
    oppStore = true;
    intake.move(127);
    // Transfer
    chassis.moveToPoint(0, 16, 250);
    // Set of 3 Blocks
    chassis.moveToPoint(-12, 53, 1500);
    // Transfer
    chassis.moveToPoint(-8, 37.5, 750, {.forwards = false});
    unstore.move(-127);
    pros::delay(500);
    unstore.brake();
    // Middle Stake
    // chassis.moveToPoint(0, 48, 1000, {.maxSpeed = 80});
    // midScore.set_value(true);
    // unstore.move(127);
    // pros::delay(2000);
    // midScore.set_value(false);
    // unstore.brake();
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

}
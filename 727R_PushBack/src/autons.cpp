#include "autons.h"
#include "globals.hpp"
#include "lemlib/chassis/chassis.hpp"

// Actuates a piston to drop a bar that gives us access to the blocks inside of the match loading tubes
void autoMatchLoad() {
    if (autoLoad == false) {
        matchLoad.set_value(false);
    } else if (autoLoad == true) {
        matchLoad.set_value(true);
    }
}

// Actuates the middle pistons to raise and lower the tube mechanism to score in the middle and high goals
void autoUpDown() {
    if (autoTube == false) {
        highMid.set_value(false);
    } else if (autoTube == true) {
        highMid.set_value(true);
    }
}

// Actuates a piston to both push out the wing from the bot and pull it back into the bot for descore and blocking
void autoWingPos() {
    if (autoWing == false) {
        wing.set_value(false);
    } else if (autoWing == true) {
        wing.set_value(true);
    }
}

// Activates the hood piston to open and close it during the autonomous period
void autoHoodMech() {
    if (autoHood == false) {
        hood.set_value(true);
    } else if (autoHood == true) {
        hood.set_value(false);
    }
}

// Swings the lever mechanism to score blocks in the middle and high goals
void autoLeverSwing() {
    if (autoLever == 0) {
        lever.move_velocity(downSpeed);
        pros::delay(1500);
        lever.move_velocity(backSpeed);
        pros::delay(1000);
        lever.brake();
        autoLever = 2;
    } else if (autoLever == 1) {
        lever.move_velocity(upSpeed);
        pros::delay(1000);
        lever.move_velocity(backSpeed);
        pros::delay(1000);
        lever.brake();
        autoLever = 2;
    }
}

// matchLoad.set_value(true); = put match load down
// highMid.set_value(true); = up
// wing.set_value(true) = open wing
// hood.set_value(true); = open hood
// lever when tube is down requires 1.5s
// lever when tube is up requires 1s

void matchAutoLeft () {
	chassis.setPose(0, 0, 0);
    // Transfer
    chassis.moveToPoint(0, 30, 750);
    chassis.turnToHeading(-45, 250);
    chassis.moveToPoint(-8, 42, 500, {}, false);
    matchLoad.set_value(true);
    intake.move(127);
    chassis.turnToHeading(0, 250);
    // Set of 3 Blocks
    chassis.moveToPoint(-8, 48, 1500);
    // Transfer
    chassis.moveToPoint(-8, 40, 1000, {.forwards = false}, false);
    matchLoad.set_value(false);
    chassis.turnToHeading(-135, 675);
    // Middle Goal
    chassis.moveToPoint(3.5, 50.6, 1000, {.forwards = false}, false);
    hood.set_value(true);
    lever.move_velocity(downSpeed);
    pros::delay(2000);
    lever.move_velocity(backSpeed);
    pros::delay(1000);
    lever.brake();
    intake.brake();
    // Transfer
    chassis.moveToPoint(-34, 16, 1500);
    hood.set_value(false);
    chassis.turnToHeading(180, 500);
    // Match Load
    matchLoad.set_value(true);
    intake.move(127);
    chassis.moveToPoint(-34, 0, 1250);
    // High Goal
    highMid.set_value(true);
    up = true;
    chassis.moveToPoint(-34, 38, 1500, {.forwards = false}, false);
    hood.set_value(true);
    lever.move_velocity(100);
    pros::delay(1000);
    lever.move_velocity(backSpeed);
    pros::delay(1000);
    lever.brake();
    intake.brake();
}

void matchAutoRight () {
	chassis.setPose(0, 0, 0);
    // Transfer
    chassis.moveToPoint(0, 30, 750);
    chassis.turnToHeading(45, 250);
    chassis.moveToPoint(8, 42, 500);
    intake.move(127);
    chassis.turnToHeading(0, 250);
    // Set of 3 Blocks
    chassis.moveToPoint(8, 50, 1500);
    // Transfer
    chassis.moveToPoint(8, 42, 1000, {.forwards = false}, false);
    intake.brake();
    chassis.turnToHeading(-45, 675);
    // Low Goal
    chassis.moveToPoint(-3, 51, 1500, {}, false);
    intake.move(-127);
    pros::delay(2000);
    intake.brake();
    // Transfer
    chassis.moveToPoint(30, 16, 1750, {.forwards = false});
    chassis.turnToHeading(180, 500);
    // Match Load
    matchLoad.set_value(true);
    intake.move(127);
    chassis.moveToPoint(30, 0, 1000, {}, false);
    // High Goal
    highMid.set_value(true);
    up = true;
    chassis.moveToPoint(34, 38, 1500, {.forwards = false}, false);
    hood.set_value(true);
    lever.move_velocity(75);
    pros::delay(1000);
    lever.move_velocity(backSpeed);
    pros::delay(1000);
    lever.brake();
    intake.brake();
}

void winPointAuto () {

}

void skillsAuto () {
	chassis.setPose(-2.5, 8.625, 0);

    // First Half
    // Transfer
    highMid.set_value(true);
    chassis.moveToPoint(-2.5, 40, 1250);
    chassis.turnToHeading(-90, 750);
    // Match Load 1
    matchLoad.set_value(true);
    intake.move(127);
    chassis.moveToPoint(-12.25,40.5,3500, {}, false);
    intake.brake();
    // Transfer
    chassis.moveToPoint(0, 41, 1250, {.forwards = false}, false);
    intake.brake();
    matchLoad.set_value(false);
    chassis.turnToHeading(-135, 250);
    chassis.moveToPoint(12, 53, 1000, {.forwards = false});
    chassis.turnToHeading(-90, 500);
    chassis.moveToPoint(95, 53, 1500, {.forwards = false});
    chassis.turnToHeading(0, 1000);
    chassis.moveToPoint(98, 38, 1100, {.forwards = false});
    chassis.turnToHeading(90, 1000);
    // Long Goal 1, 1
    chassis.moveToPoint(80, 38, 1000, {.forwards = false}, false);
    chassis.setPose(0,0,0);
    intake.move(127);
    matchLoad.set_value(true);
    hood.set_value(true);
    lever.move_velocity(skillsUpSpeed);
    pros::delay(1250);
    lever.move_velocity(backSpeed);
    pros::delay(1000);
    lever.brake();
    intake.brake();
    chassis.setPose(0,-19.25,0);
    // Match Load 2
    intake.move(127);
    chassis.moveToPoint(0, 12.75, 700, {.maxSpeed = 100}, false);
    hood.set_value(false);
    chassis.moveToPoint(1, 12.5, 700, {.maxSpeed = 100});
    chassis.moveToPoint(1, 12.75, 700, {.maxSpeed = 100});
    chassis.moveToPoint(1, 12.5, 700, {.maxSpeed = 100});
    chassis.moveToPoint(1, 12.75, 700, {.maxSpeed = 100});
    // Long Goal 1, 2
    chassis.moveToPoint(0, -19.25, 2250, {.forwards = false}, false);
    hood.set_value(true);
    lever.move_velocity(skillsUpSpeed);
    pros::delay(1250);
    lever.move_velocity(backSpeed);
    pros::delay(1000);
    lever.brake();
    intake.brake();
    matchLoad.set_value(false);
    
    // Second Half
    // Transfer
    intake.move(-127);
    chassis.moveToPoint(0, -5, 1000, {}, false);
    hood.set_value(false);
    chassis.turnToHeading(90, 500);
    chassis.moveToPoint(100, -5, 2000);
    chassis.turnToHeading(0, 500);
    intake.brake();
    // Match Load 3
    matchLoad.set_value(true);
    intake.move(127);
    chassis.moveToPoint(100,13.25,3750, {}, false);
    intake.brake();
    // Transfer
    chassis.moveToPoint(100, -5, 1250, {.forwards = false}, false);
    matchLoad.set_value(false);
    chassis.turnToHeading(-45, 250);
    chassis.moveToPoint(115, -19, 1000, {.forwards = false});
    chassis.turnToHeading(0, 500);
    chassis.moveToPoint(115, -100, 1500, {.forwards = false});
    chassis.turnToHeading(90, 1000);
    chassis.moveToPoint(101, -100, 1100, {.forwards = false});
    chassis.turnToHeading(180, 1000);
    // Long Goal 2, 1
    chassis.moveToPoint(101, -80.75, 1000, {.forwards = false}, false);
    chassis.setPose(0,0,1);
    intake.move(127);
    matchLoad.set_value(true);
    hood.set_value(true);
    lever.move_velocity(skillsUpSpeed);
    pros::delay(1250);
    lever.move_velocity(backSpeed);
    pros::delay(1000);
    lever.brake();
    intake.brake();
    chassis.setPose(0, 0, 0);
    // Match Load 4
    intake.move(127);
    chassis.moveToPoint(1, 30.5, 3000);
    pros::delay(100);
    hood.set_value(false);
    // Long Goal 2, 2
    chassis.moveToPoint(0, 0, 1000, {.forwards = false}, false);
    hood.set_value(true);
    lever.move_velocity(skillsUpSpeed);
    pros::delay(1350);
    lever.move_velocity(backSpeed);
    pros::delay(1000);
    lever.brake();
    intake.brake();
    matchLoad.set_value(false);

    // Parking
    // Transfer
    chassis.moveToPoint(0, 10, 1000);
    // // Park From Front
    // chassis.moveToPoint(48, 15, 1000);
    // chassis.turnToHeading(0, 1000);
    // intake.move(-127);
    // chassis.moveToPoint(48, 0, 2000, {.forwards = false});
    // chassis.moveToPoint(48, 47, 4000);
    // chassis.turnToHeading(-90, 1000);

    // Park From Side
    intake.move(-127);
    chassis.turnToHeading(90, 750);
    chassis.moveToPoint(24, 32.5, 2000);
    chassis.turnToHeading(90, 1000);
    chassis.moveToPoint(50, 32.5, 15000, {.maxSpeed = 80});
}

void tuningAuto () {
	chassis.setPose(0, 0, 0);

    chassis.moveToPoint(0,24,10000);
    // chassis.turnToHeading(90, 10000);
}
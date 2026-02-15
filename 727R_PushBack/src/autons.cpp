#include "autons.h"
#include "distanceReset.hpp"
#include "globals.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"

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
        hood.set_value(true);
        intake.move(127);
        lever.move_velocity(downSpeed);
        pros::delay(1500);
        lever.move_velocity(backSpeed);
        intake.brake();
        pros::delay(1000);
        hood.set_value(false);
        lever.brake();
        autoLever = 2;
    } else if (autoLever == 1) {
        hood.set_value(true);
        intake.move(127);
        lever.move_velocity(upSpeed);
        pros::delay(1000);
        lever.move_velocity(backSpeed);
        intake.brake();
        pros::delay(1000);
        hood.set_value(false);
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
    chassis.setPose(0, 0, 90);
    resetRobotPos(rearDistance, WallAxis::NEG_X);
    resetRobotPos(leftDistance, WallAxis::POS_Y);
    highMid.set_value(true);

    // 3 Blocks
    intake.move(127);
    chassis.turnToPoint(-24, 24, 1000, {.minSpeed = 50});
    chassis.moveToPoint(-24, 10, 1000, {}, false);
    matchLoad.set_value(true);
    // Middle Goal
    chassis.moveToPoint(-24, 24, 1000, {.forwards = false});
    chassis.turnToPoint(-12, 12, 1000, {.forwards = false});
    chassis.moveToPoint(-12, 12, 1000, {.forwards = false});
    hood.set_value(true);
    lever.move_velocity(downSpeed);
    pros::delay(1500);
    lever.move_velocity(backSpeed);
    pros::delay(1000);
    lever.brake();
    intake.brake();
	// Match Load
    chassis.moveToPoint(-48, 47.75, 1000);
    chassis.turnToPoint(-48, 70, 750, {}, false);
    resetRobotPos(frontDistance, WallAxis::POS_Y);
    resetRobotPos(leftDistance, WallAxis::NEG_X);
    chassis.turnToPoint(-63,47.75, 1250);
    intake.move(127);
    chassis.moveToPoint(-63,47.75,2250, {.minSpeed = 30});
    // Long Goal
    chassis.moveToPoint(-30, 48, 1750, {.forwards = false}, false);
    hood.set_value(true);
    lever.move_velocity(upSpeed);
    pros::delay(1300);
    lever.move_velocity(backSpeed);
    pros::delay(1000);
    lever.brake();
    intake.brake();
}

void matchAutoRight () {
    chassis.setPose(0, 0, -90);
    resetRobotPos(rearDistance, WallAxis::NEG_X);
    resetRobotPos(rightDistance, WallAxis::NEG_Y);
    highMid.set_value(true);
    wing.set_value(true);

    // 3 Blocks
    intake.move(127);
    chassis.turnToPoint(-24, -24, 1000, {.minSpeed = 50});
    chassis.moveToPoint(-24, -10, 1250);
    matchLoad.set_value(true);
    // Low Goal
    chassis.moveToPoint(-24, -24, 1000, {.forwards = false});
    chassis.turnToPoint(-12, -12, 1000, {.forwards = false}, false);
    matchLoad.set_value(false);
    chassis.moveToPoint(-12, -12, 1000, {.forwards = false}, false);
    pros::delay(100);
    matchLoad.set_value(true);
    intake.move_velocity(-175);
    pros::delay(1000);
    matchLoad.set_value(false);
	// Match Load
    chassis.moveToPoint(-48, -47.75, 750);
    chassis.turnToPoint(-48, -70, 750, {}, false);
    resetRobotPos(frontDistance, WallAxis::NEG_Y);
    resetRobotPos(rightDistance, WallAxis::NEG_X);
    chassis.turnToPoint(-63, -47.75, 1250);
    matchLoad.set_value(true);
    intake.move(127);
    chassis.moveToPoint(-63, -47.75,2250, {.minSpeed = 30});
    // Long Goal
    chassis.moveToPoint(-30, -48, 1750, {.forwards = false}, false);
    hood.set_value(true);
    lever.move_velocity(upSpeed);
    pros::delay(1300);
    lever.move_velocity(backSpeed);
    pros::delay(1000);
    lever.brake();
    intake.brake();
}

void winPointAuto () {
    chassis.setPose(0, 0, 0);
    resetRobotPos(rearDistance, WallAxis::POS_Y);
    resetRobotPos(leftDistance, WallAxis::NEG_X);
    highMid.set_value(true);

    // Left Match Loader
    chassis.moveToPoint(-51, 48, 1250);
    chassis.turnToPoint(-60,48, 750);
    matchLoad.set_value(true);
    intake.move(127);
    chassis.moveToPoint(-60,48,1000, {.minSpeed = 30});
    // Left Long Goal
    chassis.moveToPoint(-30, 48, 1000, {.forwards = false}, false);
    hood.set_value(true);
    lever.move_velocity(upSpeed);
    pros::delay(1000);
    lever.move_velocity(backSpeed);
    pros::delay(1000);
    intake.brake();
    lever.brake();
    matchLoad.set_value(false);
    // Pick Up 6 Blocks
    intake.move(127);
    chassis.turnToPoint(-24, 24, 1250, {}, false);
    hood.set_value(false);
    chassis.moveToPoint(-24, -36, 1750, {}, false);
    matchLoad.set_value(true);
    // Score Low Middle Goal
    chassis.moveToPoint(-24, -24, 1200, {.forwards = false});
    chassis.turnToPoint(-12, 12, 1000, {}, false);
    matchLoad.set_value(false);
    intake.brake();
    chassis.moveToPoint(-12, -12, 1000, {}, false);
    intake.move_velocity(-500);
    pros::delay(1000);
    intake.brake();

}

void skillsAuto74 () {
    // // Side Clear
    // resetRobotPos(rightDistance, WallAxis::NEG_X);
    // intake.move(127);
    // chassis.moveToPoint(-63, -12, 2000);
    // pros::delay(1750);
    // matchLoad.set_value(true);
    // pros::delay(500);
    // resetRobotPos(frontDistance, WallAxis::NEG_Y);

    // Front Clear
    // chassis.setPose(0, 0, -90);
	// resetRobotPos(frontDistance, WallAxis::NEG_X);
    // wing.set_value(true);
    // // Clear Park Zone
    // intake.move(127);
    // chassis.moveToPoint(-61, 0, 1000, {.minSpeed = 55});
    // chassis.moveToPoint(-60, 0, 750, {.forwards = false, .minSpeed = 45});
    // chassis.moveToPoint(-63, 0, 1500, {.minSpeed = 55}, false);
    // pros::delay(2100);
    // // Middle Goal
    // chassis.moveToPoint(-44, 0, 1250, {.forwards = false});
    // resetRobotPos(frontDistance, WallAxis::NEG_X);
    // chassis.turnToPoint(-24, 14, 1250);
    // chassis.moveToPoint(-24, 14, 1250);
    // pros::delay(750);
    // chassis.turnToPoint(20, 0, 1000, {.forwards = false});
    // chassis.moveToPoint(-28, 8, 1000);

    // First Half
    left_mg.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    right_mg.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(0, 0, 90);
    resetRobotPos(rearDistance, WallAxis::NEG_X);
    resetRobotPos(leftDistance, WallAxis::POS_Y);
    highMid.set_value(true);
    wing.set_value(true);

    // Match Load 1
    chassis.moveToPoint(-48, 24, 750);
    chassis.turnToPoint(-48, 48, 750);
    chassis.moveToPoint(-48, 47.75, 1000, {}, false);
    resetRobotPos(frontDistance, WallAxis::POS_Y);
    resetRobotPos(leftDistance, WallAxis::NEG_X);
    chassis.turnToPoint(-63,47.75, 1250);
    matchLoad.set_value(true);
    intake.move(127);
    chassis.moveToPoint(-63,47.75,2250, {.minSpeed = 30});
    // Long Goal 1, 1
    chassis.moveToPoint(-48, 48, 1750, {.forwards = false}, false);
    intake.brake();
    matchLoad.set_value(false);
    chassis.turnToPoint(-33, 63, 1000);
    chassis.moveToPoint(-33, 63, 1000);
    chassis.turnToPoint(48, 63, 1000, {}, false);
	resetRobotPos(rearDistance, WallAxis::NEG_X);
    // Tracks the disance from the wall the whole time the robot travels along the long goal
    pros::Task leftReset([&] {
        while (true) {
            resetRobotPos(leftDistance, WallAxis::POS_Y);
        }
    });
    chassis.moveToPoint(48, 63, 1500, {}, false);
	resetRobotPos(frontDistance, WallAxis::POS_X);
    leftReset.remove();
    chassis.turnToPoint(48, 50, 1000);
    chassis.moveToPoint(48, 50, 1000, {}, false);
	resetRobotPos(rearDistance, WallAxis::POS_Y);
    resetRobotPos(leftDistance, WallAxis::POS_X);
    chassis.turnToPoint(70, 48, 1000); // Turn away from long goal
    chassis.moveToPoint(30, 48, 1000, {.forwards = false, .minSpeed = 80}, false);
    intake.move(127);
    matchLoad.set_value(true);
    hood.set_value(true);
    lever.move_velocity(skillsUpSpeed);
    pros::delay(1300);
    lever.move_velocity(backSpeed);
    pros::delay(1000);
    lever.brake();
    intake.brake();
    // Match Load 2
    intake.move(127);
    chassis.moveToPoint(63.4, 47.75, 2500, {.maxSpeed = 100});
    pros::delay(700);
    hood.set_value(false);
    // Long Goal 1, 2
    chassis.moveToPoint(30, 48.5, 1750, {.forwards = false}, false);
    hood.set_value(true);
    lever.move_velocity(skillsUpSpeed);
    pros::delay(1300);
    lever.move_velocity(backSpeed);
    pros::delay(1000);
    lever.brake();
    intake.brake();
    matchLoad.set_value(false);
    
    // Second Half
    // Match Load 3
    intake.move(-127);
    chassis.moveToPoint(40, 48, 1000, {}, false);
    hood.set_value(false);
    chassis.turnToPoint(40, -50, 1000);
    chassis.moveToPoint(40, -50, 2000, {.maxSpeed = 110}, false);
	resetRobotPos(frontDistance, WallAxis::NEG_Y);
    resetRobotPos(leftDistance, WallAxis::POS_X);
    chassis.turnToPoint(63,-47, 1000);
    matchLoad.set_value(true);
    intake.brake();
    intake.move(127);
    chassis.moveToPoint(63,-47,2500, {.maxSpeed = 110});
    // Long Goal 2, 1
    chassis.moveToPoint(48, -48, 1250, {.forwards = false}, false);
    intake.brake();
    matchLoad.set_value(false);
    chassis.turnToPoint(33, -63, 1000);
    chassis.moveToPoint(33, -63, 1000);
    chassis.turnToPoint(-48, -63, 1000, {}, false);
	resetRobotPos(rearDistance, WallAxis::POS_X);
    // Tracks the disance from the wall the whole time the robot travels along the long goal
    pros::Task leftReset3([&] {
        while (true) {
            resetRobotPos(leftDistance, WallAxis::NEG_Y);
        }
    });
    chassis.moveToPoint(-48, -63, 1500, {}, false);
	resetRobotPos(frontDistance, WallAxis::NEG_X);
    leftReset3.remove();
    chassis.turnToPoint(-48, -48, 1000);
    chassis.moveToPoint(-48, -48, 1000, {}, false);
	resetRobotPos(rearDistance, WallAxis::NEG_Y);
    resetRobotPos(leftDistance, WallAxis::NEG_X);
    chassis.turnToPoint(-28, -48, 1000, {.forwards = false}); // Turn away from long goal
    chassis.moveToPoint(-28, -48, 1250, {.forwards = false}, false);
    intake.move(127);
    matchLoad.set_value(true);
    hood.set_value(true);
    lever.move_velocity(skillsUpSpeed);
    pros::delay(1300);
    lever.move_velocity(backSpeed);
    pros::delay(1000);
    lever.brake();
    intake.brake();
    // Match Load 4
    intake.move(127);
    chassis.moveToPoint(-64, -47.75, 2400, {.maxSpeed = 100});
    pros::delay(100);
    hood.set_value(false);
    // Long Goal 2, 2
    chassis.moveToPoint(-30, -48, 2250, {.forwards = false}, false);
    hood.set_value(true);
    lever.move_velocity(skillsUpSpeed);
    pros::delay(1300);
    lever.move_velocity(backSpeed);
    pros::delay(1000);
    lever.brake();
    intake.brake();
    matchLoad.set_value(false);

    // Parking
    chassis.moveToPoint(-48, -48, 1000);
    chassis.turnToPoint(-69, -12, 750);
    chassis.moveToPoint(-69, -12, 1000, {.minSpeed = 25});
    intake.move(-127);
    chassis.moveToPoint(-69, 0, 2000, {.minSpeed = 75});
    pros::delay(250);
    pros::Task ([&] {
        while (true) {
            resetRobotPos(rearDistance, WallAxis::NEG_Y);
        }
    });
    pros::Task ([&] {
        while (true) {
            resetRobotPos(leftDistance, WallAxis::NEG_X);
        }
    });
    intake.move(127);
}

void skillsAuto79 () {
    // // Side Clear
    // resetRobotPos(rightDistance, WallAxis::NEG_X);
    // intake.move(127);
    // chassis.moveToPoint(-63, -12, 2000);
    // pros::delay(1750);
    // matchLoad.set_value(true);
    // pros::delay(500);
    // resetRobotPos(frontDistance, WallAxis::NEG_Y);

    // Front Clear
    // chassis.setPose(0, 0, -90);
	// resetRobotPos(frontDistance, WallAxis::NEG_X);
    // wing.set_value(true);
    // // Clear Park Zone
    // intake.move(127);
    // chassis.moveToPoint(-61, 0, 1000, {.minSpeed = 55});
    // chassis.moveToPoint(-60, 0, 750, {.forwards = false, .minSpeed = 45});
    // chassis.moveToPoint(-63, 0, 1500, {.minSpeed = 55}, false);
    // pros::delay(2100);
    // // Middle Goal
    // chassis.moveToPoint(-44, 0, 1250, {.forwards = false});
    // resetRobotPos(frontDistance, WallAxis::NEG_X);
    // chassis.turnToPoint(-24, 14, 1250);
    // chassis.moveToPoint(-24, 14, 1250);
    // pros::delay(750);
    // chassis.turnToPoint(20, 0, 1000, {.forwards = false});
    // chassis.moveToPoint(-28, 8, 1000);

    // First Half
    left_mg.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    right_mg.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(0, 0, 90);
    resetRobotPos(rearDistance, WallAxis::NEG_X);
    resetRobotPos(leftDistance, WallAxis::POS_Y);
    highMid.set_value(true);
    wing.set_value(true);

    // Match Load 1
    chassis.moveToPoint(-48, 24, 750);
    chassis.turnToPoint(-48, 48, 750);
    chassis.moveToPoint(-48, 47.75, 1000, {}, false);
    resetRobotPos(frontDistance, WallAxis::POS_Y);
    resetRobotPos(leftDistance, WallAxis::NEG_X);
    chassis.turnToPoint(-63,47.75, 1250);
    matchLoad.set_value(true);
    intake.move(127);
    chassis.moveToPoint(-63,47.75,2250, {.minSpeed = 30});
    // Long Goal 1, 1
    chassis.moveToPoint(-48, 48, 1000, {.forwards = false}, false);
    intake.brake();
    matchLoad.set_value(false);
    chassis.turnToPoint(-33, 63, 1000);
    chassis.moveToPoint(-33, 63, 1000);
    chassis.turnToPoint(48, 63, 1000, {}, false);
	resetRobotPos(rearDistance, WallAxis::NEG_X);
    // Tracks the disance from the wall the whole time the robot travels along the long goal
    pros::Task leftReset([&] {
        while (true) {
            resetRobotPos(leftDistance, WallAxis::POS_Y);
        }
    });
    chassis.moveToPoint(48, 63, 1500, {}, false);
	resetRobotPos(frontDistance, WallAxis::POS_X);
    leftReset.remove();
    chassis.turnToPoint(48, 50, 1000);
    chassis.moveToPoint(48, 50, 1000, {}, false);
	resetRobotPos(rearDistance, WallAxis::POS_Y);
    resetRobotPos(leftDistance, WallAxis::POS_X);
    chassis.turnToPoint(70, 48, 1000); // Turn away from long goal
    chassis.moveToPoint(30, 48, 1000, {.forwards = false, .minSpeed = 80}, false);
    intake.move(127);
    matchLoad.set_value(true);
    hood.set_value(true);
    lever.move_velocity(skillsUpSpeed);
    pros::delay(1300);
    lever.move_velocity(backSpeed);
    pros::delay(1000);
    lever.brake();
    intake.brake();
    // Match Load 2
    intake.move(127);
    chassis.moveToPoint(63.5, 47.75, 2500, {.maxSpeed = 100});
    pros::delay(700);
    hood.set_value(false);
    // Long Goal 1, 2
    chassis.moveToPoint(30, 48.5, 1750, {.forwards = false}, false);
    hood.set_value(true);
    lever.move_velocity(skillsUpSpeed);
    pros::delay(1300);
    lever.move_velocity(backSpeed);
    pros::delay(1000);
    lever.brake();
    intake.brake();
    matchLoad.set_value(false);
    
    // Second Half
    // Clear Blue Park
    intake.move(-127);
    chassis.moveToPoint(48, 48, 1000, {}, false);
    hood.set_value(false);
    chassis.turnToPoint(67, 12, 750, {}, false);
    intake.brake();
    intake.move(127);
    chassis.moveToPoint(67, 12, 750, {.earlyExitRange = 0.5});
    chassis.moveToPoint(67, 0, 1000, {.minSpeed = 80});
    pros::delay(750);
    pros::Task rearReset1([&] {
        while (true) {
            resetRobotPos(rearDistance, WallAxis::POS_Y);
        }
    });
    pros::Task leftReset2([&] {
        while (true) {
            resetRobotPos(leftDistance, WallAxis::POS_X);
        }
    });
    pros::delay(500);
    rearReset1.suspend();
    chassis.moveToPoint(60, -12, 1000, {.minSpeed = 80});
    pros::delay(200);
    rearReset1.resume();
    pros::delay(550);
    rearReset1.remove();
    leftReset2.remove();

    // Match Load 3
    chassis.turnToPoint(48, -47, 1000);
    chassis.moveToPoint(48, -47, 2000, {.maxSpeed = 110});
    chassis.turnToPoint(48, -70, 750, {}, false);
    hood.set_value(true);
    lever.move_velocity(100);
	resetRobotPos(frontDistance, WallAxis::NEG_Y);
    resetRobotPos(leftDistance, WallAxis::POS_X);
    matchLoad.set_value(true);
    chassis.turnToPoint(63,-47, 1000, {}, false);
    lever.move_velocity(backSpeed);
    intake.brake();
    intake.move(127);
    chassis.moveToPoint(63,-47,2250);
    pros::delay(1000);
    lever.brake();
    hood.set_value(false);
    // Long Goal 2, 1
    chassis.moveToPoint(48, -48, 1250, {.forwards = false}, false);
    intake.brake();
    matchLoad.set_value(false);
    chassis.turnToPoint(33, -63, 1000);
    chassis.moveToPoint(33, -63, 1000);
    chassis.turnToPoint(-48, -63, 1000, {}, false);
	resetRobotPos(rearDistance, WallAxis::POS_X);
    // Tracks the disance from the wall the whole time the robot travels along the long goal
    pros::Task leftReset3([&] {
        while (true) {
            resetRobotPos(leftDistance, WallAxis::NEG_Y);
        }
    });
    chassis.moveToPoint(-48, -63, 1500, {}, false);
	resetRobotPos(frontDistance, WallAxis::NEG_X);
    leftReset3.remove();
    chassis.turnToPoint(-48, -48, 1000);
    chassis.moveToPoint(-48, -48, 1000, {}, false);
	resetRobotPos(rearDistance, WallAxis::NEG_Y);
    resetRobotPos(leftDistance, WallAxis::NEG_X);
    chassis.turnToPoint(-30, -48, 1000, {.forwards = false}); // Turn away from long goal
    chassis.moveToPoint(-30, -48, 1250, {.forwards = false}, false);
    intake.move(127);
    matchLoad.set_value(true);
    hood.set_value(true);
    lever.move_velocity(skillsUpSpeed);
    pros::delay(1300);
    lever.move_velocity(backSpeed);
    pros::delay(1000);
    lever.brake();
    intake.brake();
    // Match Load 4
    intake.move(127);
    chassis.moveToPoint(-64, -47.75, 2250, {.maxSpeed = 100});
    pros::delay(100);
    hood.set_value(false);
    // Long Goal 2, 2
    chassis.moveToPoint(-30, -48, 2250, {.forwards = false}, false);
    hood.set_value(true);
    lever.move_velocity(skillsUpSpeed);
    pros::delay(1300);
    lever.move_velocity(backSpeed);
    pros::delay(1000);
    lever.brake();
    intake.brake();
    matchLoad.set_value(false);

    // Parking
    chassis.moveToPoint(-48, -48, 1000);
    chassis.turnToPoint(-69, -12, 750);
    chassis.moveToPoint(-69, -12, 1000, {.minSpeed = 25});
    intake.move(-127);
    chassis.moveToPoint(-69, 0, 2000, {.minSpeed = 75});
    pros::delay(250);
    pros::Task ([&] {
        while (true) {
            resetRobotPos(rearDistance, WallAxis::NEG_Y);
        }
    });
    pros::Task ([&] {
        while (true) {
            resetRobotPos(leftDistance, WallAxis::NEG_X);
        }
    });
    intake.move(127);
}

void tuningAuto () {
	chassis.setPose(0, 0, 0);

    chassis.moveToPoint(0,24,10000);
    // chassis.turnToHeading(90, 10000);
}
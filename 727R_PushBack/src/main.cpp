#include "autons.h"
#include "liblvgl/lv_api_map.h"
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

// Intakes and outtakes the blocks
void intaking() {
	if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
		intake.move(127);
	} else if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
		intake.move(-100);
    } else {
        intake.brake();
    }
}

// Actuates the bottom piston to drop a bar that gives us access to the blocks inside of the match loading tubes
void matchLoading() {
    if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) && (matchLoadDown == false)) {
        matchLoad.set_value(true);
        matchLoadDown = true;
        pros::delay(250);
    } else if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) && (matchLoadDown == true)) {
        matchLoad.set_value(false);
        matchLoadDown = false;
        pros::delay(250);
    }
}

// Actuates the middle piston to block the intake at the middle to redirect the blocks into the middle tube
void upDown() {
    if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP) && (up == false)) {
        highMid.set_value(true);
        up = true;
		pros::delay(250);
    } else if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP) && (up == true)) {
        highMid.set_value(false);
        up = false;
        pros::delay(250);
    }
}

// Actuates a piston to both push out the wing from the bot and pull it back into the bot for descore and blocking
void wingPos() {
    if ((Controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) && (wingUp == false)) {
        wing.set_value(true);
        wingUp = true;
        pros::delay(250);
    } else if ((Controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) && (wingUp == true)) {
        wing.set_value(false);
        wingUp = false;
        pros::delay(250);
    }
}

// Activates the hood piston to open and close it
void hoodMech() {
    if ((Controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) && (hoodOpen == false)) {
        hood.set_value(false);
        hoodOpen = true;
        pros::delay(250);
    } else if ((Controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) && (hoodOpen == true)) {
        hood.set_value(true);
        hoodOpen = false;
        pros::delay(250);
    }
}

// Swings the lever mechanism to score blocks in the middle and high goals
void leverSwing() {
    if (up == false) {
        lever.move_velocity(downSpeed);
    } else if (up == true) {
// *************** Change when in skills/match ***********************
        lever.move_velocity(100);
    }
    while (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        pros::delay(10);
    }
    lever.move_velocity(backSpeed);
    pros::delay(500);
    lever.brake();
}

void initialize() {
    lever.set_brake_mode(pros::MotorBrake::brake);
    pros::lcd::initialize();
    chassis.calibrate();
    lever.move(-127);
    pros::delay(100);
    lever.brake();
	
	// Tasks for drivercontrol and autonomous periods
	pros::Task ([&]{
		while (true) {
            lv_task_handler();
			pros::lcd::print(1, "X-Value: %f", chassis.getPose().x);
			pros::lcd::print(2, "Y-Value: %f", chassis.getPose().y);
			pros::lcd::print(3, "Theta: %f", chassis.getPose().theta);
			pros::Task::delay(10);
		}
	});
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
 
void autonomous() {
    // matchAutoLeft();
    matchAutoRight();
    // winPointAuto();
    // skillsAuto();
    // tuningAuto();
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
    // Defines the slew speed for throttle
    slew throttleSlew(10, 10);

    // Tasks for drivercontrol
    pros::Task ([&] () {
        while(true) {
            lv_task_handler();
            matchLoading(); // Calls the matchLoading function
            pros::delay(10);
        }
    });
    pros::Task ([&] () {
        while(true) {
            lv_task_handler();
            upDown();       // Calls the upDown function
            pros::delay(10);
        }
    });
    pros::Task ([&] () {
        while(true) {
            lv_task_handler();
            wingPos(); // Calls the wingPos function
            pros::delay(10);
        }
    });
    pros::Task ([&] () {
        while(true) {
            lv_task_handler();
            if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
                leverSwing(); // Calls the leverSwing function
            }
            pros::delay(10);
        }
    });
    pros::Task ([&] () {
        while (true) {
            lv_task_handler();
            hoodMech(); // Calls the hoodMech function
            pros::delay(10);
        }
    });

    while (true) {
        // Arcade control scheme
        int dir = Controller.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
        int turn = Controller.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick

        // Adds slew to the inputted controller values
        int throttleOut = throttleSlew.update(dir);

        // Arcade Drive control scheme
        chassis.arcade(throttleOut, turn);

        // Curvature Drive control scheme
        // chassis.curvature(dir, turn);            // Similar to arcade but turns better

        // Calling functions
        intaking();     // Calls the intaking function

        // How long it takes for each update of inputs
        pros::delay(20);
    }
}
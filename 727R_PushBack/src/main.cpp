#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "liblvgl/widgets/lv_label.h" // IWYU pragma: keep
#include "pros/abstract_motor.hpp"
#include "pros/adi.h" // IWYU pragma: keep
#include "pros/adi.hpp"
#include "liblvgl/llemu.hpp"
#include "pros/apix.h" // IWYU pragma: keep
#include "pros/colors.hpp" // IWYU pragma: keep
#include "pros/device.hpp" // IWYU pragma: keep
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/optical.h" // IWYU pragma: keep
#include "pros/optical.hpp" // IWYU pragma: keep
#include <algorithm>
#include <cmath>
#include <thread> // IWYU pragma: keep

pros::Controller Controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup left_mg({-11, -12, -13}, pros::MotorGearset::blue);    // Creates a motor group with reversed ports 11 12 & 13
pros::MotorGroup right_mg({20, 19, 18}, pros::MotorGearset::blue);  // Creates a motor group with forwards port 18 19 & 20
pros::Motor intake(-2, pros::MotorGearset::blue);
pros::Motor unstore(-9);
pros::Motor store(10);
pros::adi::DigitalOut matchLoad('C');
pros::adi::DigitalOut midScore('B');
pros::adi::DigitalOut storage('A');

// Drivetrain settings
lemlib::Drivetrain drivetrain(&left_mg, // left motor group
                              &right_mg, // right motor group
                              12.75, // 12.75 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              800, // drivetrain rpm is the gear ratio multiplied by the rpm of the driving motor
                              2 // horizontal drift is 2 (for now)
);

// Creates an imu on port 21
pros::Imu imu(21);

// Creates a V5 vertical rotation sensor on port 9
pros::Rotation vertical_sensor(17);

// Creates a V5 optical sensor on port 11
pros::Optical colorSensor (3);

// Vertical Tracking Wheel
lemlib::TrackingWheel vertical_tracker(&vertical_sensor, lemlib::Omniwheel::NEW_2, 0);

// Combines all sensors into one item
lemlib::OdomSensors sensors(&vertical_tracker, // vertical tracking wheel 1
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// Lateral PID controller
lemlib::ControllerSettings lateral_controller(7, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3.5, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// Angular PID controller
lemlib::ControllerSettings angular_controller(1.8, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              11.7, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in degrees
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in degrees
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// Creates the chassis
lemlib::Chassis chassis(drivetrain, // Drivetrain Settings
                        lateral_controller, // Lateral PID Settings
                        angular_controller, // Angular PID Settings
                        sensors // Odometry Sensors
);

class slew {
public:

    slew(int accelRate = 8, int decelRate = 8)
        : accelRate(std::abs(accelRate)), decelRate(std::abs(decelRate)), output(0) {}

    int update(int target) {
        target = std::clamp(target, -127, 127);
        int delta = target - output;

        if (delta > 0) {
            output += std::min(delta, accelRate);
        } else if (delta < 0) {
            output += std::max(delta, -decelRate);
        }
        return output;
    }

    void reset(int value = 0) {
        output = std::clamp(value, -127, 127);
    }

private:
    int accelRate;
    int decelRate;
    int output;
};

// Used to toggle the top piston to allow for shooting blocks into the top storage
bool oppStore = false;

// Used to toggle the top piston to allow for scoring blocks in the middle goal
bool middleScore = false;

// Used to toggle the top piston to access the blocks at the bottom of the match loading tubes
bool matchLoadDown = false;

// Used to disable the color sensor while trying to score in the top or middle goals
bool scoring = false;

// Left off right switch
// Used to toggle between sorting red out vs blue out
// Blue is false, Red is true
bool sortColorToggle = true;

// Used to toggle the color sensor on and off
bool sortToggle = true;

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
		intake.move(-127);
    }
}

// Takes blocks out of storage when holding R1 and holds the position of the rubber band roller when not holding it so extra blocks wont come out of storage
void unstoring() {
    unstore.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        unstore.move(127);
    } else if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
        unstore.move(-127);
    } else {
        unstore.brake();
    }
}

// Actuates the piston at the top of the robot to allow for storage of the opposite color of block
void opposingStorage() {
    // Toggles the top piston on and off and changes boolean values to help with the color sort modes
    if ((Controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) && (oppStore == false)) {
    	storage.set_value(true);
        oppStore = true;
    	pros::delay(250);
    } else if ((Controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) && (oppStore == true)) {
    	storage.set_value(false);
        oppStore = false;
    	pros::delay(250);
    }
}

void colorSorting() {
    // Sorts the blocks into the storage system and up and over the robot
    if ((oppStore == true) && (sortToggle == true)) {
        if (sortColorToggle == false) {
            // Store Blue Blocks
            if(((colorSensor.get_hue() >= 65) && (colorSensor.get_hue() <= 359)) && (colorSensor.get_proximity() >= 50)) {
            store.move(127);
            pros::delay(100);
            } else if(((colorSensor.get_hue() >= 0) && (colorSensor.get_hue() <= 57)) && (colorSensor.get_proximity() >= 50)) {
                store.move(-127);
            pros::delay(750);
            } else {
                store.brake();
            }
        } else if (sortColorToggle == true) {
            // Store Red Blocks
            if(((colorSensor.get_hue() >= 0) && (colorSensor.get_hue() <= 57)) && (colorSensor.get_proximity() >= 50)) {
                store.move(127);
                pros::delay(100);
            } else if(((colorSensor.get_hue() >= 65) && (colorSensor.get_hue() <= 359)) && (colorSensor.get_proximity() >= 50)) {
                store.move(-127);
                pros::delay(750);
            } else {
                store.brake();
            }
        }
    } else if (oppStore == false && (sortToggle == true)) {
        // Shoots both color of block out the front of the robot
        if((((colorSensor.get_hue() >= 0) && (colorSensor.get_hue() <= 57)) || ((colorSensor.get_hue() >= 65) && (colorSensor.get_hue() <= 359))) && (colorSensor.get_proximity() >= 50)) {
            store.move(-127);
            pros::delay(250);
        } else {
            store.brake();
        }
    } else if (sortToggle == false) {
        // Store Both Colors of Blocks
        if((((colorSensor.get_hue() >= 0) && (colorSensor.get_hue() <= 57)) || ((colorSensor.get_hue() >= 65) && (colorSensor.get_hue() <= 359))) && (colorSensor.get_proximity() >= 50)) {
            store.move(127);
            pros::delay(100);
        } else {
            store.brake();
        }
    }
}

// Actuates the bottom piston to drop a bar that gives us access to the blocks inside of the match loading tubes
void matchLoading() {
    if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) && (matchLoadDown == false)) {
        matchLoad.set_value(true);
        storage.set_value(true);
        matchLoadDown = true;
        oppStore = true;
        pros::delay(250);
    } else if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) && (matchLoadDown == true)) {
        matchLoad.set_value(false);
        storage.set_value(false);
        matchLoadDown = false;
        oppStore = false;
        pros::delay(250);
    }
}

// Actuates the middle piston to block the intake at the middle to redirect the blocks into the middle tube
void midScoring() {
    if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP) && (middleScore == false)) {
        midScore.set_value(false);
        middleScore = true;
		pros::delay(250);
    } else if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP) && (middleScore == true)) {
        midScore.set_value(true);
        middleScore = false;
        pros::delay(250);
    }
}

void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();
	chassis.setPose(0, 0, 0);
    // Sets the speed at which the color sensor scans the color
    colorSensor.set_integration_time(1);
	
	// Tasks for drivercontrol and autonomous periods
	pros::Task ([&]{
		while (true) {
			// Defines the color sensor values and prints the values on the screen
            // Sets the brightness of the color sensor's lights (This affects color detection)
            colorSensor.set_led_pwm(100);
			pros::c::optical_rgb_s_t rgb = colorSensor.get_rgb();
			double hue = colorSensor.get_hue();
			double brightness = colorSensor.get_brightness();
			double proximity = colorSensor.get_proximity();
			pros::lcd::print(1, "RGB: %f, %f, %f", rgb.red, rgb.green, rgb.blue);
			pros::lcd::print(2, "Hue: %f", hue);
			pros::lcd::print(3, "Brightness: %f", brightness);
			pros::lcd::print(4, "Proximity: %f", proximity);
			pros::lcd::print(5, "X-Value: %f", chassis.getPose().x);
			pros::lcd::print(6, "Y-Value: %f", chassis.getPose().y);
			pros::lcd::print(7, "Theta: %f", chassis.getPose().theta);
			pros::Task::delay(10);
		}
	});
    pros::Task ([&] {
        while (true) {
            // Toggles the top piston on and off and changes boolean values to help with the color sort modes
            if ((Controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) && (oppStore == false)) {
                storage.set_value(true);
                oppStore = true;
            } else if ((Controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) && (oppStore == true)) {
                storage.set_value(false);
                oppStore = false;
            }
            pros::Task::delay(200);
        }
    });
    pros::Task ([&] {
        while (true) {
            // Toggles the color of the color sorting
            if ((Controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) && (sortColorToggle == false)) {
                sortColorToggle = !sortColorToggle;
            } else if ((Controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) && (sortColorToggle == true)) {
                sortColorToggle = !sortColorToggle;
            }
            pros::Task::delay(200);
        }
    });
    pros::Task ([&] {
        while (true) {
            // Toggles the color sensing on and off
            if ((Controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) && (sortToggle == true)) {
                sortToggle = false;
            } else if ((Controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) && (sortToggle == false)) {
                sortToggle = true;
            }
            pros::Task::delay(200);
        }
    });
    pros::Task ([&] {
        while (true) {
            colorSorting();
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
    // Match Auto Half Left
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

    // Skills Auto
    // chassis.moveToPoint(0, -25, 1000, {.forwards = false});
    // chassis.moveToPoint(0, 0, 5000, {.minSpeed = 127});
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
    slew throttleSlew(8, 8);

    // Tasks for drivercontrol
    pros::Task ([&] () {
        while(true) {
            midScoring(); // Calls the midScoring function
            pros::delay(10);
        }
    });
    pros::Task ([&] () {
        while(true) {
            matchLoading(); // Calls the matchLoading function
            pros::delay(10);
        }
    });
    // This task is the issue
    pros::Task ([&] () {
        while(true) {
            colorSorting(); // Calls the colorSorting function
            pros::delay(10);
        }
    });

    while (true) {
        // Arcade control scheme
        int dir = Controller.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
        int turn = Controller.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick

        // Adds slew to the inputted controller values
        int throttleOut = throttleSlew.update(dir);

        chassis.arcade(throttleOut, turn);

        // Curvature Drive control scheme
        // chassis.curvature(dir, turn);            // Similar to arcade but turns better

        // Calling functions
        intaking();     // Calls the intaking function
        unstoring();     // Calls the unstoring function

        // How long it takes for each update of inputs
        pros::delay(20);
    }
}
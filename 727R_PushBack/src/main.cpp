#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include "liblvgl/lv_api_map.h"
#include "liblvgl/widgets/lv_label.h" // IWYU pragma: keep
#include "pros/abstract_motor.hpp"
#include "pros/adi.h" // IWYU pragma: keep
#include "pros/adi.hpp"
#include "pros/colors.hpp" // IWYU pragma: keep
#include "pros/device.hpp" // IWYU pragma: keep
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/optical.h" // IWYU pragma: keep
#include "pros/optical.hpp" // IWYU pragma: keep
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
                              360, // drivetrain rpm is 360
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
lemlib::OdomSensors sensors(&vertical_tracker, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// Lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              10 // maximum acceleration (slew)
);

// Angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              10 // maximum acceleration (slew)
);

// Creates the chassis
lemlib::Chassis chassis(drivetrain, // Drivetrain Settings
                        lateral_controller, // Lateral PID Settings
                        angular_controller, // Angular PID Settings
                        sensors // Odometry Sensors
);

// Used to toggle the top piston to allow for shooting blocks into the top storage
bool oppStore = false;

// Used to toggle the top piston to allow for scoring blocks in the middle goal
bool middleScore = false;

// Used to toggle the top piston to access the blocks at the bottom of the match loading tubes
bool matchLoadDown = false;

// Used to disable the color sensor while trying to score in the top or middle goals
bool scoring = false;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize() {
	pros::lcd::initialize();
    // Sets the brightness of the color sensor's lights (This affects color detection)
	colorSensor.set_led_pwm(100);
    // Sets the speed at which the color sensor scans the color
    colorSensor.set_integration_time(3);
}

// Intakes and outtakes the blocks
void intaking() {
	if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
		intake.move(127);
	} else if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
		intake.move(-127);
    } else {
		intake.brake();
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

    // Sorts the blocks into the storage system and up and over the robot
	while(oppStore == true) {
        // Blue Team Color Sorting
	    // if(((colorSensor.get_hue() >= 60) && (colorSensor.get_hue() <= 75)) && (colorSensor.get_proximity() >= 100)) {
	    // 	store.move(127);
		// 	pros::delay(100);
	    // } else if(((colorSensor.get_hue() >= 0) && (colorSensor.get_hue() <= 53)) && (colorSensor.get_proximity() >= 100)) {
	    //     store.move(-127);
		// 	pros::delay(500);
	    // } else {
	    //     store.brake();
        //     break;
	    // }

        // Red Team Color Sorting
        if(((colorSensor.get_hue() >= 0) && (colorSensor.get_hue() <= 53)) && (colorSensor.get_proximity() >= 100)) {
	    	store.move(127);
			pros::delay(100);
	    } else if(((colorSensor.get_hue() >= 60) && (colorSensor.get_hue() <= 75)) && (colorSensor.get_proximity() >= 100)) {
	        store.move(-127);
			pros::delay(500);
	    } else {
	        store.brake();
            break;
	    }
	}

    // Shoots both color of block out the front of the robot
    while(oppStore == false) {
	    if((((colorSensor.get_hue() >= 0) && (colorSensor.get_hue() <= 53)) || ((colorSensor.get_hue() >= 60) && (colorSensor.get_hue() <= 75))) && (colorSensor.get_proximity() >= 100)) {
	        store.move(-127);
			pros::delay(250);
	    } else {
	        store.brake();
            break;
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
    // Tasks for drivercontrol
    pros::Task ([&] () {
        while(true) {
            lv_task_handler();
            midScoring(); // Calls the midScoring function
            pros::delay(10);
        }
    });
    pros::Task ([&] () {
        while(true) {
            lv_task_handler();
            matchLoading(); // Calls the matchLoading function
            pros::delay(10);
        }
    });
    // This task is the issue
    pros::Task ([&] () {
        while(true) {
            lv_task_handler();
            opposingStorage(); // Calls the opposingStorage function
            pros::delay(10);
        }
    });
        
	while (true) {
        // Defines the color sensor values and prints the values on the screen
        pros::c::optical_rgb_s_t rgb = colorSensor.get_rgb();
        double hue = colorSensor.get_hue();
        double brightness = colorSensor.get_brightness();
        double proximity = colorSensor.get_proximity();
        pros::lcd::print(0, "RGB: %f, %f, %f", rgb.red, rgb.green, rgb.blue);
        pros::lcd::print(1, "Hue: %f", hue);
        pros::lcd::print(2, "Brightness: %f", brightness);
        pros::lcd::print(3, "Proximity: %f", proximity);

		// Arcade control scheme
		int dir = Controller.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = Controller.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		chassis.arcade(dir, turn);

        // Curvature Drive control scheme
        // chassis.curvature(dir, turn);            // Similar to arcade but turns better

        // Calling functions
        intaking();     // Calls the intaking function
        unstoring();     // Calls the unstoring function

        // How long it takes for each update of inputs
		pros::delay(20);
	}
}
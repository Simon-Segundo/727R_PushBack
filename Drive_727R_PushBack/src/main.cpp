#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/colors.hpp" // IWYU pragma: keep
#include "pros/device.hpp" // IWYU pragma: keep
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/optical.h" // IWYU pragma: keep
#include "pros/optical.hpp"

pros::Controller Controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup left_mg({1, -2, 3}, pros::MotorGearset::blue);    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
pros::MotorGroup right_mg({-4, 5, -6}, pros::MotorGearset::blue);  // Creates a motor group with forwards port 5 and reversed ports 4 & 6
pros::Motor intake(10, pros::MotorGearset::blue);
pros::Motor highMedium(17);

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_mg, // left motor group
                              &right_mg, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              360, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

// Creates an imu on port 10
pros::Imu imu(10);

// Creates a V5 vertical rotation sensor on port 9
pros::Rotation vertical_sensor(9);

// Creates a V5 horizontal rotation sensor on port 8
pros::Rotation horizontal_sensor(8);

// Creates a V5 optical sensor on port 11
pros::Optical colorSensor (11);

// Horizontal Tracking Wheel
lemlib::TrackingWheel horizontal_tracker(&horizontal_sensor, lemlib::Omniwheel::NEW_275, -2);

// Vertical Tracking Wheel
lemlib::TrackingWheel vertical_tracker(&vertical_sensor, lemlib::Omniwheel::NEW_275, 0);

lemlib::OdomSensors sensors(&vertical_tracker, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracker, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // Drivetrain Settings
                        lateral_controller, // Lateral PID Settings
                        angular_controller, // Angular PID Settings
                        sensors // Odometry Sensors
);

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	while (true) { // infinite loop
        // print measurements from the adi encoder
        pros::lcd::print(0, "X: %i", vertical_sensor.get_position());
        // print measurements from the rotation sensor
        pros::lcd::print(1, "Y: %i", horizontal_sensor.get_position());
        pros::delay(10); // delay to save resources. DO NOT REMOVE
    }
}

// Senses the color of the balls entering the intake and if a blue one tries to enter it spits it back out
void colorSensing () {
	colorSensor.set_led_pwm(100);
}

// Intakes and outtakes the balls
void intaking () {
	if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
		intake.move(100);
	} else if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
		intake.move(-100);
	} else {
		intake.brake();
	}
}

// Spins the motor at the top of the robot to score in the high or medium height goal
void scoring () {
	if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
		highMedium.move(100);
	} else if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
		highMedium.move(-100);
	} else {
		highMedium.brake();
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
void autonomous() {}

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
	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

		// Arcade control scheme
		int dir = Controller.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = Controller.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		left_mg.move(dir - turn);                      // Sets left motor voltage
		right_mg.move(dir + turn);                     // Sets right motor voltage
		intaking();
		colorSensing();
		pros::delay(20);                          // Run for 20 ms then update
	}
}
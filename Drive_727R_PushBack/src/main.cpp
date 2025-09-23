#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.h" // IWYU pragma: keep
#include "pros/adi.hpp"
#include "pros/colors.hpp" // IWYU pragma: keep
#include "pros/device.hpp" // IWYU pragma: keep
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/optical.h" // IWYU pragma: keep
#include "pros/optical.hpp"
#include <thread> // IWYU pragma: keep
#include "gif-pros/gifclass.hpp" // IWYU pragma: keep

// Used to toggle the color sensor between sensing blue or red balls
int color = 0;

// Used to stop intake from being able to run from controller input
bool sensed = false;

pros::Controller Controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup left_mg({1, -2, 3}, pros::MotorGearset::blue);    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
pros::MotorGroup right_mg({-4, 5, -6}, pros::MotorGearset::blue);  // Creates a motor group with forwards port 5 and reversed ports 4 & 6
pros::Motor intake(-10, pros::MotorGearset::blue);
pros::Motor store(16, pros::MotorGearset::blue);
pros::Motor highScore(17, pros::MotorGearset::blue);
pros::adi::DigitalOut matchLoad('A');
pros::adi::DigitalOut midScore('B');
pros::adi::DigitalOut storage('C');

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
pros::Rotation vertical_sensor(9);

// Creates a V5 optical sensor on port 11
pros::Optical colorSensor (11);

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
                                              20 // maximum acceleration (slew)
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
                                              0 // maximum acceleration (slew)
);

// Creates the chassis
lemlib::Chassis chassis(drivetrain, // Drivetrain Settings
                        lateral_controller, // Lateral PID Settings
                        angular_controller, // Angular PID Settings
                        sensors // Odometry Sensors
);

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
    colorSensor.set_integration_time(10);
    // Creates a startup animation for the code
    Gif gif("/usd/<name of your gif>.gif", lv_scr_act());
}

/* Senses the color of the balls entering the intake and if an orb of the opposing color tries to enter the storage,
   the code actuates a piston and runs a motor to put the opposing colored ball into a separate storage area */
void colorSensing () {
    // Senses if a red orb is trying to enter the storage and actuates the piston at the top of the robot to have it stored in a separate area
    if ((Controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) && (color == 0)) {
        if ((colorSensor.get_hue() >= 0) && (colorSensor.get_hue() <= 25)) {
            sensed = true;
            storage.set_value(true);
            store.move(100);
            pros::delay(5000);
        } else if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            store.move(-100);
        }
        storage.set_value(false);
        sensed = false;
        color++;
    // Senses if a blue orb is trying to enter the storage and actuates the piston at the top of the robot to have it stored in a separate area
    } else if ((Controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) && (color == 1)) {
        if ((colorSensor.get_hue() >= 50) && (colorSensor.get_hue() <= 75)) {
            sensed = true;
            storage.set_value(true);
            store.move(100);
            pros::delay(5000);
        } else if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            store.move(-100);
        }
        storage.set_value(false);
        sensed = false;
        color--;
    }
}

// Intakes and outtakes the balls
void intaking () {
	if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && sensed == false) {
		intake.move(100);
	} else if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2) && sensed == false) {
		intake.move(-100);
    } else {
		intake.brake();
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
	while (true) {
        pros::c::optical_rgb_s_t rgb = colorSensor.get_rgb();
        double hue = colorSensor.get_hue();
        double brightness = colorSensor.get_brightness();
        int proximity = colorSensor.get_proximity();
        pros::lcd::print(0, "RGB: %f %f %f", rgb.red, rgb.green, rgb.blue);
        pros::lcd::print(1, "Hue: %f", hue);
        pros::lcd::print(2, "Brightness: %f", brightness);
        pros::lcd::print(3, "Proximity: %d", proximity);
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
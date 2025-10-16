#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
<<<<<<< Updated upstream
#include "pros/abstract_motor.hpp"
=======
#include "liblvgl/lv_api_map.h"
#include "liblvgl/widgets/lv_label.h" // IWYU pragma: keep
#include "pros/abstract_motor.hpp"
#include "pros/adi.h" // IWYU pragma: keep
#include "pros/adi.hpp"
>>>>>>> Stashed changes
#include "pros/colors.hpp" // IWYU pragma: keep
#include "pros/device.hpp" // IWYU pragma: keep
#include "pros/misc.h"
#include "pros/misc.hpp"
<<<<<<< Updated upstream
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
=======
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
>>>>>>> Stashed changes
                              360, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

<<<<<<< Updated upstream
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
=======
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
>>>>>>> Stashed changes
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

<<<<<<< Updated upstream
// lateral PID controller
=======
// Lateral PID controller
>>>>>>> Stashed changes
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
<<<<<<< Updated upstream
                                              20 // maximum acceleration (slew)
);

// angular PID controller
=======
                                              10 // maximum acceleration (slew)
);

// Angular PID controller
>>>>>>> Stashed changes
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
<<<<<<< Updated upstream
                                              0 // maximum acceleration (slew)
);

// create the chassis
=======
                                              10 // maximum acceleration (slew)
);

// Creates the chassis
>>>>>>> Stashed changes
lemlib::Chassis chassis(drivetrain, // Drivetrain Settings
                        lateral_controller, // Lateral PID Settings
                        angular_controller, // Angular PID Settings
                        sensors // Odometry Sensors
);

<<<<<<< Updated upstream
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
=======
// Used to toggle the top piston to allow for shooting blocks into the top storage
bool oppStore = false;

// Used to toggle the top piston to allow for scoring blocks in the middle goal
bool middleScore = false;

// Used to toggle the top piston to access the blocks at the bottom of the match loading tubes
bool matchLoadDown = false;

// Used to disable the color sensor while trying to score in the top or middle goals
bool scoring = false;
>>>>>>> Stashed changes

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
<<<<<<< Updated upstream
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
=======

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
>>>>>>> Stashed changes
		intake.brake();
	}
}

<<<<<<< Updated upstream
// Spins the motor at the top of the robot to score in the high or medium height goal
void scoring () {
	if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
		highMedium.move(100);
	} else if (Controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
		highMedium.move(-100);
	} else {
		highMedium.brake();
	}
=======
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
>>>>>>> Stashed changes
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
<<<<<<< Updated upstream
void autonomous() {}
=======
void autonomous() {

}
>>>>>>> Stashed changes

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
<<<<<<< Updated upstream
	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs
=======
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
>>>>>>> Stashed changes

		// Arcade control scheme
		int dir = Controller.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = Controller.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
<<<<<<< Updated upstream
		left_mg.move(dir - turn);                      // Sets left motor voltage
		right_mg.move(dir + turn);                     // Sets right motor voltage
		intaking();
		colorSensing();
		pros::delay(20);                          // Run for 20 ms then update
=======
		chassis.arcade(dir, turn);

        // Curvature Drive control scheme
        // chassis.curvature(dir, turn);            // Similar to arcade but turns better

        // Calling functions
        intaking();     // Calls the intaking function
        unstoring();     // Calls the unstoring function

        // How long it takes for each update of inputs
		pros::delay(20);
>>>>>>> Stashed changes
	}
}
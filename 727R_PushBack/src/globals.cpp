#include "globals.hpp"

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
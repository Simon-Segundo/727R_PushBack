#include "globals.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/abstract_motor.hpp"

pros::Controller Controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup left_mg({-1, -2, -3}, pros::MotorGearset::blue);    // Creates a motor group with reversed ports 11 12 & 13
pros::MotorGroup right_mg({10, 9, 8}, pros::MotorGearset::blue);  // Creates a motor group with forwards port 18 19 & 20
pros::Motor intake(-6, pros::MotorGearset::blue);
pros::Motor lever(-20, pros::MotorGearset::red);
pros::adi::DigitalOut matchLoad('A');
pros::adi::DigitalOut highMid('B');
pros::adi::DigitalOut hood('C');
pros::adi::DigitalOut wing('D');

// Drivetrain settings
lemlib::Drivetrain drivetrain(&left_mg, // left motor group
                              &right_mg, // right motor group
                              11.75, // 12.75 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              800, // drivetrain rpm is the gear ratio multiplied by the rpm of the driving motor
                              2 // horizontal drift is 2 (for now)
);

// Creates an imu on port 19
pros::Imu imu(19);

// Creates a V5 vertical rotation sensor on port 17
pros::Rotation vertical_sensor(-17);

// Creates a V5 distance sensor on port 7
pros::Distance frontDistance (7);

// Creates a V5 distance sensor on port 17
pros::Distance rearDistance (16);

// Creates a V5 distance sensor on port 4
pros::Distance leftDistance (4);

// Creates a V5 distance sensor on port 5
pros::Distance rightDistance (5);

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
lemlib::ControllerSettings lateral_controller(5, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              0, // derivative gain (kD)
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

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve turnCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// Creates the chassis
lemlib::Chassis chassis(drivetrain, // Drivetrain Settings
                        lateral_controller, // Lateral PID Settings
                        angular_controller, // Angular PID Settings
                        sensors, // Odometry Sensors
                        &throttleCurve, // Slews movement and makes smaller movements more accurate
                        &turnCurve // Slews movement and makes smaller turns more accurate
);

// Used to toggle the top piston to access the blocks at the bottom of the match loading tubes
bool matchLoadDown = false;

// Used to toggle match load in the autonomous period
bool autoLoad = false;

// Used to toggle the pistons to push the tube upwards to score in high goal
bool up = false;

// Used to toggle tube up and down in the autonomous period
bool autoTube = false;

// Used to toggle the wing up and down
bool wingUp = false;

// Used to toggle the wing in the autonomous period
bool autoWing = false;

// Used to toggle the hood
bool hoodOpen = false;

// Used to toggle the hood in the autonomous period
bool autoHood = false;

// Used to swing the lever in the autonomous period
int autoLever = 2;

// Used to set the speed of the lever when the tube is down
float downSpeed = 15;

// Used to set the speed of the lever when the tube is up
int upSpeed = 75;

// Used to set the speed of the lever when the tube is up in skills
int skillsUpSpeed = 30;

// Used to set the speed of the lever when returning to the origin
int backSpeed = -100;

// Used for resetting the robot position using distance sensors
double robotPosition;

slew::slew(int accel, int decel)
    : accelRate(std::abs(accel)), decelRate(std::abs(accel)), output(0) {}

int slew::update(int target) {
    target = std::clamp(target, -127, 127);
    int delta = target - output;
    
    if (delta > 0) {
        output += std::min(delta, accelRate);
    } else if (delta < 0) {
        output += std::max(delta, -decelRate);
    }
    return output;
}
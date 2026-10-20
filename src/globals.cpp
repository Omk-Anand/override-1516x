// Copyright 2026 California High Robotics, Team 1516X
// SPDX-License-Identifier: GPL-3.0-or-later


#include "globals.h"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/distance.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include "pros/optical.hpp"
#include "pros/vision.hpp"
#include "Chassis.h"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::Motor motor3(-3, pros::MotorGears::blue);
pros::Motor motor4(-4, pros::MotorGears::blue);
pros::Motor motor5(-5, pros::MotorGears::green);
pros::Motor motor6(6, pros::MotorGears::green);
pros::Motor motor7(7, pros::MotorGears::blue);
pros::Motor motor8(8, pros::MotorGears::blue);
pros::Motor clawMotor(18, pros::MotorGears::green);

pros::MotorGroup rightMotors({6, 7, 8}, pros::MotorGears::invalid);
pros::MotorGroup leftMotors({-3, -4, -5}, pros::MotorGears::invalid);

lemlib::Drivetrain drivebase(
    &leftMotors, 
    &rightMotors, 
    10.8,
    2.78, 
    450, 
    8);

//-0.0756927599614
//6.44304298828

//0.066057675731
//6.76806367608

const float horizontal_offset = 0.0f;

pros::Imu imu(2);
pros::Rotation horizontal_tracking_sensor(9);

lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_tracking_sensor, 2, horizontal_offset, 1);

lemlib::OdomSensors sensors(nullptr, nullptr, &horizontal_tracking_wheel, nullptr, &imu);

const float lateralKP = 4.5, lateralKI = 0.0, lateralKD = 24.0;
const float angularKP = 2.6, angularKI = 0.0, angularKD = 12.0;


lemlib::ControllerSettings lateral_controller(lateralKP, // proportional gain (kP)
                                              lateralKI, // integral gain (kI)
                                              lateralKD, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              50, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              100, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

lemlib::ControllerSettings angular_controller(angularKP, // proportional gain (kP)
                                              angularKI, // integral gain (kI)
                                              angularKD, // derivative gain (kD)
                                               3, // anti windup
                                              1, // small error range, in inches
                                              50, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              100, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);





Chassis chassis(drivebase, lateral_controller, angular_controller, sensors,
                &lemlib::defaultDriveCurve, &lemlib::defaultDriveCurve);

pros::MotorGroup lift({1,-10}, pros::MotorGears::blue); // ports 1 and 10, blue cartridge, not reversed
pros::Motor toggleMotor(20, pros::v5::MotorGears::green);



pros::Distance rightDistance(15);
pros::Distance leftDistance(14);
pros::Distance frontDistance(17);
pros::Distance backDistance(16);


pros::adi::Pneumatics clawRotation('H', false);

pros::AIVision* aiVisionSensor = nullptr;

pros::Optical color_sensor(19);

Color allianceColor = Color::RED;

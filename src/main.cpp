// Copyright 2026 California High Robotics, Team 1516X
// SPDX-License-Identifier: GPL-3.0-or-later

#include "main.h"
#include "globals.h" 
#include "claw.h"
#include "RCL.h"
#include "liblvgl/core/lv_obj_pos.h"
#include "auton/autonRoutines.h"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "robodash/views/image.hpp"
#include "robodash/views/selector.hpp"
#include "colorSort.h"
#include "warnings.h"
#include <cstdint>
#include <atomic>
#include <cmath>
#include <string>
#include "MCL.h"
#include "distanceReset.h"
#include "auton/autonFunctions.h"
#include "motorDashboard.h"
#include "IntakeAntiJam.h"
#include "localization/AprilTagLocalization.hpp"
#include <memory>

LV_IMG_DECLARE(callogo);

static void update_alliance_btn(lv_obj_t* btn, Color newColor) {
    lv_obj_t* label = lv_obj_get_child(btn, 0);

    if (newColor == Color::RED) {
        lv_obj_set_style_bg_color(btn, lv_palette_main(LV_PALETTE_RED), 0);
        lv_label_set_text(label, "RED");
    } else {
        lv_obj_set_style_bg_color(btn, lv_palette_main(LV_PALETTE_BLUE), 0);
        lv_label_set_text(label, "BLUE");
    }

    std::cout << "Alliance switched to: " 
              << (newColor == Color::RED ? "Red" : "Blue") 
              << std::endl;
}

static void alliance_btn_event_handler(lv_event_t* e) {
    lv_obj_t* btn = lv_event_get_target(e);

    if (allianceColor == RED) {
        allianceColor = Color::BLUE;
    } else {
        allianceColor = Color::RED;
    }

    lv_async_call([](void* user_data){
        lv_obj_t* btn = (lv_obj_t*)user_data;
        update_alliance_btn(btn, allianceColor);
    }, btn);
}

void create_alliance_selector() {
    lv_obj_t* btn = lv_btn_create(lv_layer_top());
    lv_obj_align(btn, LV_ALIGN_BOTTOM_RIGHT, -10, -10);
    lv_obj_set_size(btn, 80, 40);

    lv_obj_set_style_bg_color(btn, lv_palette_main(LV_PALETTE_RED), 0);

    lv_obj_t* label = lv_label_create(btn);
    lv_label_set_text(label, "RED");
    lv_obj_center(label);

    lv_obj_add_event_cb(btn, alliance_btn_event_handler, LV_EVENT_CLICKED, nullptr);
}

/*
rd::Selector selector({
    {"Right 7 Split", right_auton_split},
    {"Right 7 Wing", right_7_wing},
    {"Right Rush", right_rush},
    {"Left 7 Block", left_auton_split},
    {"Left Rush", left_rush},
    {"AWP", awp_auton},
    {"Skills", skills_auton},
    {"Carry", carry_auton}
});
*/

rd::Console console;
rd::Image image = rd::Image(&callogo, "logo");

enum class DriveMode { TANK, ARCADE };
constexpr DriveMode STARTING_DRIVE_MODE = DriveMode::ARCADE; //change
std::atomic<bool> toggle_task_running{false};
std::atomic<bool> toggle_stop_requested{false};


void initialize() {
	lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
	clawMotor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	chassis.calibrate();
	motor_disconnect_warning();
    create_alliance_selector();
    init_motor_dashboard();
    chassis.setPose(0,0,0);

    static std::unique_ptr<pros::AIVision> aprilTagSensor;
    static std::unique_ptr<hololib::ApriltagLocalization> aprilTagLocalization;
    if (!aprilTagSensor && hololib::aprilTagConfig.visionPort >= 1 &&
        hololib::aprilTagConfig.visionPort <= 21) {
        aprilTagSensor = std::make_unique<pros::AIVision>(hololib::aprilTagConfig.visionPort);
        aiVisionSensor = aprilTagSensor.get();
        if (aprilTagSensor->enable_detection_types(pros::AivisionModeType::tags) == PROS_ERR ||
            aprilTagSensor->disable_detection_types(pros::AivisionModeType::colors,
                                                    pros::AivisionModeType::objects,
                                                    pros::AivisionModeType::color_merge) == PROS_ERR ||
            aprilTagSensor->set_tag_family(hololib::aprilTagConfig.tagFamily, true) == PROS_ERR) {
            aiVisionSensor = nullptr;
            aprilTagSensor.reset();
            std::cerr << "AI Vision AprilTag detection failed to initialize." << std::endl;
        } else {
            aprilTagLocalization = std::make_unique<hololib::ApriltagLocalization>(
                *aprilTagSensor, chassis, hololib::aprilTagConfig);
            if (!aprilTagLocalization->isConfigured()) {
                aprilTagLocalization.reset();
                std::cout << "AprilTag pose localization disabled: configure camera calibration and field tags."
                          << std::endl;
            } else if (!aprilTagLocalization->startTask()) {
                aprilTagLocalization.reset();
                std::cerr << "AprilTag pose localization failed to initialize." << std::endl;
            }
        }
    } else if (hololib::aprilTagConfig.visionPort == 0) {
        std::cout << "AI Vision AprilTag detection disabled: set the sensor port in AprilTagLocalization.cpp."
                  << std::endl;
    } else {
        std::cerr << "AI Vision AprilTag detection disabled: sensor port must be between 1 and 21." << std::endl;
    }

    pros::Task screen_task([&]() {
        while (true) {
            console.clear();
            
            lemlib::Pose odomPose = chassis.getPose();
            //distancePose pose = distanceReset(false);
            console.printf("X: %.3f  Y: %.3f\n", odomPose.x, odomPose.y);
            console.printf("Theta: %.3f\n\n", odomPose.theta);
			console.printf("IMU: %.1f  Pod: %.1f\n", imu.get_heading(), horizontal_tracking_sensor.get_position() / 100.0);
#if DISTANCE_SENSORS_ENABLED
			console.printf("RCL: %s  F:%d B:%d\n", rcl.isRunning() ? "ON" : "OFF",
						   static_cast<int>(frontDistance.get_distance()), static_cast<int>(backDistance.get_distance()));
			console.printf("RCL axes X:%s Y:%s\n", rcl.hasXCorrection() ? "DS" : "ODOM",
						   rcl.hasYCorrection() ? "DS" : "ODOM");
			console.printf("Left:%d Right:%d\n", static_cast<int>(leftDistance.get_distance()),
						   static_cast<int>(rightDistance.get_distance()));
#endif

            pros::delay(50);
        }
    });
}

void disabled() {
	rcl.end();
}

void competition_initialize() {
}

void autonomous() {
	left_auton();
}
//Min turn: Sensitvity at low speed
//Max Turn: Sensitivity at high speed
/*
float dynamic_steer_curve(float throttle, float steer, float min_turn_scale = 0.25, float max_turn_scale = 1.3) {
    float t = std::abs(throttle) / 127.0f;
    float s = steer / 127.0f;
    float curve = t * t; 
    float scale = min_turn_scale + (max_turn_scale - min_turn_scale) * curve;
    float s_out = s * scale;
    if(std::abs(throttle) < 3)
    {
        s_out * 2;
    }
    return s_out * 127.0f;
}
    */



void opcontrol() {
    rcl.end();
	const DriveMode drive_mode = STARTING_DRIVE_MODE;

	while (true) {
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			lift.move_voltage(12000);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			lift.move_voltage(-12000); // move lift down at 12000 voltage
		} else {
			lift.move_voltage(0); // stop lift
		}

		const bool stop_toggle = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT);
		if (stop_toggle) {
			toggle_stop_requested.store(true);
			toggleMotor.move_voltage(0);
		}

		if (!stop_toggle &&
			controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN) &&
			!toggle_task_running.exchange(true)) {
			toggle_stop_requested.store(false);
			pros::Task toggle_task([]() {
				spinToggleUntilColor(Color::BLUE, &toggle_stop_requested);
				toggle_task_running.store(false);
			});
		}

		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			claw_control(ClawAction::OPEN);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			claw_control(ClawAction::CLOSE);
		} else {
			claw_idle();
		}

		if (drive_mode == DriveMode::TANK) {
			// tank control scheme
			int left = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y); // left side from left joystick
			int right = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y); // right side from right joystick
			chassis.tank(left, right);
		} else {
			// arcade control scheme
			int dir = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y); // forward/backward from left joystick
			int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X); // turn left/right from right joystick
			chassis.arcade(dir, turn);
		}
		pros::delay(20); // run for 20 ms then update
	}
}

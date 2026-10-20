// Copyright 2026 California High Robotics, Team 1516X
// SPDX-License-Identifier: GPL-3.0-or-later

#include "globals.h" 
#include "auton/autonRoutines.h"
#include "auton/autonFunctions.h"
#include "claw.h"
#include "lemlib/chassis/chassis.hpp"
#include "ltv.h"
#include "pros/rtos.hpp"
#include "velocityController.h"
#include "paths.h"
#include "distanceReset.h"
#include <cstdlib>
#include <atomic>
#include <map>
#include <sys/types.h>
#include "RCL.h"








namespace {
bool inHueRange(double hue, double low, double high) {
    if (low <= high) {
        return hue >= low && hue <= high;
    }
    return hue >= low || hue <= high;
}

}

void spinToggleUntilColor(Color target, const std::atomic<bool>* stopRequested) {
    double low = 0;
    double high = 0;

    switch (target) {
        case Color::RED:
            low = 330;
            high = 25;
            break;
        case Color::BLUE:
            low = 190;
            high = 260;
            break;
        default:
            return;
    }

    toggleMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    toggleMotor.move_voltage(4500);
    while (stopRequested == nullptr || !stopRequested->load()) {
        double hue = color_sensor.get_hue();
        if (color_sensor.get_proximity() > 0 && inHueRange(hue, low, high)) {
            break;
        }
        pros::delay(10);
    }
    toggleMotor.move_voltage(0);
}


/*
void precompute_auton_paths(std::string path_name) {
    std::map<std::string, std::vector<std::string>> pathMap = 
    {
        {"Skills", {"skills_1", "skills_2", "skills_3", "skills_4", "parkingzone_curve"}},
        {"Right 7 Split", {"awp_1"}},
        {"Right 7", {"right_7_1"}},
        {"AWP", {"awp_2"}},
    };
    for (const auto& [key, value] : pathMap) {
    {
        if(path_name == key)
        {
            ltv.precompute_paths(pathMap[key]);
            return;
        }
    }
}
*/



void left_auton() {
    chassis.setPose(0, 0, 0);
    spinToggleUntilColor(allianceColor);

    std::atomic<bool> keep_claw_closing{true};
    std::atomic<bool> claw_close_task_finished{false};
    pros::Task claw_close_task([&]() {
        while (keep_claw_closing.load()) {
            claw_control(ClawAction::CLOSE);
            pros::delay(20);
        }
        clawMotor.move_voltage(0);
        claw_close_task_finished.store(true);
    });

    lift.move_relative(1000, 120);
    chassis.moveToPoint(0.0f, 18.0f, 1000, {.maxSpeed = 50});
    chassis.waitUntilDone();

    chassis.turnToHeading(90.0f, 1000, {.maxSpeed = 30});
    chassis.waitUntilDone();
    chassis.moveToGoal(30.0f, 18.0f, 1000, false, -1, config, {.maxSpeed = 50});
    chassis.waitUntilDone();
    lift.move_relative(-800, 120);
    chassis.waitUntilDone();
    pros::delay(1000);
    keep_claw_closing.store(false);
    while (!claw_close_task_finished.load()) {
        pros::delay(10);
    }
    claw_control(ClawAction::OPEN);
    pros::delay(1000);
    clawMotor.move_voltage(0);
    chassis.moveToPoint(5, 18, 2000, {.forwards = false});
}

void right_auton() {
    chassis.setPose(0, 0, 0);
    spinToggleUntilColor(allianceColor);

    std::atomic<bool> keep_claw_closing{true};
    std::atomic<bool> claw_close_task_finished{false};
    pros::Task claw_close_task([&]() {
        while (keep_claw_closing.load()) {
            claw_control(ClawAction::CLOSE);
            pros::delay(20);
        }
        clawMotor.move_voltage(0);
        claw_close_task_finished.store(true);
    });

    lift.move_relative(1000, 120);
    chassis.moveToPoint(0.0f, 18.0f, 1000, {.maxSpeed = 50});
    chassis.waitUntilDone();

    chassis.turnToHeading(-90.0f, 1000, {.maxSpeed = 30});
    chassis.waitUntilDone();
    chassis.moveToGoal(-30.0f, 18.0f, 1000, false, -1, config, {.maxSpeed = 50});
    chassis.waitUntilDone();
    lift.move_relative(-800, 120);
    chassis.waitUntilDone();
    pros::delay(1000);
    keep_claw_closing.store(false);
    while (!claw_close_task_finished.load()) {
        pros::delay(10);
    }
    claw_control(ClawAction::OPEN);
    pros::delay(1000);
    clawMotor.move_voltage(0);
    chassis.moveToPoint(-5.0f, 19.0f, 2000, {.forwards = false});
}

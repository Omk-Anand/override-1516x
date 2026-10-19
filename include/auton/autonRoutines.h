/*
 * Copyright 2026 California High Robotics, Team 1516X
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#pragma once

#include "main.h"
#include "colorSort.h"
#include <atomic>

extern const int longgoal_delay;
extern const int midgoal_delay;
extern const int matchload_delay;
extern const int mid_triball_delay;
extern const int longgoal_offset;
extern const int midgoal_offset;
extern const int matchload_offset;
extern const int triball_delay;
extern const int dual_ball_delay;

// Function declarations

void precompute_auton_paths(std::string path_name);
void spinToggleUntilColor(Color target, const std::atomic<bool>* stopRequested = nullptr);
void left_auton();
void right_auton();
void right_auton_split();
void left_auton_split();
void right_rush();
void left_rush();
void right_7_wing();
void left_7();
void carry_auton();
void awp_auton();
void skills_auton();

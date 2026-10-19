// Copyright 2026 California High Robotics, Team 1516X
// SPDX-License-Identifier: GPL-3.0-or-later

#include "globals.h"
#include "lemlib/util.hpp"
#include "pros/distance.hpp"
#include <algorithm>
#include <cmath>

const double MM_TO_IN = 0.0393701;
const double FIELD_WIDTH = 144.0 - 2.0 * 0.75;
const double FIELD_HEIGHT = 144.0 - 2.0 * 0.75;
const double HALF_WIDTH = FIELD_WIDTH / 2.0;
const double HALF_HEIGHT = FIELD_HEIGHT / 2.0;
const double MAX_SENSOR_RANGE = 2000 * MM_TO_IN;
const double MIN_SENSOR_RANGE = 50 * MM_TO_IN;

bool controller_screen_avilable;

struct SensorConfig {
    double forward_offset;
    double strafe_offset;
    double mounting_angle;
};


const SensorConfig front_sensor_cfg = {0.5, 0.0, 0};
const SensorConfig left_sensor_cfg  = {1.5, -4.0, -90};
const SensorConfig right_sensor_cfg = {1.5, 4.0, 90};
const SensorConfig back_sensor_cfg  = {-0.5, 0.0, 180};

struct SensorReadings {
    double dist_mm;
    int object_size;
    int confidence;
};

struct distancePose {
    double x;
    double y;
    bool using_odom_x;
    bool using_odom_y;
};

distancePose calculateGlobalPosition(
    const SensorReadings& front_data,
    const SensorReadings& left_data,
    const SensorReadings& right_data,
    const SensorReadings& back_data,
    double heading_deg,
    float heading_tolerance = 40.0)
{
    (void)heading_tolerance;
    lemlib::Pose current_pose = chassis.getPose();
    double correction_x_sum = 0.0;
    double correction_y_sum = 0.0;
    int correction_x_count = 0;
    int correction_y_count = 0;

    struct SensorData { SensorConfig cfg; double dist_in; };
    const SensorData sensors[] = {
        {front_sensor_cfg, front_data.dist_mm * MM_TO_IN},
        {left_sensor_cfg,  left_data.dist_mm * MM_TO_IN},
        {right_sensor_cfg, right_data.dist_mm * MM_TO_IN},
        {back_sensor_cfg,  back_data.dist_mm * MM_TO_IN}
    };

    const double heading_rad = lemlib::degToRad(heading_deg);
    for (const SensorData& sensor : sensors) {
        if (sensor.dist_in < MIN_SENSOR_RANGE || sensor.dist_in >= MAX_SENSOR_RANGE) {
            continue;
        }

        const double sensor_heading = heading_rad + lemlib::degToRad(sensor.cfg.mounting_angle);
        const double ray_x = std::sin(sensor_heading);
        const double ray_y = std::cos(sensor_heading);
        const double origin_x = current_pose.x + sensor.cfg.forward_offset * std::sin(heading_rad)
                              + sensor.cfg.strafe_offset * std::cos(heading_rad);
        const double origin_y = current_pose.y + sensor.cfg.forward_offset * std::cos(heading_rad)
                              - sensor.cfg.strafe_offset * std::sin(heading_rad);

        double wall_distance_x = 1e9;
        double wall_distance_y = 1e9;
        if (ray_x > 1e-6) wall_distance_x = (HALF_WIDTH - origin_x) / ray_x;
        else if (ray_x < -1e-6) wall_distance_x = (-HALF_WIDTH - origin_x) / ray_x;
        if (ray_y > 1e-6) wall_distance_y = (HALF_HEIGHT - origin_y) / ray_y;
        else if (ray_y < -1e-6) wall_distance_y = (-HALF_HEIGHT - origin_y) / ray_y;

        const double predicted_distance = std::min(wall_distance_x, wall_distance_y);
        if (predicted_distance <= 0.0 || predicted_distance >= MAX_SENSOR_RANGE) {
            continue;
        }

        const double residual = predicted_distance - sensor.dist_in;
        if (std::abs(residual) > 6.0) {
            continue;
        }

        if (std::abs(ray_x) > 0.1) {
            correction_x_sum += residual * ray_x;
            ++correction_x_count;
        }
        if (std::abs(ray_y) > 0.1) {
            correction_y_sum += residual * ray_y;
            ++correction_y_count;
        }
    }

    distancePose pose;
    pose.x = current_pose.x + (correction_x_count ? correction_x_sum / correction_x_count : 0.0);
    pose.y = current_pose.y + (correction_y_count ? correction_y_sum / correction_y_count : 0.0);
    pose.using_odom_x = correction_x_count == 0;
    pose.using_odom_y = correction_y_count == 0;
    return pose;
}


distancePose distanceReset(bool setPose = false, bool filter = true, float filter_range = 3.5, float heading_tolerance = 40.0) {
    double heading_deg = chassis.getPose().theta;

#if DISTANCE_SENSORS_ENABLED
    const SensorReadings front_data = {(double)frontDistance.get_distance(), frontDistance.get_object_size(), frontDistance.get_confidence()};
    const SensorReadings left_data = {(double)leftDistance.get_distance(), leftDistance.get_object_size(), leftDistance.get_confidence()};
    const SensorReadings right_data = {(double)rightDistance.get_distance(), rightDistance.get_object_size(), rightDistance.get_confidence()};
    const SensorReadings back_data = {(double)backDistance.get_distance(), backDistance.get_object_size(), backDistance.get_confidence()};
    distancePose pose = calculateGlobalPosition(front_data, left_data, right_data, back_data, heading_deg, heading_tolerance);
#else
    const SensorReadings invalid_reading = {10000, 0, 0};
    distancePose pose = calculateGlobalPosition(invalid_reading, invalid_reading, invalid_reading, invalid_reading, heading_deg, heading_tolerance);
#endif
    if(filter)
    {
        if(std::abs(pose.x - chassis.getPose().x) > filter_range)
        {
            pose.x = chassis.getPose().x;
            pose.using_odom_x = true;
            std::cout << "Filtered X" << std::endl;
        }
        if(std::abs(pose.y - chassis.getPose().y) > filter_range)
        {
            pose.y = chassis.getPose().y;
            pose.using_odom_y = true;
            std::cout << "Filtered Y" << std::endl;
        }
        
    }
    if(setPose) {
        chassis.setPose(pose.x, pose.y, chassis.getPose().theta);
        pros::delay(2);
    }

    std::cout << "Distance Reset Pose: " << pose.x << ", " << pose.y << ", using_odom_x: " << pose.using_odom_x << ", using_odom_y: " << pose.using_odom_y << std::endl;
    return pose;
}

distancePose distanceReset(bool left_use, bool right_use, bool front_use, bool back_use, bool setPose) {
    double heading_deg = chassis.getPose().theta; 

#if DISTANCE_SENSORS_ENABLED
    const int invalid_dist_mm = 10000;
    const int invalid_confidence = 0;
    SensorReadings front_data = front_use
        ? SensorReadings{(double)frontDistance.get_distance(), frontDistance.get_object_size(), frontDistance.get_confidence()}
        : SensorReadings{invalid_dist_mm, 0, invalid_confidence};
    SensorReadings left_data = left_use
        ? SensorReadings{(double)leftDistance.get_distance(), leftDistance.get_object_size(), leftDistance.get_confidence()}
        : SensorReadings{invalid_dist_mm, 0, invalid_confidence};
    SensorReadings right_data = right_use
        ? SensorReadings{(double)rightDistance.get_distance(), rightDistance.get_object_size(), rightDistance.get_confidence()}
        : SensorReadings{invalid_dist_mm, 0, invalid_confidence};
    SensorReadings back_data = back_use
        ? SensorReadings{(double)backDistance.get_distance(), backDistance.get_object_size(), backDistance.get_confidence()}
        : SensorReadings{invalid_dist_mm, 0, invalid_confidence};
    distancePose pose = calculateGlobalPosition(front_data, left_data, right_data, back_data, heading_deg);
#else
    (void)left_use;
    (void)right_use;
    (void)front_use;
    (void)back_use;
    const SensorReadings invalid_reading = {10000, 0, 0};
    distancePose pose = calculateGlobalPosition(invalid_reading, invalid_reading, invalid_reading, invalid_reading, heading_deg);
#endif
    
    if(setPose) {
        chassis.setPose(pose.x, pose.y, chassis.getPose().theta);
    }

    return pose;
}
#pragma once

#include "Chassis.h"
#include "pros/ai_vision.hpp"
#include "pros/rtos.hpp"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include <array>
#include <cstdint>
#include <memory>
#include <vector>

namespace hololib {

struct FieldTag {
    int tagId;
    // Tag-center coordinates in the LemLib field frame; Z is height above the field.
    std::array<float, 3> positionInches;
    // Heading of the tag's outward normal, in radians.
    float yawRadians;
};

struct AprilTagConfig {
    // Leave at 0 until the connected AI Vision smart port is known.
    uint8_t visionPort = 0;
    pros::AivisionTagFamily tagFamily = pros::AivisionTagFamily::tag_21H7;
    float tagSizeInches = 1.5f;
    // Calibrated camera intrinsics, in pixels.
    float focalLengthX = 0.0f;
    float focalLengthY = 0.0f;
    float principalPointX = 0.0f;
    float principalPointY = 0.0f;
    // OpenCV order: k1, k2, p1, p2, k3.
    std::array<float, 5> distortion{};
    // Camera translation in inches and yaw relative to the robot.
    float cameraOffsetXInches = 0.0f;
    float cameraOffsetYInches = 0.0f;
    float cameraYawOffsetRadians = 0.0f;
    std::vector<FieldTag> fieldTags;
};

extern const AprilTagConfig aprilTagConfig;

class ApriltagLocalization {
public:
    ApriltagLocalization(pros::AIVision& visionSensor, Chassis& chassis, const AprilTagConfig& config);

    bool isConfigured() const;
    bool startTask(uint32_t periodMs = 20);
    lemlib::Pose getPoseFromTag(bool useRadians = false);

private:
    struct IPPEOutput {
        Eigen::Vector3f rvec1;
        Eigen::Vector3f tvec1;
        Eigen::Vector3f rvec2;
        Eigen::Vector3f tvec2;
        float reprojection_error1;
        float reprojection_error2;
        bool valid;
    };

    struct SortedPoses {
        Eigen::Matrix4f M1;
        Eigen::Matrix4f M2;
        float err1;
        float err2;
    };

    static std::array<Eigen::Vector2f, 4> generateSquareObjectCorners2D(float squareLength);
    static std::array<Eigen::Vector3f, 4> generateSquareObjectCorners3D(float squareLength);
    static Eigen::Matrix3f rotateVec2ZAxis(const Eigen::Vector3f& vector);
    static Eigen::Vector3f rot2vec(const Eigen::Matrix3f& rotation);
    static Eigen::Vector3f computeTranslation(const std::array<Eigen::Vector2f, 4>& objectPoints,
                                              const std::array<Eigen::Vector2f, 4>& normalizedImagePoints,
                                              const Eigen::Matrix3f& rotation);
    static std::pair<Eigen::Matrix3f, Eigen::Matrix3f> computeRotations(
        float j00, float j01, float j10, float j11, float p, float q);
    static std::pair<Eigen::Matrix4f, Eigen::Matrix4f> solveCanonicalForm(
        const std::array<Eigen::Vector2f, 4>& canonicalObjectPoints,
        const std::array<Eigen::Vector2f, 4>& normalizedInputPoints,
        const Eigen::Matrix3f& homography);
    static Eigen::Matrix3f homographyFromSquarePoints(const std::array<Eigen::Vector2f, 4>& targetPoints,
                                                       float halfLength);
    static std::array<Eigen::Vector2f, 4> projectPoints(
        const std::array<Eigen::Vector3f, 4>& objectPoints,
        const Eigen::Matrix3f& rotation,
        const Eigen::Vector3f& translation,
        const Eigen::Matrix3f& cameraMatrix,
        const Eigen::Matrix<float, 5, 1>& distortion);
    static float evalReprojError(const std::array<Eigen::Vector3f, 4>& objectPoints,
                                 const std::array<Eigen::Vector2f, 4>& imagePoints,
                                 const Eigen::Matrix3f& cameraMatrix,
                                 const Eigen::Matrix<float, 5, 1>& distortion,
                                 const Eigen::Matrix4f& transform);
    static SortedPoses sortPosesByReprojError(const std::array<Eigen::Vector3f, 4>& objectPoints,
                                               const std::array<Eigen::Vector2f, 4>& imagePoints,
                                               const Eigen::Matrix3f& cameraMatrix,
                                               const Eigen::Matrix<float, 5, 1>& distortion,
                                               const Eigen::Matrix4f& firstPose,
                                               const Eigen::Matrix4f& secondPose);
    static std::array<Eigen::Vector2f, 4> undistortPoints5Coeffs(
        const std::array<Eigen::Vector2f, 4>& source,
        const Eigen::Matrix3f& cameraMatrix,
        const Eigen::Matrix<float, 5, 1>& distortion,
        int maxIterations = 5,
        float epsilon = 0.01f);
    static IPPEOutput IPPESquare(const std::array<Eigen::Vector2f, 4>& imagePoints,
                                 const Eigen::Matrix3f& cameraMatrix,
                                 const Eigen::Matrix<float, 5, 1>& distortion,
                                 float squareLength);

    lemlib::Pose estimateRobotPose(const IPPEOutput& ippeOutput,
                                   const lemlib::Pose& odometryPoseRadians,
                                   int tagId) const;
    void update(uint32_t periodMs);

    pros::AIVision& visionSensor;
    Chassis& chassis;
    Eigen::Matrix3f cameraMatrix;
    Eigen::Matrix<float, 5, 1> distCoeffs;
    pros::AivisionTagFamily tagFamily;
    float tagSizeInches;
    Eigen::Vector2f visionSensorOffset;
    float visionSensorYawOffset;
    std::vector<FieldTag> fieldTags;
    std::unique_ptr<pros::Task> task;
};

} // namespace hololib

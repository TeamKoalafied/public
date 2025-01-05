//==============================================================================
// AlignToAmpCommand.cpp
//==============================================================================

#include "AlignToAmpCommand.h"

#include "LogPathPlannerCommand.h"

#include "../RobotConfiguration.h"
#include "../subsystems/Vision.h"
#include "../subsystems/SwerveDrivebase.h"

#include <pathplanner/lib/auto/AutoBuilder.h>

#include <iostream>

using namespace pathplanner;
namespace RC = RobotConfiguration;

std::optional<frc2::CommandPtr> AlignToAmpCommand::DoAlignToAmp(SwerveDrivebase& drivebase) {
    // Get the April tag for the amp as it is directly above the middle of the amp
    const frc::AprilTag* amp_april_tag = drivebase.GetVision().GetAprilTag(drivebase.GetVision().GetAmpTagId());
    if (amp_april_tag == nullptr) {
        // This should be impossible
        return std::nullopt;
    }

    // Calculate the posed for the robot that it directly in front of the amp. The robot is positioned by
    // its centre and so the position needs to be half the drivebase length in front of the amp. The tramper
    // is on the front of the robot was it must be facing the amp (180 degrees from the April tag).
    frc::Pose2d tag_pose = amp_april_tag->pose.ToPose2d();
    frc::Translation2d robot_position = tag_pose.Translation() + frc::Translation2d(RC::kDriveBaseBumperLength / 2, tag_pose.Rotation());
    frc::Rotation2d robot_rotation = tag_pose.Rotation().RotateBy(180_deg);
    frc::Pose2d robot_amp_pose(robot_position, robot_rotation);

    // Get the current robot pose and calculate the distance and angle delta to move
    frc::Pose2d robot_current_pose = drivebase.GetPose();
    units::meter_t distance = robot_current_pose.Translation().Distance(robot_amp_pose.Translation());
    units::degree_t angle_delta = (robot_current_pose.Rotation() - robot_amp_pose.Rotation()).Degrees();

    // Log everything about the targeting calculation
    std::cout << "\nAmp Align Targeting -----------------------------------------------------------\n";
    std::cout << "    Tag X: " << tag_pose.X().value() << " Y: " << tag_pose.Y().value() << " Rot: " << tag_pose.Rotation().Degrees().value() << "\n";
    std::cout << "    Amp X: " << robot_amp_pose.X().value() << " Y: " << robot_amp_pose.Y().value() << " Rot: " << robot_amp_pose.Rotation().Degrees().value() << "\n";
    std::cout << "    Robot X: " << robot_current_pose.X().value() << " Y: " << robot_current_pose.Y().value() << " Rot: " << robot_current_pose.Rotation().Degrees().value() << "\n";
    std::cout << "Distance: " << distance.value() << " Angle Delta: " << angle_delta.value() << "\n";

    // If the distance or angle to turn is too great return 'nullopt' to indicate that 
    const units::meter_t MAX_DISTANCE = 2_m;
    const units::degree_t MAX_ANGLE = 45_deg;
    if (distance > MAX_DISTANCE || units::math::abs(angle_delta) > MAX_ANGLE) {
        return std::nullopt;
    }


    units::meters_per_second_t max_velocity = 1_mps;
    units::meters_per_second_squared_t max_acceleration = 0.5_mps_sq;

    // Calculate a path from the current pose to the pose in front of the amp
    std::shared_ptr<PathPlannerPath> path;
    std::vector<frc::Pose2d> poses{
        robot_current_pose,
        robot_amp_pose
    };
    std::vector<frc::Translation2d> bezier_points = PathPlannerPath::bezierFromPoses(poses);
    path = std::make_shared<PathPlannerPath>(
        bezier_points,
        PathConstraints(max_velocity, max_acceleration, 360_deg_per_s, 720_deg_per_s_sq), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
        GoalEndState(0.0_mps, frc::Rotation2d(robot_rotation)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
    );
    path->preventFlipping = true;

    // Build the auto path command and return it
    return AutoBuilder::followPath(path);
}

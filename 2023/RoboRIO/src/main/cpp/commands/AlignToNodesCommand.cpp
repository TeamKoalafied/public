//==============================================================================
// AlignToNodesCommand.cpp
//==============================================================================

#include "AlignToNodesCommand.h"
#include "../RobotConfiguration.h"

namespace RC = RobotConfiguration;

frc::Trajectory* AlignToNodesCommand::CreateTrajectory(frc::SwerveDriveKinematics<4>& kinematics, frc::Pose2d pose) {
    frc::Trajectory* trajectory = new frc::Trajectory;
    // Set up config for trajectory
    frc::TrajectoryConfig config(RC::kMaxSpeed, RC::kMaxAcceleration);
    // Add kinematics to ensure max speed is actually obeyed
    config.SetKinematics(kinematics);

    *trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        pose,
        {frc::Translation2d{pose.X(), pose.Y() + 0.5_m}},
        frc::Pose2d{pose.X(), pose.Y() + 1.0_m, pose.Rotation()},
        // Pass the config
        config);

    return trajectory;
}

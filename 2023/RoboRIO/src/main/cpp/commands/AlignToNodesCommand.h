//==============================================================================
// AlignToNodesCommand.h
//==============================================================================

#pragma once

#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>


class AlignToNodesCommand {
public:

    // Uses the robot kinematics to generate a trajectory
    static frc::Trajectory* CreateTrajectory(frc::SwerveDriveKinematics<4>& kinematics, frc::Pose2d pose);

};
//==============================================================================
// SwerveFollowerCommand.h
//==============================================================================

#pragma once

#include <frc/Timer.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <units/length.h>
#include <units/time.h>
#include <units/voltage.h>

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "../pathfollower/SwerveTrajectory.h"
#include "../pathfollower/SwerveFollower.h"


class SwerveFollowerCommand : public frc2::CommandHelper<frc2::Command, SwerveFollowerCommand> {
public:

    SwerveFollowerCommand(SwerveTrajectory trajectory, frc::SwerveDriveKinematics<4>& kinematics,
                          std::function<frc::Pose2d()> get_pose_function,
                          std::function<void(std::array<SwerveFollowerModuleState, 4>)> set_module_states_function,
                          std::initializer_list<frc2::Subsystem*> requirements);

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;

 private:
    //==========================================================================
    // Member Variables
    //SwerveTrajectory m_trajectory;
    frc::SwerveDriveKinematics<4>& m_kinematics;
    std::function<frc::Pose2d()> m_get_pose_function;
    std::function<void(std::array<SwerveFollowerModuleState, 4>)> m_set_module_states_function;
    frc::Timer m_timer;
    SwerveFollower m_follower;
};

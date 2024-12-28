//==============================================================================
// TestTurnCommand.h
//==============================================================================

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>

#include "../subsystems/SwerveDrivebase.h"


class TestTurnCommand : public frc2::CommandHelper<frc2::Command, TestTurnCommand> {

 public:
    //==========================================================================
    // Construction

    // Constructor
    //
    // drivebase - Drivebase for the command to operate on
    explicit TestTurnCommand(SwerveDrivebase* drivebase);


    //==========================================================================
    // frc::Command Virtual Functions
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    //==========================================================================

    //==========================================================================
    // Static Setup

    // Setup shuffleboard controls for the parameters of this command
    static void SetupDashboard();

 private:
    units::meters_per_second_t m_speed;
    units::second_t m_time_before_turn;
    units::second_t m_time_after_turn;
    frc::Rotation2d m_turn;
    SwerveDrivebase* m_drivebase;
    frc::Timer m_timer;
    bool m_turned;
};
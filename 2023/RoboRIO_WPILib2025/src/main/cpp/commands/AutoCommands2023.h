//==============================================================================
// AutoCommands2023.h
//==============================================================================

#pragma once

#include "IAutonomousProvider.h"

#include <units/length.h>

// Create the auto commands for the 2023 game Charged Up
class AutoCommands2023 : public IAutonomousProvider {
public:
    //==========================================================================
    // Construction

    // Construtor
    AutoCommands2023();

    //==========================================================================
    // Virtual Functions from IAutonomousProvider
    virtual const char* Name() override;
    virtual int SetupDashboard(frc::ShuffleboardTab& auto_tab, int x_pos, int y_pos) override;
    virtual frc2::CommandPtr CreateAutonomousCommand(SwerveDrivebase& drivebase, Manipulator& manipulator) override;
    //==========================================================================

private:
    //==========================================================================
    // Creating Commands

    // Create a command to place a single cone at the high level
    //
    // drivebase - Robot drivebase to control
    // manipulator - Robot manipulator to control
    frc2::CommandPtr CreatePlaceConeCommand(SwerveDrivebase& drivebase, Manipulator& manipulator);

    // Create a command to place a single cone at the high level and then back away a give distance
    //
    // drivebase - Robot drivebase to control
    // manipulator - Robot manipulator to control
    // distance - Distance to back away after placing the cone
    frc2::CommandPtr CreatePlaceConeBackAwayCommand(SwerveDrivebase& drivebase, Manipulator& manipulator,
                                                    units::inch_t distance);

    // Create a command for auto balancing
    //
    // drivebase - Robot drivebase to control
    frc2::CommandPtr CreateBalanceCommand(SwerveDrivebase& drivebase);
};
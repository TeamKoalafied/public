//==============================================================================
// AutoCommands2024.h
//==============================================================================

#pragma once

#include "IAutonomousProvider.h"

#include <units/length.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/time.h>

// Create the auto commands for the 2023 game Charged Up
class AutoCommands2024 : public IAutonomousProvider {
public:
    //==========================================================================
    // Construction

    // Construtor
    AutoCommands2024();

    //==========================================================================
    // Virtual Functions from IAutonomousProvider
    virtual const char* Name() override;
    virtual int SetupDashboard(frc::ShuffleboardTab& auto_tab, int x_pos, int y_pos) override;
    virtual frc2::CommandPtr CreateAutonomousCommand(SwerveDrivebase& drivebase, Manipulator& manipulator) override;
    //==========================================================================

private:
    //==========================================================================
    // Creating Commands

    frc2::CommandPtr CreateAutoNoteCommand(SwerveDrivebase& drivebase, Manipulator& manipulator, std::string auto_name);
    frc2::CommandPtr CreatePathPlannerAutoCommand(SwerveDrivebase& drivebase, std::string auto_name);

    //frc2::CommandPtr CreatePathNoteCommand(SwerveDrivebase& drivebase, Manipulator& manipulator, std::string path_name, units::second_t delay);

    // frc2::CommandPtr CreatePathPlannerPathCommand(SwerveDrivebase& drivebase, std::string path_name, units::second_t delay,
    //                                               units::meters_per_second_t max_velocity,
    //                                               units::meters_per_second_squared_t m_max_acceleration);

};
//==============================================================================
// AutonomousCommand.h
//==============================================================================

#pragma once

#include <frc2/command/Command.h>

class SwerveDrivebase;
class Manipulator;
namespace frc { class ShuffleboardTab; }

// AutonomousCommand namespace contains the functions to setup the dashboard for autonomous
// and for creating the autonomous command.
//
// During actual competition autonomous is used for the appropriate auto routine for the game
// beig played. During development, demonstrations and pratice autonomous is used for a range of tasks
// such as tuning, characterising and others.
namespace AutonomousCommand {
    // Create the shuffleboard interface for select the exact autonomous command to use during competition
    //
    // auto_tab - Shuffleboard tab to add any UI for selecting the auto
    // x_pos - X position to add the UI at
    // y_pos - Y position to add the UI at
    void SetupGameAutonomousShuffleboard(frc::ShuffleboardTab& auto_tab, int x_pos, int y_pos);

    // Create the shuffleboard interface for select the exact autonomous command to use for development
    void SetupDevAutonomousShuffleboard();

    // Create the autonomous command depending on the approriate shuffleboard settings
    //
    // drivebase - Drivebase to use for the command
    // manipulator - Manipulator to use for the command
    frc2::CommandPtr CreateAutonomousCommand(SwerveDrivebase& drivebase, Manipulator& manipulator);
};
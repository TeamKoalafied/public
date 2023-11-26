//==============================================================================
// IAutonomousProvider.h
//==============================================================================

#pragma once

#include <frc2/command/Command.h>

class SwerveDrivebase;
class Manipulator;
namespace frc { class ShuffleboardTab; }


// Interface to define a 'provider' of autonomous commands. These allows for flexibility and extension
// without have to hack the code and recompile all the time. During testing or demons there may be
// multiple of these. During competition only the autonomous commands for the current game will be present.
class IAutonomousProvider {
public:
    // Get the name of this provider
    virtual const char* Name() = 0;

    // Setup the dashboard. This provide can use as much width as it likes starting at the given
    // position and also going down as far as required.
    //
    // auto_tab - Shuffleboard tab to add any UI for this provider
    // x_pos - X position to add the UI at
    // y_pos - Y position to add the UI at
    //
    // Returns the width of requird, which can be zero for none
    virtual int SetupDashboard(frc::ShuffleboardTab& auto_tab, int x_pos, int y_pos) = 0;

    // Create the autonomous command that the user has selected
    //
    // drivebase - Robot drivebase to control
    // manipulator - Robot manipulator to control
    virtual frc2::CommandPtr CreateAutonomousCommand(SwerveDrivebase& drivebase, Manipulator& manipulator) = 0;

    // Width allowed for the UI of this provider on the Shuffleboard
    static const int SHUFFLEBOARD_WIDTH = 6;
};


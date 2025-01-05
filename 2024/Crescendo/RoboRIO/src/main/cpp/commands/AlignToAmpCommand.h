//==============================================================================
// AlignToAmpCommand.h
//==============================================================================

#pragma once

#include <frc2/command/CommandPtr.h>
#include <optional>

class SwerveDrivebase;


namespace AlignToAmpCommand {
    // Create a path following command to move the robot from its current position to directly in front
    // of the amp, if the robot is already relatively close.
    //
    // drivebase - Drivebase to create the command for
    //
    // Returns a path following command, or nullopt if the robot is too far away
    std::optional<frc2::CommandPtr> DoAlignToAmp(SwerveDrivebase& drivebase);
}


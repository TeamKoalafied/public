//==============================================================================
// AutonomousCommand.h
//==============================================================================

#ifndef AutonomousCommand_H
#define AutonomousCommand_H

namespace frc { class Command; }
class RobotPath;
class Point2D;


// Namespace with functions for setting up the command runs the autonomous period for the robot.
namespace AutonomousCommand {

    //==========================================================================
	// Dashboard Setup

	// Set up controls on the dashboard for choosing autonomous parameters
	void SetupAutonomousDashboard();

    // Create the autonomous command to run the currently selected strategy
    //
    // Returns the command. Ownership is taken by the caller. May return nullptr for no autonomous.
    frc::Command* CreateAutonomousCommand();

    // Update the dashboard
    void UpdateDashboard();


    //==========================================================================
    // Path Creation

    // Create a path to shoot our initial balls and move foreward off the tarmac
    //
    //  delay_s - Delay in seconds before the path begins
    //
    // Returns the path
    RobotPath* CreateShootAndMoveForewardPath(double delay_s);

    // Create a path to move forward to pick up a second ball and then shoot both balls
    //
    //  delay_s - Delay in seconds before the path begins
    //
    // Returns the path
    RobotPath* CreateTwoBallPath(double delay_s);


    //==========================================================================
    // Path Creation Helpers

    // Add a segment to the given robot path to delay for ta given time
    //
    // robot_path - Path to add the segment to
    // delay_s - Delay in seconds
    void AddDelaySegment(RobotPath* robot_path, double delay_s);

    // Add a segment to the given robot path for a given mechanism command
    //
    // robot_path - Path to add the segment to
    // command - Mechanism command to add
    void AddCommandSegment(RobotPath* robot_path, const char* command);

}
#endif  // AutonomousCommand_H

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
    // Returns the command. Ownership is taken by the caller.
    frc::Command* CreateAutonomousCommand();

    // Update the dashboard
    void UpdateDashboard();


    //==========================================================================
    // Path Creation

    // Create a path to shoot our initial 3 balls and do nothing else
    //
    //  delay_s - Delay in seconds before the path begins
    //  trench_offset_inch - Offset of the robot from directly in front of the target.
    //      Towards the trench is positive.
    //
    // Returns the path
    RobotPath* CreateShootPath(double delay_s, double trench_offset_inch);

    // Create a path to shoot our initial 3 balls and move backward out of the way (towards target)
    //
    //  delay_s - Delay in seconds before the path begins
    //  trench_offset_inch - Offset of the robot from directly in front of the target.
    //      Towards the trench is positive.
    //
    // Returns the path
    RobotPath* CreateShootAndMoveBackwardPath(double delay_s, double trench_offset_inch);

    // Create a path to shoot our initial 3 balls and move foreward out of the way (away from target)
    //
    //  delay_s - Delay in seconds before the path begins
    //  trench_offset_inch - Offset of the robot from directly in front of the target.
    //      Towards the trench is positive.
    //
    // Returns the path
    RobotPath* CreateShootAndMoveForewardPath(double delay_s, double trench_offset_inch);

    // Create a path to shoot our initial 3 balls then get 3 balls from the trench and shoot them
    //
    //  delay_s - Delay in seconds before the path begins
    //  trench_offset_inch - Offset of the robot from directly in front of the target.
    //      Towards the trench is positive.
    //
    // Returns the path
    RobotPath* CreateShootAndTrenchPath(double delay_s, double trench_offset_inch);


    // Create a path to shoot our initial 3 balls then get 3 balls from under the shield generator and shoot them
    //
    //  delay_s - Delay in seconds before the path begins
    //  trench_offset_inch - Offset of the robot from directly in front of the target.
    //      Towards the trench is positive.
    //
    // Returns the path
    RobotPath* CreateShootAndShieldPath(double delay_s, double trench_offset_inch);


    //==========================================================================
    // Path Creation Helpers

    // Add a segment to the given robot path to delay for ta given time
    //
    // robot_path - Path to add the segment to
    // delay_s - Delay in seconds
    void AddDelaySegment(RobotPath* robot_path, double delay_s);

    // Add a segment to the given robot path to shoot the 3 balls
    //
    // robot_path - Path to add the segment to
    void AddShootSegment(RobotPath* robot_path);

    // Add a segment to the given robot path to rotate to the target
    //
    // robot_path - Path to add the segment to
    void AddRotateToTargetSegment(RobotPath* robot_path);

    // Get the starting position and offset of the robot from the trench offset, assuming that the
    // robot is pointed towards the centre of the inner port.
    //
    //  trench_offset_inch - Offset of the robot from directly in front of the target.
    //      Towards the trench is positive.
    // start - Returns the starting position of the robot. Origin is directly in front of the power port
    //      on the initiation line. Heading 0 (x axis) is directly away from the power port.
    // direction - Returns the direction vector of the robot, in the same coordinates as 'start' 
    void GetStartPosition(double trench_offset_inch, Point2D& start, Point2D& direction);

    // Get the heading for the robot to be pointed towards the centre of the inner port
    //
    // position - The position of the robot. Origin is directly in front of the power port
    //      on the initiation line. Heading 0 (x axis) is directly away from the power port.
    //
    // Returns the required heading of the robot  in degrees
    double GetTargetHeadingDegrees(const Point2D& position);
}
#endif  // AutonomousCommand_H

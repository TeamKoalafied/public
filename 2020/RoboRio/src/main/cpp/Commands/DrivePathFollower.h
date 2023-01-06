//==============================================================================
// DrivePathFollower.h
//==============================================================================

#pragma once

#include <frc/commands/Command.h>
#include <frc/Joystick.h>
class PathFollower;
class RobotPath;


class DrivePathFollower : public frc::Command {
public:
    //==========================================================================
    // Construction and Destruction

	// Constuctor
	//
	// path_follower - Path follower for driving the path this object takes ownership)
	DrivePathFollower(PathFollower* path_follower);

	// Destructor
	~DrivePathFollower();


    //==========================================================================
	// Function Overrides from frc::Command
	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;
    //==========================================================================

    //==========================================================================
	// Static Joystick Testing Control Functions

	// Do joystick control of following Pathfinder paths for testing
	//
	// joystick - The joystick to query for user input
	static void DoJoystickTestControl(frc::Joystick* joystick);

private:
    //==========================================================================
	// Static Joystick Testing Implementation Functions

	static PathFollower* CreatePathfinderFollower(RobotPath* robot_path);
	static PathFollower* CreatePathPointsFollower(RobotPath* robot_path);
	static RobotPath* CreateVisionPathFromDashBoard(double max_velocity, double max_acceleration);
	static RobotPath* CreateStraightPath(double max_velocity, double max_acceleration);
	static RobotPath* CreateBall1Path(double max_velocity, double max_acceleration);
	static RobotPath* CreateBall2Path(double max_velocity, double max_acceleration);

    //==========================================================================
    // Member Variables

	PathFollower* m_path_follower;					// Path follower for driving the path (owned by this object)

    static DrivePathFollower* ms_test_command;		// Command being used during joystick test control
};


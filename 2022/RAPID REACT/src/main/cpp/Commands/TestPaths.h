//==============================================================================
// TestPaths.h
//==============================================================================

#pragma once

class RobotPath;

namespace TestPaths
{

    //==========================================================================
	// Static Joystick Testing Implementation Functions

	RobotPath* CreateTestPath(int pov_angle, double max_velocity, double max_acceleration);




	RobotPath* CreateVisionPathFromDashBoard(double max_velocity, double max_acceleration);
	RobotPath* CreateStraightPath(double max_velocity, double max_acceleration, double distance);
	RobotPath* CreateTestCirclePath(double max_velocity, double max_acceleration);
}


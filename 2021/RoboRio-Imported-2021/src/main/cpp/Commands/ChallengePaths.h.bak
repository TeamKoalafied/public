// ChallengePaths.h
#include <frc/Joystick.h>
class PathSegment;
class Point2D;
class RobotPath;

class ChallengePaths
{
public:
    //==========================================================================
	// Static Joystick Testing Control Functions

	// Do joystick control of following Pathfinder paths for testing
	//
	// joystick - The joystick to query for user input
	//static void DoJoystickTestControl(frc::Joystick* joystick);

	static RobotPath* CreateTestPath(int pov_angle, double max_velocity, double max_acceleration);

   	//==========================================================================
	// Path Creation

	static RobotPath* CreateStraightPath(double max_velocity, double max_acceleration);
	static RobotPath* CreateSlalomPath(double max_velocity, double max_acceleration);
	static RobotPath* CreateSlalomPathRightAngles(double max_velocity, double max_acceleration);
	static RobotPath* CreateBarrelRacingPathRightAngles(double max_velocity, double max_acceleration);
	static RobotPath* CreateBouncePath(double max_velocity, double max_acceleration);
	static RobotPath* CreateTestPath(double max_velocity, double max_acceleration);
	static RobotPath* CreateGalaticSearchPath(double max_velocity, double max_acceleration);

   	//==========================================================================
	// Path Building

	static void AddStraight(PathSegment* path_segment, double length);
	static void AddTurnLeft(PathSegment* path_segment, double radius);
	static void AddTurnRight(PathSegment* path_segment, double radius);
	static void AddSlalomLeft(PathSegment* path_segment, double length, double width, double fraction);
	static void AddSlalomRight(PathSegment* path_segment, double length, double width, double fraction);

	// Get the current position and direction from the last Bezier curve in a given path segment
	//
	// path_segment - Path segment to get position and direction from. Must have at least one Bezier curve.
	// position - Returns the position of the current end of the path segment
	// direction - Returns the direction of the current end of the path segment
	static void GetCurrent(PathSegment* path_segment, Point2D& position, Point2D& direction);

	static void AddStraight(PathSegment* path_segment, double length, const Point2D& start, const Point2D& direction);
	static void AddTurnLeft(PathSegment* path_segment, double radius, const Point2D& start, const Point2D& direction);
	static void AddTurnRight(PathSegment* path_segment, double radius, const Point2D& start, const Point2D& direction);
	static void AddSlalomLeft(PathSegment* path_segment, double length, double width, double fraction, const Point2D& start, const Point2D& direction);
	static void AddSlalomRight(PathSegment* path_segment, double length, double width, double fraction, const Point2D& start, const Point2D& direction);


   	//==========================================================================
	// Utility Helpers

	// Get a direction vector rotated 90 degrees to the left
	static Point2D TurnLeft(const Point2D& direction);

	// Get a direction vector rotated 90 degrees to the right
	static Point2D TurnRight(const Point2D& direction);
};
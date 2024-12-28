//==============================================================================
// PathBuilder.h
//==============================================================================

#pragma once

#include "RobotPath.h"

#include <units/angle.h>


namespace PathBuilder
{

    //==========================================================================
    // Create Current Position

    void AddStraight(RobotPath& robot_path, units::meter_t length);
    void AddTurnLeft(RobotPath& robot_path, units::meter_t radius);
    void AddTurnRight(RobotPath& robot_path, units::meter_t radius);
    void AddSlalomLeft(RobotPath& robot_path, units::meter_t length, units::meter_t width, double fraction);
    void AddSlalomRight(RobotPath& robot_path, units::meter_t length, units::meter_t width, double fraction);
    void AddSegment(RobotPath& robot_path, const Vector2D<units::meter_t>& end, units::degree_t end_heading);

    //==========================================================================
    // Create From Arbitrary Position

    void AddStraight(RobotPath& robot_path, units::meter_t length, const Vector2D<units::meter_t>& start, const Vector2D<units::meter_t>& direction);
    void AddTurnLeft(RobotPath& robot_path, units::meter_t radius, const Vector2D<units::meter_t>& start, const Vector2D<units::meter_t>& direction);
    void AddTurnRight(RobotPath& robot_path, units::meter_t radius, const Vector2D<units::meter_t>& start, const Vector2D<units::meter_t>& direction);
    void AddSlalomLeft(RobotPath& robot_path, units::meter_t length, units::meter_t width, double fraction, const Vector2D<units::meter_t>& start, const Vector2D<units::meter_t>& direction);
    void AddSlalomRight(RobotPath& robot_path, units::meter_t length, units::meter_t width, double fraction, const Vector2D<units::meter_t>& start, const Vector2D<units::meter_t>& direction);
    void AddSegment(RobotPath& robot_path, const Vector2D<units::meter_t>& end, units::degree_t end_heading, const Vector2D<units::meter_t>& start, const Vector2D<units::meter_t>& direction);


    //==========================================================================
    // Helper Functions

    // Get the current position and direction from the last Bezier curve in a given path segment
    //
    // path_segment - Path segment to get position and direction from. Must have at least one Bezier curve.
    // position - Returns the position of the current end of the path segment
    // direction - Returns the direction of the current end of the path segment
    void GetCurrent(RobotPath& robot_path, Vector2D<units::meter_t>& position, Vector2D<units::meter_t>& direction);

    // Get a direction vector rotated 90 degrees to the left
    Vector2D<units::meter_t> TurnLeft(const Vector2D<units::meter_t>& direction);

    // Get a direction vector rotated 90 degrees to the right
    Vector2D<units::meter_t> TurnRight(const Vector2D<units::meter_t>& direction);

    // Constant for forming Bezier quarter arcs that are as close to circles as possible.
    // See https://spencermortensen.com/articles/bezier-circle/
    const double kBezierCircleC = 0.551915024494;

} // namespace PathBuilder

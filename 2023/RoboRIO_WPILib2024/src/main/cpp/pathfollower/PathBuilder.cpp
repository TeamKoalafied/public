//==============================================================================
// PathBuilder.cpp
//==============================================================================

#include "PathBuilder.h"


//==============================================================================
// Create Current Position

void PathBuilder::AddStraight(RobotPath& robot_path, units::meter_t length) {
    Vector2D<units::meter_t> start;
    Vector2D<units::meter_t> direction;
    GetCurrent(robot_path, start, direction);
    AddStraight(robot_path, length, start, direction);
}

void PathBuilder::AddTurnLeft(RobotPath& robot_path, units::meter_t radius) {
    Vector2D<units::meter_t> start;
    Vector2D<units::meter_t> direction;
    GetCurrent(robot_path, start, direction);
    AddTurnLeft(robot_path, radius, start, direction);
}

void PathBuilder::AddTurnRight(RobotPath& robot_path, units::meter_t radius) {
    Vector2D<units::meter_t> start;
    Vector2D<units::meter_t> direction;
    GetCurrent(robot_path, start, direction);
    AddTurnRight(robot_path, radius, start, direction);
}

void PathBuilder::AddSlalomLeft(RobotPath& robot_path, units::meter_t length, units::meter_t width, double fraction) {
    Vector2D<units::meter_t> start;
    Vector2D<units::meter_t> direction;
    GetCurrent(robot_path, start, direction);
    AddSlalomLeft(robot_path, length, width, fraction, start, direction);
}

void PathBuilder::AddSlalomRight(RobotPath& robot_path, units::meter_t length, units::meter_t width, double fraction) {
    Vector2D<units::meter_t> start;
    Vector2D<units::meter_t> direction;
    GetCurrent(robot_path, start, direction);
    AddSlalomRight(robot_path, length, width, fraction, start, direction);
}

void PathBuilder::AddSegment(RobotPath& robot_path, const Vector2D<units::meter_t>& end, units::degree_t end_heading) {
    Vector2D<units::meter_t> start;
    Vector2D<units::meter_t> direction;
    GetCurrent(robot_path, start, direction);
    AddSegment(robot_path, end, end_heading, start, direction);
}


//==============================================================================
// Create From Arbitrary Position

void PathBuilder::AddStraight(RobotPath& robot_path, units::meter_t length, const Vector2D<units::meter_t>& start, const Vector2D<units::meter_t>& direction) {
    Vector2D<units::meter_t> unit_direction = direction;
    unit_direction.Normalize();

    Bezier3_2D<units::meter_t> path;
    path.m_point1 = start;
    path.m_point2 = start + unit_direction * (0.25 * length).value();
    path.m_point3 = start + unit_direction * (0.75 * length).value();
    path.m_point4 = start + unit_direction * length.value();

    robot_path.m_bezier_list.push_back(path);

}

void PathBuilder::AddTurnLeft(RobotPath& robot_path, units::meter_t radius, const Vector2D<units::meter_t>& start, const Vector2D<units::meter_t>& direction) {
    Vector2D<units::meter_t> unit_direction = direction;
    unit_direction.Normalize();

    Bezier3_2D<units::meter_t> path;
    path.m_point1 = start;
    path.m_point2 = start + unit_direction * radius.value() * kBezierCircleC;
    path.m_point3 = start + unit_direction * radius.value() + TurnLeft(unit_direction) * radius.value() * (1.0 - kBezierCircleC);
    path.m_point4 = start + unit_direction * radius.value() + TurnLeft(unit_direction) * radius.value();

    robot_path.m_bezier_list.push_back(path);
}

void PathBuilder::AddTurnRight(RobotPath& robot_path, units::meter_t radius, const Vector2D<units::meter_t>& start, const Vector2D<units::meter_t>& direction) {
    Vector2D<units::meter_t> unit_direction = direction;
    unit_direction.Normalize();

    Bezier3_2D<units::meter_t> path;
    path.m_point1 = start;
    path.m_point2 = start + unit_direction * radius.value() * kBezierCircleC;
    path.m_point3 = start + unit_direction * radius.value() + TurnRight(unit_direction) * radius.value() * (1.0 - kBezierCircleC);
    path.m_point4 = start + unit_direction * radius.value() + TurnRight(unit_direction) * radius.value();

    robot_path.m_bezier_list.push_back(path);
}

void PathBuilder::AddSlalomLeft(RobotPath& robot_path, units::meter_t length, units::meter_t width, double fraction, const Vector2D<units::meter_t>& start, const Vector2D<units::meter_t>& direction) {
    Vector2D<units::meter_t> unit_direction = direction;
    unit_direction.Normalize();

    Vector2D<units::meter_t> fraction_vector = fraction * unit_direction;
    Vector2D<units::meter_t> end = start + unit_direction * length.value() + TurnLeft(unit_direction) * width.value();

    Bezier3_2D<units::meter_t> path;
    path.m_point1 = start;
    path.m_point2 = start + fraction_vector;
    path.m_point3 = end - fraction_vector;
    path.m_point4 = end;

    robot_path.m_bezier_list.push_back(path);

}

void PathBuilder::AddSlalomRight(RobotPath& robot_path, units::meter_t length, units::meter_t width, double fraction, const Vector2D<units::meter_t>& start, const Vector2D<units::meter_t>& direction) {
    Vector2D<units::meter_t> unit_direction = direction;
    unit_direction.Normalize();

    Vector2D<units::meter_t> fraction_vector = fraction * unit_direction;
    Vector2D<units::meter_t> end = start + unit_direction * length.value() + TurnRight(unit_direction) * width.value();

    Bezier3_2D<units::meter_t> path;
    path.m_point1 = start;
    path.m_point2 = start + fraction_vector;
    path.m_point3 = end - fraction_vector;
    path.m_point4 = end;

    robot_path.m_bezier_list.push_back(path);

}

void PathBuilder::AddSegment(RobotPath& robot_path, const Vector2D<units::meter_t>& end, units::degree_t end_heading, const Vector2D<units::meter_t>& start, const Vector2D<units::meter_t>& direction) {
    Vector2D<units::meter_t> unit_direction = direction;
    unit_direction.Normalize();

    units::meter_t distance = (end - start).Length();
    double fraction = kBezierCircleC;


    Vector2D<units::meter_t> end_direction = Vector2D<units::meter_t>::UnitVector(end_heading);
    // if (path_segment->m_reverse) {
    //     end_direction = -end_direction;
    // }

    Bezier3_2D<units::meter_t> path;
    path.m_point1 = start;
    path.m_point2 = start + unit_direction * (distance * fraction).value();
    path.m_point3 = end - end_direction * (distance * fraction).value();
    path.m_point4 = end;

    // std::cout << "AddSegment\n";
    // std::cout << "Pt1 (" << path.m_point1.x << ", " << path.m_point1.y << ")\n";
    // std::cout << "Pt2 (" << path.m_point2.x << ", " << path.m_point2.y << ")\n";
    // std::cout << "Pt3 (" << path.m_point3.x << ", " << path.m_point3.y << ")\n";
    // std::cout << "Pt4 (" << path.m_point4.x << ", " << path.m_point4.y << ")\n";

    robot_path.m_bezier_list.push_back(path);
}

//==============================================================================
// Helper Functions

void PathBuilder::GetCurrent(RobotPath& robot_path, Vector2D<units::meter_t>& position, Vector2D<units::meter_t>& direction) {
    const Bezier3_2D<units::meter_t>& path = robot_path.m_bezier_list.back();
    position = path.m_point4;
    direction = path.m_point4 - path.m_point3; 
}

Vector2D<units::meter_t> PathBuilder::TurnLeft(const Vector2D<units::meter_t>& direction) {
    return Vector2D<units::meter_t>(-direction.y, direction.x);
}

Vector2D<units::meter_t> PathBuilder::TurnRight(const Vector2D<units::meter_t>& direction) {
    return Vector2D<units::meter_t>(direction.y, -direction.x);
}

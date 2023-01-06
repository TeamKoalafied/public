//==============================================================================
// RobotPathPoints.cpp
//==============================================================================

#include "RobotPathPoints.h"

#include <frc/Timer.h>

#define _USE_MATH_DEFINES
#include <math.h>
#include <iostream>

//=============================================================================
// Construction and Destruction

RobotPathPoints::RobotPathPoints()
{
}


RobotPathPoints::~RobotPathPoints()
{
}


//=============================================================================
// Path Generation

void RobotPathPoints::GeneratePathPoints(const std::vector<Bezier3>& path_definition,
								   		 const MotionProfile& motion_profile, double wheel_base,
										 bool reverse, double time_period_s)
{
	double wheelbase_half_width = wheel_base / 2.0;

	std::vector<double> segment_lengths(path_definition.size());
	for (int i = 0; i < int(path_definition.size()); i++) segment_lengths[i] = path_definition[i].Length();

	double total_time = motion_profile.GetTotalTime();
	int total_time_steps = (int)ceil(total_time / time_period_s);

	m_left_path_points.resize(total_time_steps + 1);
	m_right_path_points.resize(total_time_steps + 1);

	for (int i = 0; i <= total_time_steps; i++) {
		double time = i * time_period_s;
		double position;
		double velocity;
		double acceleration;
		motion_profile.GetMotion(time, position, velocity, acceleration);

		//std::cout << "time: " << time << " position: " << position << " velocity: " << velocity << " acceleration: " << acceleration << "\n";


		int segment_index = 0;
		double segment_position = position;
		while (segment_position > segment_lengths[segment_index]) {
			segment_position -= segment_lengths[segment_index];
			segment_index++;
		}

		double u;
		Point2D path_point = path_definition[segment_index].ArcLengthPoint(segment_position, u);
		Point2D direction = path_definition[segment_index].Derivative(u);

		double angle_rad = atan2(direction.y, direction.x);
		double heading_degrees = angle_rad * 180.0 / M_PI;
		while (heading_degrees >= 360.0) heading_degrees -= 360.0;
		while (heading_degrees <    0.0) heading_degrees += 360.0;

		
		//                Debug.WriteLine(string.Format("Time {0}  Position {1} SegmentIndex {2} SegmentPosition {3} Point {4} u {5}",
		//                    time, position, segment_index, segment_position, path_point, u));

		// Calculate an othogonal direction to the left of the path
		Point2D orthogonal_direction_left(-direction.y, direction.x);
		orthogonal_direction_left.Normalize();
		orthogonal_direction_left *= wheelbase_half_width;
//		Debug::WriteLine("path_point: (" + path_point.x + ", " + path_point.y + ") orthogonal_direction_left: (" + orthogonal_direction_left.x + ", " + orthogonal_direction_left.y + ") u: " + u);
		//                PathPoint left_point = m_left_path_points[i];
		//                PathPoint right_point = m_right_path_points[i];
		PathPoint& left_point = m_left_path_points[i];
		PathPoint& right_point = m_right_path_points[i];
		left_point.m_time = time;
		left_point.m_position = path_point + orthogonal_direction_left;
		left_point.m_heading_deg = heading_degrees;
		right_point.m_time = time;
		right_point.m_position = path_point - orthogonal_direction_left;
		right_point.m_heading_deg = heading_degrees;
	}

	CalculatePathParameters(m_left_path_points);
	CalculatePathParameters(m_right_path_points);

	if (reverse) {
		m_left_path_points.swap(m_right_path_points);

		for (int i = 0; i < (int)m_left_path_points.size(); i++) {
			m_left_path_points[i].m_distance = -m_left_path_points[i].m_distance;
			m_right_path_points[i].m_distance = -m_right_path_points[i].m_distance;
			m_left_path_points[i].m_velocity = -m_left_path_points[i].m_velocity;
			m_right_path_points[i].m_velocity = -m_right_path_points[i].m_velocity;
			m_left_path_points[i].m_acceleration = -m_left_path_points[i].m_acceleration;
			m_right_path_points[i].m_acceleration = -m_right_path_points[i].m_acceleration;
			m_left_path_points[i].m_jerk = -m_left_path_points[i].m_jerk;
			m_right_path_points[i].m_jerk = -m_right_path_points[i].m_jerk;

			double heading = m_left_path_points[i].m_heading_deg + 180;
			if (heading > 360.0) heading -= 360.0;
			m_left_path_points[i].m_heading_deg = heading;
			m_right_path_points[i].m_heading_deg = heading;
		}	
	}
}

void RobotPathPoints::CalculatePathParameters(std::vector<PathPoint>& path_points)
{
	if (path_points.size() == 0) return;

	PathPoint& first_point = path_points[0];
	first_point.m_distance = 0.0;
	first_point.m_velocity = 0.0;
	first_point.m_acceleration = 0.0;
	first_point.m_jerk = 0.0;
	for (int i = 1; i < (int)path_points.size(); i++) {
		PathPoint& path_point = path_points[i];
		PathPoint& last_point = path_points[i - 1];
		double distance = (path_point.m_position - last_point.m_position).Length();
		double delta_t = path_point.m_time - last_point.m_time;
		path_point.m_distance = last_point.m_distance + distance;
		path_point.m_velocity = distance / delta_t;
		path_point.m_acceleration = (path_point.m_velocity - last_point.m_velocity) / delta_t;
		path_point.m_jerk = (path_point.m_acceleration - last_point.m_acceleration) / delta_t;
	}
}


//==============================================================================
// Statis Test Function

void RobotPathPoints::TestPathGeneration()
{
	std::cout << "RobotPathPoints::TestPathGenerationa()\n";

    frc::Timer timer;
	timer.Start();
	
	Bezier3 path;
	// Straight line 2m
	// path.m_point1.x = 0.0;
	// path.m_point1.y = 0.0;
	// path.m_point2.x = 0.0;
	// path.m_point2.y = 0.5;
	// path.m_point3.x = 0.0;
	// path.m_point3.y = 1.5;
	// path.m_point4.x = 0.0;
	// path.m_point4.y = 2.0;
	path.m_point1.x = 0.0;
	path.m_point1.y = 0.0;
	path.m_point2.x = 1.0;
	path.m_point2.y = 0.0;
	path.m_point3.x = 2.0;
	path.m_point3.y = 1.0;
	path.m_point4.x = 2.0;
	path.m_point4.y = 2.0;
	MotionProfile motion_profile;
	motion_profile.Setup(1.0, 0.5);
	motion_profile.InitialiseProfile(path.Length());

	std::vector<Bezier3> path_definition;
	path_definition.push_back(path);

	double wheel_base = 0.72;
	RobotPathPoints robot_path;
	robot_path.GeneratePathPoints(path_definition, motion_profile, wheel_base, false, 0.02);

	double time = timer.Get() * 1000.0;
	std::cout << "LenRobotPath::TestPathGenerationa() took " << time << "ms\n";
	std::cout << "Length: " << path.Length() << "\n";
	std::cout << "Time: " << motion_profile.GetTotalTime() << "\n";

	// std::cout << "Left\n";
	// int total_points = robot_path.GetLeftPathPoints().size();
	// for (int i = 0; i < total_points; i++) {
	// 	RobotPathPoints::PathPoint& point = robot_path.GetLeftPathPoints()[i];

	// 	std::cout << point.m_time << "," << point.m_position.x << "," << point.m_position.y << "," <<
	// 		point.m_distance << "," << point.m_heading_deg << "," << point.m_velocity << "," << point.m_acceleration << "," << point.m_jerk << "\n";
	// }
	// std::cout << "Right\n";
	// total_points = robot_path.GetRightPathPoints().size();
	// for (int i = 0; i < total_points; i++) {
	// 	RobotPathPoints::PathPoint& point = robot_path.GetRightPathPoints()[i];

	// 	std::cout << point.m_time << "," << point.m_position.x << "," << point.m_position.y << "," <<
	// 		point.m_distance << "," << point.m_heading_deg << "," << point.m_velocity << "," << point.m_acceleration << "," << point.m_jerk << "\n";
	// }

}

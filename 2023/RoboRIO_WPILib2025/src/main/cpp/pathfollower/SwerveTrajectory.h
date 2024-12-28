//==============================================================================
// SwerveTrajectory.h
//==============================================================================
#pragma once

//#include "Point2D.h"
#include "Vector2D.h"

#include <frc/geometry/Pose2d.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>

#include <vector>

namespace Logging { class CsvFile; }


class SwerveTrajectory
{
public:
	//==========================================================================
	// Pulbic Nested Types


    // Point on the linear approximate version of the path
    struct Point
    {
        units::second_t m_time;

        Vector2D<units::meter_t> m_position;
        Vector2D<units::meters_per_second_t> m_velocity_vector;
        Vector2D<units::meters_per_second_squared_t> m_acceleration_vector;

        units::degree_t m_rotation_degrees;
        units::degrees_per_second_t m_rotational_velocity_degrees;
        units::degrees_per_second_squared_t m_rotational_acceleration_degrees;

        units::degree_t m_heading_degrees;
        units::meter_t m_distance;
        units::meters_per_second_t m_velocity;
        units::meters_per_second_squared_t m_acceleration;
        double m_curvature;

        units::meters_per_second_t m_max_velocity;
    };

	//==========================================================================
	// Construction

	// Default constructor
	SwerveTrajectory() {}


	//==========================================================================
	// Operations

    void Reset(int size) {
        m_path_points.clear();
        m_path_points.resize(size);
    }

    const Point& GetPoint(int index) const {
        return m_path_points[index];
    }
    Point& GetPoint(int index) {
        return m_path_points[index];
    }

    int GetPointCount() const {
        return m_path_points.size();
    }

    units::second_t GetTime() const { return m_path_time; }
    void SetTime(units::second_t time) { m_path_time = time; }

    void LogToFile(const char* filename, std::ios_base::openmode openmode = std::ios::out | std::ios::trunc);

    void LogToFile(Logging::CsvFile& csv_log_file);

    frc::Pose2d GetInitialPose() const {
        return GetPoseFromPoint(m_path_points[0]);
    }

    static frc::Pose2d GetPoseFromPoint(const Point& point);

private:
    std::vector<Point> m_path_points;       // Point data that describes the current segment
    units::second_t m_path_time;
};


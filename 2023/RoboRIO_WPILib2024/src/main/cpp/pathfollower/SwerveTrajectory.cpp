//==============================================================================
// SwerveTrajectory.cpp
//==============================================================================

#include "SwerveTrajectory.h"

#include "../util/Logging.h"

#include <iostream>


//==========================================================================
// Operations

void SwerveTrajectory::LogToFile(const char* filename, std::ios_base::openmode openmode) {
    Logging::CsvFile csv_log_file;    // CSV log file being written to
    csv_log_file.Open(filename, openmode);
    if (csv_log_file.Fail()) {
        std::cout << "SwerveTrajectory::LogToFile() - Failed to open log file '" << filename << "'\n";
        return;
    }

    LogToFile(csv_log_file);
}

void SwerveTrajectory::LogToFile(Logging::CsvFile& csv_log_file) {

    csv_log_file << "Time";
    csv_log_file << "PositionX" << "PositionY";
    csv_log_file << "VelocityX" << "VelocityY";
    csv_log_file << "AccelerationX" << "AccelerationY";
    csv_log_file << "Rotation" << "AngularVelocity" << "AngularAcceleration";
    csv_log_file << "Heading" << "Distance" << "Velocity";
    csv_log_file << "Acceleration" << "Curvature"  << "MaxVelocity" << "\n";
    csv_log_file.SetPrecision(3, true);

    for (const Point& point : m_path_points) {
        csv_log_file << point.m_time.value();
        csv_log_file << point.m_position.x.value() << point.m_position.y.value();
        csv_log_file << point.m_velocity_vector.x.value() << point.m_velocity_vector.y.value();
        csv_log_file << point.m_acceleration_vector.x.value() << point.m_acceleration_vector.y.value();
        csv_log_file << point.m_rotation_degrees.value() << point.m_rotational_velocity_degrees.value() <<
                        point.m_rotational_acceleration_degrees.value();
        csv_log_file << point.m_heading_degrees.value() << point.m_distance.value() << point.m_velocity.value();
        csv_log_file << point.m_acceleration.value() << point.m_curvature << point.m_max_velocity.value() << "\n";
    }
}

frc::Pose2d SwerveTrajectory::GetPoseFromPoint(const Point& point) {
    return frc::Pose2d(point.m_position.x, point.m_position.y, point.m_rotation_degrees);
}

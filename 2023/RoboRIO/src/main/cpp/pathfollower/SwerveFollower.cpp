//==============================================================================
// SwerveFollower.cpp
//==============================================================================

#include "SwerveFollower.h"

#include "SwerveTrajectory.h"

#include "../util/Logging.h"

#include <wpi/MathExtras.h>
#include <units/math.h>

#include <iostream>



SwerveFollower::SwerveFollower(const SwerveTrajectory& trajectory, frc::SwerveDriveKinematics<4>& kinematics,
                               const char* log_filename) :
  m_log_filename(log_filename),
  m_trajectory(trajectory),
  m_kinematics(kinematics) {



    // Setup sensible default values for the parameters
   	// Velocity = 1.17ft/s/V => 14.04ft/s for 12V => 4.28m/s for 12V
 	// Acceleration = ~5ft/s2/V => 60ft/s2 for 12V => 18.29m/s2 for 12V
    // m_follower_parameters.m_kp = 1.0;
    // m_follower_parameters.m_ki = 0.0;
    // m_follower_parameters.m_kd = 0.0;
    // m_follower_parameters.m_kv = 1.0/4.28;
    // m_follower_parameters.m_kv_offset = 0.104;
    // m_follower_parameters.m_ka =  1.0/18.29;
    // m_follower_parameters.m_period_s = 0.02; // 20ms
    // m_follower_parameters.m_wheelbase_width_m = 0.71;
    // m_follower_parameters.m_path_point_spacing = 0.2;  // Spacing between generated path points
    // m_follower_parameters.m_max_velocity = 0.5;
    // m_follower_parameters.m_max_acceleration = 0.25;
    // m_follower_parameters.m_max_velocity_curve = 2.0;
    m_follower_parameters.m_lookahead_distance = 0.5_m;
    m_follower_parameters.m_lookahead_factor = 1.5;
    m_follower_parameters.m_correction_blend = 0.5;
    // m_follower_parameters.m_lookahead_curvature_gain = 1.0;
    // m_follower_parameters.m_path_curvature_gain = 1.0;
    // m_follower_parameters.m_lookalong_time = 0.0;

}


void SwerveFollower::Initialize() {

    m_closest_index = 0;
    m_lookahead_index = 0;
    m_segment_finished = false;

    StartLogging();
}

std::array<SwerveFollowerModuleState, 4> SwerveFollower::Update(const frc::Pose2d& pose) {

    SwerveTrajectory::Point closest_point = SwerveFollower::GetClosestPoint(pose);

    // Get the lookahead point. This is a point a short way ahead on the path that we will aim
    // the robot towards.
    Vector2D<units::meter_t> robot_point(pose.Translation().X(), pose.Translation().Y());
    units::meter_t closest_point_distance = (closest_point.m_position - robot_point).Length();
    SwerveTrajectory::Point lookahead_point = GetLookAheadPoint(pose, closest_point_distance);


    Vector2D<units::meter_t> correction_vector = lookahead_point.m_position - robot_point;
    correction_vector.Normalize();
    Vector2D<units::meters_per_second_t> velocity_vector = closest_point.m_velocity_vector;
    units::meters_per_second_t speed = velocity_vector.Normalize();

    units::meters_per_second_t min_speed = closest_point.m_acceleration * 0.02_s;
    if (speed < min_speed) speed = min_speed;

    double correction_blend = m_follower_parameters.m_correction_blend;
//    velocity_vector.x = velocity_vector.x * (1.0 - correction_blend) + correction_vector.x * correction_blend / 1_s;
//    velocity_vector.x = velocity_vector.y * (1.0 - correction_blend) + correction_vector.y * correction_blend / 1_s;

    velocity_vector.x = units::meters_per_second_t(velocity_vector.x.value() * (1.0 - correction_blend) +
                                                   correction_vector.x.value() * correction_blend);
    velocity_vector.y = units::meters_per_second_t(velocity_vector.y.value() * (1.0 - correction_blend) +
                                                   correction_vector.y.value() * correction_blend);

    velocity_vector = velocity_vector * speed.value();

    units::degrees_per_second_t rotational_velocity = closest_point.m_rotational_velocity_degrees;
    rotational_velocity += (closest_point.m_rotation_degrees - pose.Rotation().Degrees())/5_s;


    frc::ChassisSpeeds chassis_speed = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
      velocity_vector.x, velocity_vector.y, rotational_velocity, pose.Rotation());


    // frc::ChassisSpeeds chassis_speed;
    // // chassis_speed.vx = closest_point.m_velocity_vector.x;
    // // chassis_speed.vy = closest_point.m_velocity_vector.y;
    // chassis_speed.vx = velocity_vector.x;
    // chassis_speed.vy = velocity_vector.y;
    // chassis_speed.omega = rotational_velocity;
    // //units::meter_t radius = 1.0/  closest_point.curvature;

    chassis_speed.omega = closest_point.m_rotational_velocity_degrees;

    auto targetModuleStates = m_kinematics.ToSwerveModuleStates(chassis_speed);

    std::array<SwerveFollowerModuleState, 4> result;

    for (int i = 0; i < 4; i++) {
        result[i].angle = targetModuleStates[i].angle;
        result[i].speed = targetModuleStates[i].speed;
        result[i].acceleration = closest_point.m_acceleration;
    }

    UpdateLogging(pose, result);
    // std::cout << "Angle " << result[0].angle.Degrees().value() <<
    //              " Speed " << result[0].speed.value() <<
    //              " Acceleration " << result[0].acceleration.value() << "\n";

    return result;
}
/*
void SwerveFollower::Execute() {

    frc::Pose2d robot_position = m_get_pose_function();

    frc::Trajectory::State closest_point = SwerveFollower::GetClosestPoint(robot_position);

    // Get the lookahead point. This is a point a short way ahead on the path that we will aim
    // the robot towards.
    units::meter_t closest_point_distance = (closest_point.pose.Translation() - robot_position.Translation()).Norm();
//    frc::Trajectory::State lookahead_point = GetLookAheadPoint(robot_position, closest_point_distance);

    frc::ChassisSpeeds chassis_speed;
    chassis_speed.vx = closest_point.velocity * closest_point.pose.Rotation().Cos();
    chassis_speed.vy = closest_point.velocity * closest_point.pose.Rotation().Sin();
    //units::meter_t radius = 1.0/  closest_point.curvature;

    chassis_speed.omega = (closest_point.pose.Rotation() - robot_position.Rotation()).Radians() / 1_s;

    auto targetModuleStates = m_kinematics.ToSwerveModuleStates(chassis_speed);

    m_set_module_states_function(targetModuleStates);


  // Calculate feedforward velocities (field-relative)
//   auto xFF = desiredLinearVelocity * trajectoryPose.Rotation().Cos();
//   auto yFF = desiredLinearVelocity * trajectoryPose.Rotation().Sin();
//   auto thetaFF = units::radians_per_second_t{m_thetaController.Calculate(
//       currentPose.Rotation().Radians(), desiredHeading.Radians())};
}

void SwerveFollower::End(bool interrupted) {
    m_timer.Stop();
}
*/
bool SwerveFollower::IsFinished() {

    return m_segment_finished;
    //return m_timer.HasElapsed(m_trajectory.TotalTime());
}

void SwerveFollower::End() {
    EndLogging();
}


//==============================================================================
// Pure Pursuit Calculations

SwerveTrajectory::Point SwerveFollower::GetClosestPoint(frc::Pose2d robot_position) {

    // Zero the velocity and acceleration in case we don't find any values
    // path_velocity = 0;
    // path_acceleration = 0;
    // path_curvature = 0;

    // Search for the closest point on the path starting at the previous closest point and
    // only searching forwards. This prevents accidentally turning around.
    int closest_index = -1;
    units::meter_t closest_distance;
    Vector2D<units::meter_t> closest_point;
    int path_index = m_closest_index;
    Vector2D<units::meter_t> robot_point(robot_position.Translation().X(), robot_position.Translation().Y());
    //double closest_relative = 0.0;
    SwerveTrajectory::Point closest_state;
    while (path_index < m_trajectory.GetPointCount() - 1) {
        // Get the ends of the path segment to test
        const SwerveTrajectory::Point& segment_pt1 = m_trajectory.GetPoint(path_index);
        const SwerveTrajectory::Point& segment_pt2 = m_trajectory.GetPoint(path_index + 1);

        // Get the distance and relative position of the closest point on the path segment
        double relative;
        Vector2D<units::meter_t> point_on_line = ProjectPointOntoLine(robot_point, segment_pt1.m_position,
                                                                segment_pt2.m_position, relative);
        SwerveTrajectory::Point on_line_state;
        if (0.0 <= relative && relative <= 1.0) {
            // Closest point is within the segment, so 'relative' and 'point_on_line' are OK
            on_line_state = Lerp(segment_pt1, segment_pt2, relative);
        }
        else if (relative < 0.0) {
            // Closest point is the beginning of the segment
            relative = 0.0;
            point_on_line = segment_pt1.m_position;
            on_line_state = segment_pt1;
        } else {
            // Closest point is the end of the segment
            relative = 1.0;
            point_on_line = segment_pt2.m_position;
            on_line_state = segment_pt2;
        }
        units::meter_t distance = (robot_point - point_on_line).Length();

        // If this is the first test, or is closer than any other record the point as the current closest
        if (closest_index == -1 || distance < closest_distance) {
            closest_index = path_index;
            closest_distance = distance;
            closest_point = point_on_line;
            closest_state = on_line_state;
            //closest_relative = relative;

            // Get the velocity and acceleration by linearly interpolating between the ends of the segment
            // path_velocity = Lerp(segment_pt1.m_velocity, segment_pt2.m_velocity, relative);
            // path_acceleration = Lerp(segment_pt1.m_acceleration, segment_pt2.m_acceleration, relative);
            // path_curvature = Lerp(segment_pt1.m_curvature, segment_pt2.m_curvature, relative);
        } else if (distance > 10*closest_distance || distance > 2.0_m) {
            // If this segment is a long way further than the best then stop searching. This saves time
            // and also prevent accidentally picking up a later part of the path if it loops back on
            // itself (a figure-8 for example).
            break;
        } else if (path_index - m_closest_index > 5) {
            // Stop searching if we have advanced by more that 5 segments. This has been founc
            // to be effective in preventing the robot skipping to a later part of the path.
            // For 0.1 spacing and 20ms update 5 segments is 0.5/0.02 = 25m/s
            break;
        }
        path_index++;
    }

    // If no point was found just return the robot position (as this is safe) and record the segment as finished.
    // This should not be possible.
    if (closest_index == -1) {
//        m_segment_finished = true;
//        return robot_position;
    }

    // Update the closest point index for next time and return the closest point
    m_closest_index = closest_index;
 

    // if (m_follower_parameters.m_lookalong_time == 0.0) {
    //     // Get the velocity and acceleration by linearly interpolating between the ends of the segment
    //     const PathPoint& segment_pt1 = m_path_points[closest_index];
    //     const PathPoint& segment_pt2 = m_path_points[closest_index + 1];
    //     path_velocity = Lerp(segment_pt1.m_velocity, segment_pt2.m_velocity, closest_relative);
    //     path_acceleration = Lerp(segment_pt1.m_acceleration, segment_pt2.m_acceleration, closest_relative);
    //     path_curvature = Lerp(segment_pt1.m_curvature, segment_pt2.m_curvature, closest_relative);
    // }
    // else {
    //     double lookalong_time = m_follower_parameters.m_lookalong_time;
    //     while (true) {
    //         const PathPoint& segment_pt1 = m_path_points[closest_index];
    //         const PathPoint& segment_pt2 = m_path_points[closest_index + 1];

    //         double velocity = Lerp(segment_pt1.m_velocity, segment_pt2.m_velocity, closest_relative);
    //         double average_velocity = (velocity + segment_pt2.m_velocity)/2.0;
    //         double segment_length = segment_pt2.m_distance - segment_pt1.m_distance;
    //         double remaining_distance_in_segment = segment_length * (1.0 - closest_relative);
    //         double remaining_time_in_segment = remaining_distance_in_segment / average_velocity;
    //         if (lookalong_time < remaining_time_in_segment) {
    //             closest_relative = Lerp(closest_relative, 1.0, lookalong_time/remaining_time_in_segment);
    //             path_velocity = Lerp(segment_pt1.m_velocity, segment_pt2.m_velocity, closest_relative);
    //             path_acceleration = Lerp(segment_pt1.m_acceleration, segment_pt2.m_acceleration, closest_relative);
    //             path_curvature = Lerp(segment_pt1.m_curvature, segment_pt2.m_curvature, closest_relative);
    //             break;
    //         }
    //         else {
    //             lookalong_time -= remaining_time_in_segment;
    //             closest_relative = 0.0;
    //             if (closest_index < (int)m_path_points.size() - 2) {
    //                 closest_index++;
    //             }
    //             else {
    //                 path_velocity = segment_pt2.m_velocity;
    //                 path_acceleration = segment_pt2.m_acceleration;
    //                 path_curvature = segment_pt2.m_curvature;
    //                 break;
    //             }
    //         }
    //     }
    // }

    // If we are closest to a point on the 'extension' segment then we are passed the 
    // end of the path and so we are done
    if (closest_index == m_trajectory.GetPointCount() - 2) {
        m_segment_finished = true;
    }

    // If we are on the last segment that is part of the actual path test for being
    // very close to the end because sometimes we stop a little bit short
    if (closest_index == m_trajectory.GetPointCount() - 3) {
         Vector2D<units::meter_t> end_point = m_trajectory.GetPoint(closest_index + 1).m_position;
        const units::meter_t STOP_TOLERANCE_M = 0.05_m; // 5cm tolerance for reaching the end point
        if ((closest_point - end_point).Length() < STOP_TOLERANCE_M) {
            m_segment_finished = true;
        }
    }

    // Return the closest point
    return closest_state;
}

SwerveTrajectory::Point SwerveFollower::GetLookAheadPoint(frc::Pose2d robot_position, units::meter_t closest_point_distance) {
    // The lookahead distance must be greater than the distance to the path or it will most likely
    // not intersect with the path. Make sure the lookahead distance is greater that the closest point
    // distance by some margin. This margin is an important parameter. If it is not large enough then
    // the search may not find a lookahead point point and following the path will fail.
    units::meter_t lookahead_distance = m_follower_parameters.m_lookahead_distance;
    units::meter_t lookahead_distance_closest = closest_point_distance * m_follower_parameters.m_lookahead_factor;
    if (lookahead_distance < lookahead_distance_closest) {
        lookahead_distance = lookahead_distance_closest;
    }

    // Search for the closest point on the path starting at the previous closest point and
    // only searching forwards. This prevents accidentally turning around.
    Vector2D<units::meter_t> robot_point(robot_position.Translation().X(), robot_position.Translation().Y());
    while (m_lookahead_index < m_trajectory.GetPointCount() - 1) {
        // Get the ends of the path segment to test
        const SwerveTrajectory::Point& segment_pt1 = m_trajectory.GetPoint(m_lookahead_index);
        const SwerveTrajectory::Point& segment_pt2 = m_trajectory.GetPoint(m_lookahead_index + 1);

        // Get the distance and relative position of the closest point on the path segment
        double relative;
        Vector2D<units::meter_t> point_on_line = ProjectPointOntoLine(robot_point, segment_pt1.m_position, segment_pt2.m_position, relative);
        units::meter_t distance = (robot_point - point_on_line).Length();

        units::meter_t segment_length = (segment_pt1.m_position - segment_pt2.m_position).Length();

        units::meter_t forward_distance = units::math::sqrt(lookahead_distance*lookahead_distance - distance*distance);

        double intersection_relative = relative + forward_distance/segment_length;

        if (0.0 <= intersection_relative && intersection_relative <= 1.0) {
            return Lerp(segment_pt1, segment_pt2, intersection_relative);
        }

        m_lookahead_index++;
    }

    m_segment_finished = true;
    return m_trajectory.GetPoint(m_trajectory.GetPointCount() - 1);

}

frc::Translation2d SwerveFollower::ProjectPointOntoLine(frc::Translation2d pt, frc::Translation2d line1,
                                                               frc::Translation2d line2, double& relative) {
                                                        
    // 
    //        P .     . B
    //            . X
    //      A .
    //
    //
    //                


    frc::Translation2d AP = pt - line1;
    frc::Translation2d AB = line2 - line1;

    relative = (AP.X() * AB.X() + AP.Y() * AB.Y())/(AB.X() * AB.X() + AB.Y() * AB.Y());

    return line1 + AB * relative;
}

Point2D SwerveFollower::ProjectPointOntoLine(Point2D pt, Point2D line1, Point2D line2, double& relative) {
                                                        
    // 
    //        P .     . B
    //            . X
    //      A .
    //
    //
    //                


    Point2D AP = pt - line1;
    Point2D AB = line2 - line1;

    relative = (AP.x * AB.x + AP.y * AB.y)/(AB.x * AB.x + AB.y * AB.y);

    return line1 + AB * relative;
}

Vector2D<units::meter_t> SwerveFollower::ProjectPointOntoLine(Vector2D<units::meter_t> pt, Vector2D<units::meter_t> line1, Vector2D<units::meter_t> line2, double& relative) {
                                                       
    // 
    //        P .     . B
    //            . X
    //      A .
    //
    //
    //                


    Vector2D<units::meter_t> AP = pt - line1;
    Vector2D<units::meter_t> AB = line2 - line1;

    relative = (AP.x * AB.x + AP.y * AB.y)/(AB.x * AB.x + AB.y * AB.y);

    return line1 + AB * relative;
}

//==============================================================================
// Trajectory Operations

SwerveTrajectory::Point SwerveFollower::Lerp(const SwerveTrajectory::Point& state1, const SwerveTrajectory::Point& state2, double t) {

    SwerveTrajectory::Point result;

    result.m_position = wpi::Lerp(state1.m_position, state2.m_position, t);
    result.m_velocity_vector = wpi::Lerp(state1.m_velocity_vector, state2.m_velocity_vector, t);
    result.m_acceleration_vector = wpi::Lerp(state1.m_acceleration_vector, state2.m_acceleration_vector, t);

    result.m_rotation_degrees = wpi::Lerp(state1.m_rotation_degrees, state2.m_rotation_degrees, t);
    result.m_rotational_velocity_degrees = wpi::Lerp(state1.m_rotational_velocity_degrees, state2.m_rotational_velocity_degrees, t);
    result.m_rotational_acceleration_degrees = wpi::Lerp(state1.m_rotational_acceleration_degrees, state2.m_rotational_acceleration_degrees, t);

    // TODO m_heading_degrees should not be interpolated like this because it wraps around
    result.m_heading_degrees = wpi::Lerp(state1.m_heading_degrees, state2.m_heading_degrees, t);
    result.m_distance = wpi::Lerp(state1.m_distance, state2.m_distance, t);
    result.m_velocity = wpi::Lerp(state1.m_velocity, state2.m_velocity, t);
    result.m_acceleration = wpi::Lerp(state1.m_acceleration, state2.m_acceleration, t);
    result.m_curvature = wpi::Lerp(state1.m_curvature, state2.m_curvature, t);

    result.m_max_velocity = wpi::Lerp(state1.m_max_velocity, state2.m_max_velocity, t);

        // Vector2D<units::meter_t> m_position;
        // Vector2D<units::meters_per_second_t> m_velocity_vector;
        // Vector2D<units::meters_per_second_squared_t> m_acceleration_vector;

        // units::degree_t m_rotation_degrees;
        // units::degrees_per_second_t m_rotational_velocity_degrees;
        // units::degrees_per_second_squared_t m_rotational_acceleration_degrees;

        // units::degree_t m_heading_degrees;
        // units::meter_t m_distance;
        // units::meters_per_second_t m_velocity;
        // units::meters_per_second_squared_t m_acceleration;
        // double m_curvature;

        // units::meters_per_second_t m_max_velocity;

    return state1;
}


//==============================================================================
// Logging

void SwerveFollower::StartLogging() {
    // If the log fileanem is not set then we do no logging
    if (m_log_filename == nullptr) return;

    // Open the log file and bail out if it fails
    m_csv_log_file = new Logging::CsvFile();
    m_csv_log_file->OpenLogFile(m_log_filename, std::ios::out | std::ios::app);
    if (m_csv_log_file->Fail()) {
        // If opening the file fails stop logging
        std::cout << "ERROR: SwerveFollower::Initialize() - Failed to open log file\n";
        delete m_csv_log_file;
        m_csv_log_file = nullptr;
        return;
    }

    // Log an initial heading
    *m_csv_log_file << "SwerveFollower" << "\n";

    // Log the parameters we are using
    *m_csv_log_file << "Parameters" << "\n";
    *m_csv_log_file << "Lookahead (m)" << m_follower_parameters.m_lookahead_distance.value() << "\n";
    *m_csv_log_file << "Lookahead Factor" << m_follower_parameters.m_lookahead_factor << "\n";
    *m_csv_log_file << "Correction Blend" << m_follower_parameters.m_correction_blend << "\n";
    *m_csv_log_file <<"\n";

    // Log the trajectory we are following
    *m_csv_log_file << "Trajectory" << "\n";
    m_trajectory.LogToFile(*m_csv_log_file);
    *m_csv_log_file <<"\n";

    // Log a heading for the data that will following in each update period
    *m_csv_log_file << "Follower Data" << "\n";
    *m_csv_log_file << "Time";
    *m_csv_log_file << "X" << "Y" << "Rotation";
    *m_csv_log_file << "M1 angle" << "M1 velocity" << "M1 acceleration";
    *m_csv_log_file <<"\n";
    m_csv_log_file->SetPrecision(3, true);

    // Start the timer
    m_timer.Reset();
    m_timer.Start();
}

void SwerveFollower::UpdateLogging(const frc::Pose2d& pose, const std::array<SwerveFollowerModuleState, 4>& result) {
    // If the CSV log file is null we are not doing any logging
    if (m_csv_log_file == nullptr) return;

    *m_csv_log_file << m_timer.Get().value();
    *m_csv_log_file << pose.X().value() << pose.Y().value() << pose.Rotation().Degrees().value();
    *m_csv_log_file << result[0].angle.Degrees().value() << result[0].speed.value() << result[0].acceleration.value();
    *m_csv_log_file <<"\n";
}

void SwerveFollower::EndLogging() {
    // If the CSV log file is null we are not doing any logging
    if (m_csv_log_file == nullptr) return;

    // Log two blank rows to separate any data appended later
    *m_csv_log_file <<"\n";
    *m_csv_log_file <<"\n";

    // Close the file and delete the file object
    m_csv_log_file->Close();
    delete m_csv_log_file;
    m_csv_log_file = nullptr;
}

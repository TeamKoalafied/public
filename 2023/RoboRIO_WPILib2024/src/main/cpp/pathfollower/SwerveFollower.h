//==============================================================================
// SwerveFollower.h
//==============================================================================

#pragma once

#include "SwerveTrajectory.h"
#include "SwerveFollowerModuleState.h"

#include "Point2D.h"

#include <frc/Timer.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <units/length.h>
#include <units/time.h>
#include <units/voltage.h>

namespace Logging { class CsvFile; }


class SwerveFollower {
public:
    //==========================================================================
    // Public Nested Types

    // Parameters that control how the path is generated and followed
    struct FollowerParameters {
        // double m_kp;                    // P gain for the PID control of wheel velocity
        // double m_ki;                    // I gain for the PID control of wheel velocity
        // double m_kd;                    // D gain for the PID control of wheel velocity

        // double m_kv;                    // Velocity constant [output/velocity]
        // double m_kv_offset;             // Velocity offsee [output]
        // double m_ka;                    // Acceleration constant [output/acceleration]
        // double m_period_s;              // Update period in seconds
        // double m_wheelbase_width_m;     // Width of the wheelbase in metres

        // double m_path_point_spacing;  // Spacing between generated path points
        // double m_max_velocity;          // Maximum velocity allowed on the path
        // double m_max_acceleration;      // Maximum acceleration allowed on the path
        // double m_max_velocity_curve;    // Scale factor to determine maximum cornering velocity from curvature

        units::meter_t m_lookahead_distance;    // Lookahead distance
        double m_lookahead_factor;      // Scale factor to get minimum lookahead distance from closest point distance
        double m_correction_blend;        
        // double m_lookahead_curvature_gain;  // Gain applied to the curvature from the pure pursuit lookahead
        // double m_path_curvature_gain;   // Gain applied to the curvature from the path
        // double m_lookalong_time;        // Time to look along the path into the future to get the velocity and acceleration
    };



    // Constructor
    //
    // trajectory - Trajectory to follow
    // kinematics - Drivebase kinematics
    // log_filename - Name of the file to log to, or null for none
    SwerveFollower(const SwerveTrajectory& trajectory, frc::SwerveDriveKinematics<4>& kinematics, const char* log_filename);

    void Initialize();

    std::array<SwerveFollowerModuleState, 4> Update(const frc::Pose2d& pose);
//    void Execute();
//    void End(bool interrupted);
    bool IsFinished();
    void End();

 private:
    //==========================================================================
    // Pure Pursuit Calculations

    // Calculate the closest point on the path to the current robot position and the desired
    // velocity, acceleration and curvature. Only searches forward from the last point found.
    //
    // robot_position - Current robot position
    // path_velocity - Returns the target velocity for this point on the path
    // path_acceleration - Returns the target acceleration for this point on the path
    // path_curvature - Returns the target curvature for this point on the path
    //
    // Returns the closest point on the curve. If past the end of the path 'm_segment_finished'
    // will be set true and the final path point returned (final point of the actual path,
    // not the path 'extension')
    SwerveTrajectory::Point GetClosestPoint(frc::Pose2d robot_position);


    // Calculate the lookahead point. Only searches forward from the last point found.
    //
    // robot_position - Current robot position
    // closest_point_distance - Current distance from the robot 
    //
    // Returns the lookahead position. If a lookahed point cannot be found 'm_segment_finished'
    // will be set true and the robot position returned.
    SwerveTrajectory::Point GetLookAheadPoint(frc::Pose2d robot_position, units::meter_t closest_point_distance);


    frc::Translation2d ProjectPointOntoLine(frc::Translation2d pt, frc::Translation2d line1, frc::Translation2d line2, double& relative);
    Point2D ProjectPointOntoLine(Point2D pt, Point2D line1, Point2D line2, double& relative);
    Vector2D<units::meter_t> ProjectPointOntoLine(Vector2D<units::meter_t> pt, Vector2D<units::meter_t> line1, Vector2D<units::meter_t> line2, double& relative);

    //==========================================================================
    // Trajectory Operations

    /**
     * Sample the trajectory at a point in time.
     *
     * @param t The point in time since the beginning of the trajectory to sample.
     * @return The state at that point in time.
     */
    //static frc::Trajectory::State Sample(const frc::Trajectory& trajectory, units::second_t t);


    static SwerveTrajectory::Point Lerp(const SwerveTrajectory::Point& state1, const SwerveTrajectory::Point& state2, double t);


    //==========================================================================
    // Logging

    // Start the CSV logging, if required, and log any initial information
    void StartLogging();

    // Update the CSV logging, if required
    //
    // pose - Current pose of the robot
    // result - Calculated swerve module states
    void UpdateLogging(const frc::Pose2d& pose, const std::array<SwerveFollowerModuleState, 4>& result);

    // End the CSV logging, if required
    void EndLogging();


    //==========================================================================
    // Member Variables

    const char* m_log_filename;                 // Name of the file to log to, or null for none
    SwerveTrajectory  m_trajectory;
    frc::SwerveDriveKinematics<4>& m_kinematics;
    std::function<frc::Pose2d()> m_get_pose_function;
    std::function<void(std::array<frc::SwerveModuleState, 4>)> m_set_module_states_function;

    FollowerParameters m_follower_parameters;   // Parameters that control how the path is followed

    int m_closest_index;                        // Index of the closest path point at the last update
    int m_lookahead_index;                      // Index of the lookahead path point at the last update
    bool m_segment_finished;                    // Flag to indicate that the trajectory is finished

    Logging::CsvFile* m_csv_log_file = nullptr; // CSV log file being written to, or nullptr for none
    frc::Timer m_timer;
};

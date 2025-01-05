//==============================================================================
// AutoTestSwerveFollower.h
//==============================================================================

#pragma once

#include "IAutonomousProvider.h"

// #include "../pathfollower/SwerveTrajectory.h"
// #include "../pathfollower/SwerveTrajectoryGenerator.h"

#include <frc/smartdashboard/SendableChooser.h>
#include <units/length.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <units/acceleration.h>

namespace frc { class SimpleWidget; }


// Create the auto commands for testing the swerve drive oath following code
class AutoTestSwerveFollower : public IAutonomousProvider {
public:
    //==========================================================================
    // Construction

    // Construtor
    AutoTestSwerveFollower();

    //==========================================================================
    // Virtual Functions from IAutonomousProvider
    virtual const char* Name() override;
    virtual int SetupDashboard(frc::ShuffleboardTab& auto_tab, int x_pos, int y_pos) override;
    virtual frc2::CommandPtr CreateAutonomousCommand(SwerveDrivebase& drivebase, Manipulator& manipulator) override;
    //==========================================================================

private:
    //==========================================================================
    // Private Nested Types

    // Template class to store values of a type paired with a string label for initialising
    // a SendableChooser widget
    template <class T>
    struct ChooserValue {
        T value;
        std::string label;
    };

    // Type of test path to generate
    enum class PathType {
        Forward,        // Move forward
        Turn,           // Move forward and turn
        Slalom,         // Shimmy sideways
        TestTurn,
        FilePath        // First path loaded from a file
    };

    // The follower to use for navigating the path
    enum class FollowerType {
        WPILib,         // WPILib
        PathPlanner,    // Path Planner library
        SwerveFollower, // Koalafieds SwerveFollower
    };

    // Shuffleboard widgets for adjusting test parameters
    struct ShuffleboardWidgets {
        frc::SendableChooser<PathType> m_path_chooser; // Smart dashboard chooser for the autonomouse path type
        frc::SendableChooser<FollowerType> m_follower_chooser; // Smart dashboard chooser for the autonomouse follower type
        frc::SimpleWidget* m_length;                // Widget for entering the length of the path (m)
        frc::SimpleWidget* m_width;                 // Widget for entering the width of the path (m)
        frc::SimpleWidget* m_rotatation;            // Widget for entering the rotations (degrees)
        frc::SimpleWidget* m_max_speed;             // Widget for entering the maximum speed (m/s)
        frc::SimpleWidget* m_max_acceleration;      // Widget for entering the maximum acceleration (m2/s)
        frc::SimpleWidget* m_repeat_count;          // Widget for entering the repeat count
        frc::SimpleWidget* m_lookahead_distance;    // Widget for the follower look ahead distance (m)
        frc::SimpleWidget* m_lookahead_factor;      // Widget for the follower look ahead factor
        frc::SimpleWidget* m_correction_blend;      // Widget for the follower correction blend

        frc::SimpleWidget* m_direction_p;           // Widget for the P gain of the direction PID controller
        frc::SimpleWidget* m_direction_d;           // Widget for the D gain of the direction PID controller
        frc::SimpleWidget* m_rotation_p;            // Widget for the P gain of the rotation PID controller
        frc::SimpleWidget* m_rotation_d;            // Widget for the D gain of the rotation PID controller

    };

    //==========================================================================
    // Create WPILib Commands

    frc2::CommandPtr CreateWPILibCommand(SwerveDrivebase& drivebase, PathType path_type, units::meter_t length,
                                         units::meter_t width, units::degree_t rotatation,
                                         units::meters_per_second_t max_velocity,
                                         units::meters_per_second_squared_t m_max_acceleration);


    //==========================================================================
    // Create PathPlanner Commands

    frc2::CommandPtr CreatePathPlannerCommand(SwerveDrivebase& drivebase, PathType path_type, units::meter_t length,
                                              units::meter_t width, units::degree_t rotatation,
                                              units::meters_per_second_t max_velocity,
                                               units::meters_per_second_squared_t m_max_acceleration,
                                               int repeat_count);
    void GetPathPlannerPathFilePaths(std::vector<std::string>& path_names);
    void GetPathPlannerAutoFilePaths(std::vector<std::string>& path_names);



    //==========================================================================
    // Creating Swerve Follower Commands

    frc2::CommandPtr CreateSwerveFollowerCommand(SwerveDrivebase& drivebase, PathType path_type, units::meter_t length, units::meter_t width, units::degree_t rotatation,
                                                 units::meters_per_second_t max_velocity, units::meters_per_second_squared_t m_max_acceleration);


    // SwerveTrajectory CreateForwardSwerveTrajectory(const SwerveTrajectoryGenerator::Parameters& parameters, units::meter_t distance, units::degree_t start_rotation, units::degree_t end_rotation);
    // SwerveTrajectory CreateSlalomSwerveTrajectory(units::meter_t distance, units::meter_t width, units::meter_t length);
    frc2::CommandPtr CreateLogDrivebase(SwerveDrivebase& drivebase);




    //==========================================================================
    // Member Data

    ShuffleboardWidgets* m_shuffleboard_widgets;    // Shuffleboard widgets for adjusting test parameters. Null if not initialised.
    std::vector<std::string> m_path_planner_names;  // Names of path planner file paths found

    //==========================================================================
    // Static Data

    static const ChooserValue<PathType> kPathTypeMapping[];
                                                    // Mapping between the labels on the dashboard and the path type
    static const int kPathTypeMappingCount;         // Number of entries in the mapping between the labels on the dashboard and the path type
    static const ChooserValue<FollowerType> kFollowerTypeMapping[];
                                                    // Mapping between the labels on the dashboard and the follower type
    static const int kFollowerTypeMappingCount;     // Number of entries in the mapping between the labels on the dashboard and the folloer type
};
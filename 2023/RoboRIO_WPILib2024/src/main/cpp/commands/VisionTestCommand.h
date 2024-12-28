//==============================================================================
// VisionTestCommand.h
//==============================================================================

#pragma once

#include "IAutonomousProvider.h"
#include "../subsystems/SwerveDrivebase.h"
// #include "../pathfollower/SwerveTrajectory.h"
// #include "../pathfollower/SwerveTrajectoryGenerator.h"
#include <pathplanner/lib/path/PathPlannerPath.h>

#include <frc/smartdashboard/SendableChooser.h>
#include <units/length.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <units/acceleration.h>

namespace frc { class SimpleWidget; }


// Create the auto commands for testing the swerve drive oath following code
class VisionTestCommand : public IAutonomousProvider {
public:
    //==========================================================================
    // Construction

    // Construtor
    VisionTestCommand();

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
        Horizontal,        // Move forward
        Vertical,           // Move forward and turn
  
    };

    // Shuffleboard widgets for adjusting test parameters
    struct ShuffleboardWidgets {
        frc::SendableChooser<PathType> m_path_chooser; // Smart dashboard chooser for the autonomouse path type
        frc::SimpleWidget* m_length;                // Widget for entering the length of the path (m)
        frc::SimpleWidget* m_width;                 // Widget for entering the width of the path (m)
        frc::SimpleWidget* m_rotatation;            // Widget for entering the rotations (degrees)
        
        frc::SimpleWidget* m_max_speed;             // Widget for entering the maximum speed (m/s)
        frc::SimpleWidget* m_max_acceleration;      // Widget for entering the maximum acceleration (m2/s)
        frc::SimpleWidget* m_repeat_count;          // Widget for entering the repeat count

        frc::SimpleWidget* m_direction_p;           // Widget for the P gain of the direction PID controller
        frc::SimpleWidget* m_direction_d;           // Widget for the D gain of the direction PID controller
        frc::SimpleWidget* m_rotation_p;            // Widget for the P gain of the rotation PID controller
        frc::SimpleWidget* m_rotation_d;            // Widget for the D gain of the rotation PID controller

    };
    
    //==========================================================================
    // Create PathPlanner Commands

    frc2::CommandPtr CreatePathPlannerCommand(SwerveDrivebase& drivebase, PathType path_type, units::meter_t length,
                                              units::meter_t width, units::degree_t rotatation,
                                              units::meters_per_second_t max_velocity,
                                               units::meters_per_second_squared_t m_max_acceleration,
                                               int repeat_count);
    // template<typename... Args>
    std::shared_ptr<pathplanner::PathPlannerPath> GeneratePathSegment(pathplanner::PathConstraints constraints, 
                                                                        units::degree_t rotation, 
                                                                        std::span<frc::Pose2d> poses);

    std::shared_ptr<pathplanner::PathPlannerPath> GeneratePathSegment(pathplanner::PathConstraints constraints, 
                                                                      frc::Pose2d start, frc::Pose2d finish);



    void PrintFinalPosition(SwerveDrivebase& drivebase);

    //==========================================================================
    // Member Data

    ShuffleboardWidgets* m_shuffleboard_widgets;    // Shuffleboard widgets for adjusting test parameters. Null if not initialised.

    //==========================================================================
    // Static Data

    static const ChooserValue<PathType> kPathTypeMapping[];
                                                    // Mapping between the labels on the dashboard and the path type
    static const int kPathTypeMappingCount;         // Number of entries in the mapping between the labels on the dashboard and the path type
                                                    // Mapping between the labels on the dashboard and the follower type
    static const int kFollowerTypeMappingCount;     // Number of entries in the mapping between the labels on the dashboard and the folloer type
};
//==============================================================================
// SwerveDrivebaseShuffleboard.h
//==============================================================================

#pragma once

#include "SwerveDrivebase.h"
#include "Manipulator.h"
#include <frc/smartdashboard/Field2d.h>

namespace frc {
    class ShuffleboardTab;
    class SimpleWidget;
}

// Used to publish relevant drivebase information to the shuffleboard
class SwerveDrivebaseShuffleboard {
public:
    // Constructs SwerveDrivebaseShuffleboard object
    //
    // drivebase - reference to the drivebase
    // manipulator - reference to the manipulator
    SwerveDrivebaseShuffleboard(const SwerveDrivebase* drivebase, const Manipulator* manipulator);

    // Contains robot state values relevant for driving robot during a match
    void DriveTabSetup();

    // Contains summary data for each swerve module
    void DrivebaseTabSetup();

    // Contains pose estimation and vision data
    void VisionTabSetup();

    // Updates all published data
    void UpdateShuffleboard();

    // Get the slowdown factor the use has entered
    double GetSlowdownFactor();

private:
    // Struct containing the relevant widgets for the drivebase tab (except module-specific)
    struct DrivebaseTabWidgets {
        frc::SimpleWidget* m_robot_x_widget;            // Shuffleboard widget for displaying the current x velocity of the robot
        frc::SimpleWidget* m_robot_y_widget;            // Shuffleboard widget for displaying the current y velocity of the robot
        frc::SimpleWidget* m_robot_rotation_widget;     // Shuffleboard widget for displaying the current rotational velocity of the robot
        frc::Field2d m_field_widget;                    // Widget for displaying the robot field position on the shuffleboard
    };

    // Struct containing widgets relevant to the vision tab
    struct VisionTabWidgets {
        frc::SimpleWidget* m_estimator_x;                  // Pose estimator x position
        frc::SimpleWidget* m_estimator_y;                  // Pose estimator y position
        frc::SimpleWidget* m_estimator_heading;            // Pose estimator heading
        frc::SimpleWidget* m_odometry_x;                // Odometry x position
        frc::SimpleWidget* m_odometry_y;                // Odometry y position
        frc::SimpleWidget* m_odometry_heading;          // Odometry heading
        frc::SimpleWidget* m_photon_x;                  // Vision (AprilTqag) x position
        frc::SimpleWidget* m_photon_y;                  // Vision (AprilTqag) y position
        frc::SimpleWidget* m_photon_heading;            // Vision (AprilTqag) heading
        frc::SimpleWidget* m_targets;                   // Boolean for if AprilTag currently in camera view
        frc::SimpleWidget* m_has_pose;                  // Boolean for if photonvision currently has an estimated pose
        frc::SimpleWidget* m_target_area;               // Area of the best April tag currently in view
        frc::Field2d*  m_field_widget;                  // Robot pose on field
    };

    // Struct containing widgets relevant to the drive tab
    struct DriveTabWidgets {
        frc::SimpleWidget* m_slowdown_widget;           // Slowdown factor for the drivebase
        frc::SimpleWidget* m_field_relative_widget;     // Boolean for if field-relative is enabled
        frc::SimpleWidget* m_roll_widget;               // Current roll angle of the robot
        frc::SimpleWidget* m_pitch_widget;              // Current pitch angle of the robot
        frc::SimpleWidget* m_cone_widget;               // Boolean for if in cone mode
        frc::SimpleWidget* m_cube_widget;               // Boolean for if in cube mode

        frc::SimpleWidget* m_drivebase_ok_widget;       // Boolean for whether the drivebase is OK
        frc::SimpleWidget* m_vision_ok_widget;          // Boolean for whether the drivebase is OK
        frc::SimpleWidget* m_manipulator_ok_widget;     // Boolean for whether the drivebase is OK
        frc::SimpleWidget* m_test_widget;               // Boolean for testing

    };

    DrivebaseTabWidgets* m_drivebase_widgets;           // Widgets on the Drivebase tab or nullptr if not being used
    VisionTabWidgets* m_vision_widgets;                 // Widgets on the Vision tab or nullptr if not being used
    DriveTabWidgets* m_drive_tab_widgets;               // Widgets on the Drive tab or nullptr if not being used

    const SwerveDrivebase* m_drivebase;                 // Reference to the drivebase for getting current values
    const Manipulator* m_manipulator;                   // Reference to the manipulator for getting current values
};
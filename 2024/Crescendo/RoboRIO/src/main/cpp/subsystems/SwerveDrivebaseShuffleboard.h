//==============================================================================
// SwerveDrivebaseShuffleboard.h
//==============================================================================

#pragma once

#include <frc/smartdashboard/Field2d.h>
#include <wpi/array.h>

namespace frc {
    class ShuffleboardTab;
    class SimpleWidget;
}
class SwerveDrivebase;
class Manipulator;


// Used to publish relevant drivebase information to the shuffleboard
class SwerveDrivebaseShuffleboard {
public:
    // Constructs SwerveDrivebaseShuffleboard object
    //
    // drivebase - reference to the drivebase
    // manipulator - reference to the manipulator
    SwerveDrivebaseShuffleboard(const SwerveDrivebase* drivebase, const Manipulator* manipulator);

    // Contains summary data for each swerve module
    void DrivebaseTabSetup();

    // Contains pose estimation and vision data
    void VisionTabSetup();

    // Updates all published data
    void UpdateShuffleboard();

    wpi::array<double, 3> GetStdDevs();

    units::degree_t GetToeInAngle();

private:
    // Struct containing the relevant widgets for the drivebase tab (except module-specific)
    struct DrivebaseTabWidgets {
        frc::SimpleWidget* m_robot_x_widget;            // Shuffleboard widget for displaying the current x velocity of the robot
        frc::SimpleWidget* m_robot_y_widget;            // Shuffleboard widget for displaying the current y velocity of the robot
        frc::SimpleWidget* m_robot_rotation_widget;     // Shuffleboard widget for displaying the current rotational velocity of the robot
        frc::Field2d m_field_widget;                    // Widget for displaying the robot field position on the shuffleboard

        frc::SimpleWidget* m_toe_in_widget;             // Toe-in angle in degrees
    };

    // Struct containing widgets relevant to the vision tab
    struct VisionTabWidgets {
        frc::SimpleWidget* m_estimator_x;               // Pose estimator x position
        frc::SimpleWidget* m_estimator_y;               // Pose estimator y position
        frc::SimpleWidget* m_estimator_heading;         // Pose estimator heading
        frc::SimpleWidget* m_odometry_x;                // Odometry x position
        frc::SimpleWidget* m_odometry_y;                // Odometry y position
        frc::SimpleWidget* m_odometry_heading;          // Odometry heading
        frc::SimpleWidget* m_photon_x;                  // Vision (AprilTqag) x position
        frc::SimpleWidget* m_photon_y;                  // Vision (AprilTqag) y position
        frc::SimpleWidget* m_photon_heading;            // Vision (AprilTqag) heading
        frc::SimpleWidget* m_targets;                   // Boolean for if AprilTag currently in camera view
        frc::SimpleWidget* m_has_pose;                  // Boolean for if photonvision currently has an estimated pose
        frc::SimpleWidget* m_target_area;               // Area of the best April tag currently in view
        frc::SimpleWidget* m_speaker_pose_ambiguity;    // Pose amgiguity of the centre speaker tag or -1 if not visible
        frc::SimpleWidget* m_speaker_angle;             // Angle to the centre speaker tag
        frc::SimpleWidget* m_speaker_distance;          // Distance to the centre speaker tag
        frc::Field2d*  m_field_widget;                  // Robot pose on field
        frc::SimpleWidget* m_x_dev_widget;
        frc::SimpleWidget* m_y_dev_widget;
        frc::SimpleWidget* m_theta_dev_widget;
    };

    DrivebaseTabWidgets* m_drivebase_widgets;           // Widgets on the Drivebase tab or nullptr if not being used
    VisionTabWidgets* m_vision_widgets;                 // Widgets on the Vision tab or nullptr if not being used

    const SwerveDrivebase* m_drivebase;                 // Reference to the drivebase for getting current values
    const Manipulator* m_manipulator;                   // Reference to the manipulator for getting current values
};
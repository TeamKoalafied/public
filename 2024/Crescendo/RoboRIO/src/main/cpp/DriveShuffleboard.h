//==============================================================================
// DriveShuffleboard.h
//==============================================================================

#pragma once

#include <frc/smartdashboard/Field2d.h>

namespace frc {
    class ShuffleboardTab;
    class SimpleWidget;
}
class SwerveDrivebase;
class Manipulator;


// Shuffleboard display for overall robot control. Create the main 'Drive' tab
class DriveShuffleboard {
public:
    // Constructor
    //
    // drivebase - Reference to the drivebase
    // manipulator - Reference to the manipulator
    DriveShuffleboard(const SwerveDrivebase* drivebase, const Manipulator* manipulator);

    // Contains robot state values relevant for driving robot during a match
    //
    // game_auto - Whether the drive tab should include the controls for getting the game auto
    void DriveTabSetup(bool game_auto);

    // Updates all published data
    void UpdateShuffleboard();

    // Get the slowdown factor the use has entered
    double GetSlowdownFactor();

private:

    // Struct containing widgets relevant to the drive tab
    struct DriveTabWidgets {
        frc::SimpleWidget* m_slowdown_widget;           // Slowdown factor for the drivebase
        frc::SimpleWidget* m_field_relative_widget;     // Boolean for if field-relative is enabled
        frc::SimpleWidget* m_use_april_tags_widget;     // Boolean for if april tags are being used
        frc::SimpleWidget* m_lift_widget;               // Boolean for if in trap/amp mode
        frc::SimpleWidget* m_shooter_widget;            // Boolean for if in lift mode
        frc::SimpleWidget* m_distance_widget;
        frc::SimpleWidget** m_shoot_distance_widgets;   // Array of widges for fixed shooter distances
        frc::SimpleWidget* m_pivot_angle_widget;        // Manual shooter pivot angle
        frc::SimpleWidget* m_shooter_rpm_widget;        // Manual shooter speed (rpm)
        frc::Field2d       m_field_widget;              // Robot pose on field
        frc::SimpleWidget* m_prefire_shooter_speed;     // Current prefire shooter speed. Not updated when not prefiring.
        frc::SimpleWidget* m_prefire_pivot_angle;       // Current prefire pivot angle. Not updated when not prefiring.
        frc::SimpleWidget* m_pointing_at_target;        // Whether the robot is pointing at the target when in shooting state
        frc::SimpleWidget* m_prefire_ready;             // Get whether the prefire shooter RPM and pivot angle have been achieved
    };

    DriveTabWidgets* m_drive_tab_widgets;               // Widgets on the Drive tab or nullptr if not being used

    const SwerveDrivebase* m_drivebase;                 // Reference to the drivebase for getting current values
    const Manipulator* m_manipulator;                   // Reference to the manipulator for getting current values
};

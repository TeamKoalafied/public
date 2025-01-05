//==============================================================================
// DriveShuffleboard.cpp
//==============================================================================

#include "DriveShuffleboard.h"

#include "commands/AutonomousCommand.h"
#include "subsystems/SwerveDrivebase.h"
#include "subsystems/Manipulator.h"
#include "subsystems/Vision.h"

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardContainer.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <frc/kinematics/SwerveDriveOdometry.h>


DriveShuffleboard::DriveShuffleboard(const SwerveDrivebase* drivebase, const Manipulator* manipulator) {
    m_drivebase = drivebase;
    m_manipulator = manipulator;
}

void DriveShuffleboard::DriveTabSetup(bool game_auto) {
    // Create a 'Drive' shuffleboard tab and the object to store its widgets
    frc::ShuffleboardTab& drive_tab = frc::Shuffleboard::GetTab("Drive");
    m_drive_tab_widgets = new DriveTabWidgets;
    DriveTabWidgets* dt = m_drive_tab_widgets;

    // Create widgets for driving control. Note that for a numerical control to display
    // a double number it must be initialised with one (so 0.0, rather than 0) otherwise
    // later it will only display ints.
    dt->m_slowdown_widget = &drive_tab.Add("Slowdown factor", RC::kDefaultDrivebaseSlowDownFactor)
                                        .WithPosition(0,0).WithSize(3,2);
    dt->m_field_relative_widget = &drive_tab.Add("Field relative", false)
                                            .WithPosition(3,0)
                                            .WithSize(3,2)
                                            .WithWidget(frc::BuiltInWidgets::kBooleanBox);
    dt->m_use_april_tags_widget = &drive_tab.Add("April Tags", false).WithPosition(6,0).WithSize(3,2);

    dt->m_lift_widget = &drive_tab.Add("TrapAmp", false)
								.WithPosition(0,2)
								.WithSize(3,2)
								.WithWidget(frc::BuiltInWidgets::kBooleanBox);
	dt->m_shooter_widget = &drive_tab.Add("Shooter", false)
								.WithPosition(3,2)
								.WithSize(3,2)
								.WithWidget(frc::BuiltInWidgets::kBooleanBox);
    dt->m_distance_widget = &drive_tab.Add("Distance",0.0).WithPosition(9,0).WithSize(3,2);


    dt->m_pivot_angle_widget = &drive_tab.Add("Manual Pivot",0.0).WithPosition(6,2).WithSize(3,2);
    dt->m_shooter_rpm_widget = &drive_tab.Add("Manual RPM",0.0).WithPosition(9,2).WithSize(3,2);
    dt->m_prefire_pivot_angle = &drive_tab.Add("Prefire Pivot",0.0).WithPosition(6,4).WithSize(3,2);
    dt->m_prefire_shooter_speed = &drive_tab.Add("Prefire RPM",0.0).WithPosition(9,4).WithSize(3,2);

    dt->m_pointing_at_target = &drive_tab.Add("Pointing Taget", false)
                                        .WithPosition(6,6)
                                        .WithSize(3,2)
                                        .WithWidget(frc::BuiltInWidgets::kBooleanBox);
    dt->m_prefire_ready = &drive_tab.Add("Prefire Ready", false)
                                        .WithPosition(9,6)
                                        .WithSize(3,2)
                                        .WithWidget(frc::BuiltInWidgets::kBooleanBox);


    // Create a field widget showing the current robot pose
    drive_tab.Add("Field2", dt->m_field_widget)
         .WithWidget(frc::BuiltInWidgets::kField)
         .WithPosition(12, 0)
         .WithSize(15, 10);

    // Setup the game auto, if required
    if (game_auto) {
        AutonomousCommand::SetupGameAutonomousShuffleboard(drive_tab, 0, 4);
    }
}

void DriveShuffleboard::UpdateShuffleboard() {
    // If the 'Drive' widgets have been initialised, update them
    if (m_drive_tab_widgets != nullptr) {
        // Get the manipulator state, but only if it exists
        Manipulator::DiverterPosition diverter_pos = Manipulator::DiverterPosition::Lift;
        units::degree_t pivot_angle_degrees = 0_deg;
        units::revolutions_per_minute_t shooter_rmp = 0_rpm;
        if (m_manipulator) {
            diverter_pos = m_manipulator->GetDiverterPos();
            pivot_angle_degrees = m_manipulator->GetManualPivotAngle();
            shooter_rmp = m_manipulator->GetManualShooterRPM();
        }

        DriveTabWidgets* dt = m_drive_tab_widgets;

        // Do not update the slowdown factor as it is for user entry
        dt->m_field_relative_widget->GetEntry()->SetBoolean(m_drivebase->GetFieldRelative());
        dt->m_use_april_tags_widget->GetEntry()->SetBoolean(m_drivebase->GetVision().GetUpdatePoseEnabled());
        dt->m_lift_widget->GetEntry()->SetBoolean(diverter_pos == Manipulator::DiverterPosition::Lift);
        dt->m_shooter_widget->GetEntry()->SetBoolean(diverter_pos == Manipulator::DiverterPosition::Shooter);
        dt->m_distance_widget->GetEntry()->SetDouble(m_drivebase->GetDistanceToTarget().value());
        dt->m_pivot_angle_widget->GetEntry()->SetDouble(pivot_angle_degrees.value());
        dt->m_shooter_rpm_widget->GetEntry()->SetDouble(shooter_rmp.value());

        // Current prefire. Only update if active so that the old value is still visible after doing targetting.
        units::revolutions_per_minute_t prefire_shooter_speed;
        units::degree_t prefire_pivot_angle;
        if (m_manipulator != nullptr && m_manipulator->GetPrefire(prefire_shooter_speed, prefire_pivot_angle)) {
            dt->m_prefire_pivot_angle->GetEntry()->SetDouble(prefire_pivot_angle.value());
            dt->m_prefire_shooter_speed->GetEntry()->SetDouble(prefire_shooter_speed.value());
        }

        // Whether the current prefire is on target direction and up to speed and angle.
        dt->m_pointing_at_target->GetEntry()->SetBoolean(m_drivebase->GetPointingAtTarget());
        dt->m_prefire_ready->GetEntry()->SetBoolean(m_manipulator->PrefireReady());


        frc::Pose2d swerve_pose = m_drivebase->GetPose();
        dt->m_field_widget.SetRobotPose(swerve_pose);
    }
}

double DriveShuffleboard::GetSlowdownFactor() {
    // If the 'Drive' widgets have been initialised, get the slowdown factor from the shuffleboard,
    // otherwise use the default
    if (m_drive_tab_widgets != nullptr) {
        DriveTabWidgets* dt = m_drive_tab_widgets;

        return dt->m_slowdown_widget->GetEntry()->GetDouble(RC::kDefaultDrivebaseSlowDownFactor);
    } else {
        return RC::kDefaultDrivebaseSlowDownFactor;
    }
}

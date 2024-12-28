#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardContainer.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <frc/kinematics/SwerveDriveOdometry.h>

#include "SwerveDrivebaseShuffleboard.h"
#include "Vision.h"


SwerveDrivebaseShuffleboard::SwerveDrivebaseShuffleboard(const SwerveDrivebase* drivebase, const Manipulator* manipulator) {
    m_drivebase = drivebase;
    m_manipulator = manipulator;
    m_drivebase_widgets = nullptr;
    m_drive_tab_widgets = nullptr;
    m_vision_widgets = nullptr;
}

void SwerveDrivebaseShuffleboard::DriveTabSetup() {
    // Create a 'Drive' shuffleboard tab and the object to store its widgets
    frc::ShuffleboardTab& drive_tab = frc::Shuffleboard::GetTab("Drive");
    m_drive_tab_widgets = new DriveTabWidgets;
    DriveTabWidgets* dt = m_drive_tab_widgets;

    // Create widgets for driving control. Note that for a numerical control to display
    // a double number it must be initialised with one (so 0.0, rather than 0) otherwise
    // later it will only display ints.
    dt->m_slowdown_widget = &drive_tab.Add("Slowdown factor", RC::kDefaultDrivebaseSlowDownFactor)
                                        .WithPosition(0,0).WithSize(3,2).WithWidget(frc::BuiltInWidgets::kTextView);
    dt->m_field_relative_widget = &drive_tab.Add("Field relative", false)
                                            .WithPosition(3,0)
                                            .WithSize(3,2)
                                            .WithWidget(frc::BuiltInWidgets::kBooleanBox);
    dt->m_pitch_widget = &drive_tab.Add("Pitch", 0.0).WithPosition(6,0).WithSize(3,2).WithWidget(frc::BuiltInWidgets::kTextView);
    dt->m_roll_widget = &drive_tab.Add("Roll", 0.0).WithPosition(9,0).WithSize(3,2).WithWidget(frc::BuiltInWidgets::kTextView);

    dt->m_cone_widget = &drive_tab.Add("Cone", false)
                                .WithPosition(0,2)
                                .WithSize(3,2)
                                .WithWidget(frc::BuiltInWidgets::kBooleanBox);
    dt->m_cube_widget = &drive_tab.Add("Cube", false)
                                .WithPosition(3,2)
                                .WithSize(3,2)
                                .WithWidget(frc::BuiltInWidgets::kBooleanBox);

    dt->m_drivebase_ok_widget = &drive_tab.Add("Drivebase OK", false)
                                .WithPosition(12, 0)
                                .WithSize(3,2)
                                .WithWidget(frc::BuiltInWidgets::kBooleanBox);
    dt->m_vision_ok_widget = &drive_tab.Add("Vision OK", false)
                                .WithPosition(12, 2)
                                .WithSize(3,2)
                                .WithWidget(frc::BuiltInWidgets::kBooleanBox);
    dt->m_manipulator_ok_widget = &drive_tab.Add("Manipulator OK", false)
                                .WithPosition(12, 4)
                                .WithSize(3,2)
                                .WithWidget(frc::BuiltInWidgets::kBooleanBox);
    dt->m_test_widget = &drive_tab.Add("Test", false)
                                .WithPosition(12, 6)
                                .WithSize(3,2)
                                .WithWidget(frc::BuiltInWidgets::kBooleanBox);
}

void SwerveDrivebaseShuffleboard::DrivebaseTabSetup() {
    // Create a 'DriveSummary' shuffleboard tab
    frc::ShuffleboardTab& shuffleboard_tab = frc::Shuffleboard::GetTab("DriveSummary");

    // Get each of the swerve modules and add their data to the tab
    wpi::array<const SwerveModule*, 4> modules = m_drivebase->GetModules();
    modules[0]->AddGyroSendable(shuffleboard_tab, 0, 0);
    modules[1]->AddGyroSendable(shuffleboard_tab, 5, 0);
    modules[2]->AddGyroSendable(shuffleboard_tab, 0, 5);
    modules[3]->AddGyroSendable(shuffleboard_tab, 5, 5);

    modules[0]->CreateShuffleboardSummary(shuffleboard_tab, 10, 0);
    modules[1]->CreateShuffleboardSummary(shuffleboard_tab, 16, 0);
    modules[2]->CreateShuffleboardSummary(shuffleboard_tab, 10, 6);
    modules[3]->CreateShuffleboardSummary(shuffleboard_tab, 16, 6);
        
    // Create the object for containing the other widgets for the 'DriveSummary' tab
    m_drivebase_widgets = new DrivebaseTabWidgets;
    DrivebaseTabWidgets* db = m_drivebase_widgets;

    // Create widgets for the x, y and rotational velocities
    db->m_robot_x_widget = &shuffleboard_tab.Add("Robot X", 0.0).WithPosition(22, 7).WithSize(3,2).WithWidget(frc::BuiltInWidgets::kTextView);
    db->m_robot_y_widget = &shuffleboard_tab.Add("Robot y", 0.0).WithPosition(25, 7).WithSize(3,2).WithWidget(frc::BuiltInWidgets::kTextView);
    db->m_robot_rotation_widget = &shuffleboard_tab.Add("Rotation", 0.0).WithPosition(28, 7).WithSize(3,2).WithWidget(frc::BuiltInWidgets::kTextView);
    
    // Create a field widget showing the current robot pose
    shuffleboard_tab.Add("Field", db->m_field_widget)
         .WithWidget(frc::BuiltInWidgets::kField)
         .WithPosition(22, 0)
         .WithSize(10,7);
}

void SwerveDrivebaseShuffleboard::VisionTabSetup() {
    if (m_drivebase->GetVision() == nullptr) return;

    // Create a 'Vision' shuffleboard tab and the object to store its widgets
    frc::ShuffleboardTab& vision_tab = frc::Shuffleboard::GetTab("Vision");
    m_vision_widgets = new VisionTabWidgets;
    VisionTabWidgets* vw = m_vision_widgets;

    // Create the widgets for pose estimation: x, y position and from odometry, vision and pose estimation
    vw->m_estimator_x = &vision_tab.Add("Estimator X", 0.0).WithPosition(0,0).WithSize(3,2).WithWidget(frc::BuiltInWidgets::kTextView);
    vw->m_estimator_y = &vision_tab.Add("Estimator Y", 0.0).WithPosition(0,2).WithSize(3,2).WithWidget(frc::BuiltInWidgets::kTextView);
    vw->m_estimator_heading = &vision_tab.Add("Estimator heading", 0.0).WithPosition(0,4).WithSize(3,2).WithWidget(frc::BuiltInWidgets::kTextView);
    vw->m_odometry_x = &vision_tab.Add("Odometry X", 0.0).WithPosition(3,0).WithSize(3,2).WithWidget(frc::BuiltInWidgets::kTextView);
    vw->m_odometry_y = &vision_tab.Add("Odometry Y", 0.0).WithPosition(3,2).WithSize(3,2).WithWidget(frc::BuiltInWidgets::kTextView);
    vw->m_odometry_heading = &vision_tab.Add("Odometry heading", 0.0).WithPosition(3,4).WithSize(3,2).WithWidget(frc::BuiltInWidgets::kTextView);
    vw->m_photon_x = &vision_tab.Add("Photon X", 0.0).WithPosition(6,0).WithSize(3,2).WithWidget(frc::BuiltInWidgets::kTextView);
    vw->m_photon_y = &vision_tab.Add("Photon Y", 0.0).WithPosition(6,2).WithSize(3,2).WithWidget(frc::BuiltInWidgets::kTextView);
    vw->m_photon_heading = &vision_tab.Add("Photon heading", 0.0).WithPosition(6,4).WithSize(3,2).WithWidget(frc::BuiltInWidgets::kTextView);

    // Create field widget for the vision tab
    vw->m_field_widget = new frc::Field2d;
    vision_tab.Add("Field", *vw->m_field_widget)
         .WithWidget(frc::BuiltInWidgets::kField)
         .WithPosition(0, 6)
         .WithSize(10,7);

    // Create boolean boxes for vision targeting and pose estimation
    vw->m_targets = &vision_tab.Add("Has targets", false)
                                .WithPosition(9,0)
                                .WithSize(3,3)
                                .WithWidget(frc::BuiltInWidgets::kBooleanBox);
    vw->m_has_pose = &vision_tab.Add("Has pose", false)
                                .WithPosition(9,3)
                                .WithSize(3,3)
                                .WithWidget(frc::BuiltInWidgets::kBooleanBox);
    vw->m_target_area = &vision_tab.Add("Target Area", 0.0).WithPosition(12,0).WithSize(3,2).WithWidget(frc::BuiltInWidgets::kTextView);
}

void SwerveDrivebaseShuffleboard::UpdateShuffleboard() {
    // Get swerve modules and call each ones update function
    wpi::array<const SwerveModule*, 4> modules = m_drivebase->GetModules();
    modules[0]->UpdateShuffleboard();
    modules[1]->UpdateShuffleboard();
    modules[2]->UpdateShuffleboard();
    modules[3]->UpdateShuffleboard();

    // Get the current robot pose for 'Drive' and 'DriveSummary' tabs
    frc::Pose2d swerve_pose = m_drivebase->GetPose();
    
    // If the 'DriveSummary' widgets have been initialised, update them
    if (m_drivebase_widgets != nullptr) {
        DrivebaseTabWidgets* db = m_drivebase_widgets;

        db->m_field_widget.SetRobotPose(swerve_pose);
        db->m_robot_x_widget->GetEntry()->SetDouble(swerve_pose.X().value());
        db->m_robot_y_widget->GetEntry()->SetDouble(swerve_pose.Y().value());
        db->m_robot_rotation_widget->GetEntry()->SetDouble(swerve_pose.Rotation().Degrees().value());
    }

    // If the 'Vision' widgets have been initialised, update them
    // Note: vision pose estimation doesn't work at the moment, so they arent updated.
    if (m_vision_widgets != nullptr) {
        frc::Pose2d estimated_pose = m_drivebase->GetVision()->GetPose();
        VisionTabWidgets* vw = m_vision_widgets;

        vw->m_estimator_x->GetEntry()->SetDouble(estimated_pose.X().value());
        vw->m_estimator_y->GetEntry()->SetDouble(estimated_pose.Y().value());
        vw->m_estimator_heading->GetEntry()->SetDouble(estimated_pose.Rotation().Degrees().value());

        vw->m_odometry_x->GetEntry()->SetDouble(swerve_pose.X().value());
        vw->m_odometry_y->GetEntry()->SetDouble(swerve_pose.Y().value());
        vw->m_odometry_heading->GetEntry()->SetDouble(swerve_pose.Rotation().Degrees().value());

        vw->m_field_widget->SetRobotPose(swerve_pose);

       bool has_pose = m_drivebase->GetVision()->GetLatestPhotonPose().has_value();
       vw->m_has_pose->GetEntry()->SetBoolean(has_pose);
        if (has_pose) {
            frc::Pose2d vision_pose = m_drivebase->GetVision()->GetLatestPhotonPose().value().estimatedPose.ToPose2d();
            vw->m_photon_x->GetEntry()->SetDouble(vision_pose.X().value());
            vw->m_photon_y->GetEntry()->SetDouble(vision_pose.Y().value());
            vw->m_photon_heading->GetEntry()->SetDouble(vision_pose.Rotation().Degrees().value());
            vw->m_targets->GetEntry()->SetBoolean(m_drivebase->GetVision()->GetLatestPhotonResult().HasTargets());
            vw->m_target_area->GetEntry()->SetDouble(m_drivebase->GetVision()->GetLatestPhotonResult().GetBestTarget().GetArea());
        } else {
            vw->m_photon_x->GetEntry()->SetDouble(0.0);
            vw->m_photon_y->GetEntry()->SetDouble(0.0);
            vw->m_photon_heading->GetEntry()->SetDouble(0.0);
            vw->m_targets->GetEntry()->SetBoolean(false);
            vw->m_target_area->GetEntry()->SetDouble(0.0);
        }
    }

    // If the 'Drive' widgets have been initialised, update them
    if (m_drive_tab_widgets != nullptr) {
        Manipulator::GamePiece game_piece = Manipulator::GamePiece::Cone;
        if (m_manipulator != nullptr) {
            game_piece = m_manipulator->GetGamePiece();
        }
        DriveTabWidgets* dt = m_drive_tab_widgets;

        // Do not update the slowdown factor as it is for user entry
        dt->m_field_relative_widget->GetEntry()->SetDouble(m_drivebase->GetFieldRelative());
        dt->m_roll_widget->GetEntry()->SetDouble(m_drivebase->GetRoll().value());
        dt->m_pitch_widget->GetEntry()->SetDouble(m_drivebase->GetPitch().value());
        dt->m_cone_widget->GetEntry()->SetBoolean(game_piece == Manipulator::GamePiece::Cone);
        dt->m_cube_widget->GetEntry()->SetBoolean(game_piece == Manipulator::GamePiece::Cube);




        dt->m_drivebase_ok_widget->GetEntry()->SetBoolean(m_drivebase->GetDrivebaseOK());
        dt->m_vision_ok_widget->GetEntry()->SetBoolean(m_drivebase->GetVision() != nullptr);
        dt->m_manipulator_ok_widget->GetEntry()->SetBoolean(m_manipulator != nullptr);
        dt->m_test_widget->GetEntry()->SetBoolean(m_drivebase->GetController()->GetLeftBumperButton());
    }
}

double SwerveDrivebaseShuffleboard::GetSlowdownFactor() {
    // If the 'Drive' widgets have been initialised, get the slowdown factor from the shuffleboard,
    // otherwise use the default
    if (m_drive_tab_widgets != nullptr) {
        DriveTabWidgets* dt = m_drive_tab_widgets;

        return dt->m_slowdown_widget->GetEntry()->GetDouble(RC::kDefaultDrivebaseSlowDownFactor);
    } else {
        return RC::kDefaultDrivebaseSlowDownFactor;
    }
}

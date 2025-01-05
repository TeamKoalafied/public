//==============================================================================
// SwerveDrivebaseShuffleboard.cpp
//==============================================================================

#include "SwerveDrivebaseShuffleboard.h"

#include "SwerveDrivebase.h"
#include "Manipulator.h"
#include "Vision.h"

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardContainer.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <frc/kinematics/SwerveDriveOdometry.h>


SwerveDrivebaseShuffleboard::SwerveDrivebaseShuffleboard(const SwerveDrivebase* drivebase, const Manipulator* manipulator) {
    m_drivebase = drivebase;
    m_manipulator = manipulator;
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
    db->m_robot_x_widget = &shuffleboard_tab.Add("Robot X", 0.0).WithPosition(22, 7).WithSize(3,2);
    db->m_robot_y_widget = &shuffleboard_tab.Add("Robot y", 0.0).WithPosition(25, 7).WithSize(3,2);
    db->m_robot_rotation_widget = &shuffleboard_tab.Add("Rotation", 0.0).WithPosition(28, 7).WithSize(3,2);
    
    // Create a field widget showing the current robot pose
    shuffleboard_tab.Add("Field", db->m_field_widget)
         .WithWidget(frc::BuiltInWidgets::kField)
         .WithPosition(22, 0)
         .WithSize(10,7);

    db->m_toe_in_widget = &shuffleboard_tab.Add("Toe In", 0.0).WithPosition(28, 9).WithSize(3,2);
}

void SwerveDrivebaseShuffleboard::VisionTabSetup() {
    // Create a 'Vision' shuffleboard tab and the object to store its widgets
    frc::ShuffleboardTab& vision_tab = frc::Shuffleboard::GetTab("Vision");
    m_vision_widgets = new VisionTabWidgets;
    VisionTabWidgets* vw = m_vision_widgets;

    //    0       3       6       9       12      15      18      21
    // 0  -----------------------------------------------------------
    //    | Est X | Odo X | Pho X | Sp Am |                         |
    // 2  ---------------------------------                         |
    //    | Est Y | Odo Y | Pho Y | Sp An |                         |
    // 4  ---------------------------------          Field          |
    //    | Est H | Odo H | Pho H | Sp Di |                         |
    // 6  ---------------------------------                         |
    //    | Has T | Has P | T AtX |       |                         |
    // 8  -----------------------------------------------------------

    // Create the widgets for pose estimation: x, y position and from odometry, vision and pose estimation
    vw->m_estimator_x = &vision_tab.Add("Estimator X", 0.0).WithPosition(0,0).WithSize(3,2);
    vw->m_estimator_y = &vision_tab.Add("Estimator Y", 0.0).WithPosition(0,2).WithSize(3,2);
    vw->m_estimator_heading = &vision_tab.Add("Estimator heading", 0.0).WithPosition(0,4).WithSize(3,2);
    vw->m_odometry_x = &vision_tab.Add("Odometry X", 0.0).WithPosition(3,0).WithSize(3,2);
    vw->m_odometry_y = &vision_tab.Add("Odometry Y", 0.0).WithPosition(3,2).WithSize(3,2);
    vw->m_odometry_heading = &vision_tab.Add("Odometry heading", 0.0).WithPosition(3,4).WithSize(3,2);
    vw->m_photon_x = &vision_tab.Add("Photon X", 0.0).WithPosition(6,0).WithSize(3,2);
    vw->m_photon_y = &vision_tab.Add("Photon Y", 0.0).WithPosition(6,2).WithSize(3,2);
    vw->m_photon_heading = &vision_tab.Add("Photon heading", 0.0).WithPosition(6,4).WithSize(3,2);

    // Create field widget for the vision tab
    vw->m_field_widget = new frc::Field2d;
    vision_tab.Add("Field", *vw->m_field_widget)
         .WithWidget(frc::BuiltInWidgets::kField)
         .WithPosition(12, 0)
         .WithSize(10,8);

    // Create boolean boxes for vision targeting and pose estimation
    vw->m_targets = &vision_tab.Add("Has targets", false)
                                .WithPosition(0, 6)
                                .WithSize(3,2)
                                .WithWidget(frc::BuiltInWidgets::kBooleanBox);
    vw->m_has_pose = &vision_tab.Add("Has pose", false)
                                .WithPosition(3, 6)
                                .WithSize(3,2)
                                .WithWidget(frc::BuiltInWidgets::kBooleanBox);
    vw->m_target_area = &vision_tab.Add("Target Area", 0.0).WithPosition(6, 6).WithSize(3,2);

    // Speaker targetting
    vw->m_speaker_pose_ambiguity = &vision_tab.Add("Speaker Ambig", 0.0).WithPosition(9,0).WithSize(3,2);
    vw->m_speaker_angle = &vision_tab.Add("Speaker Angle", 0.0).WithPosition(9,2).WithSize(3,2);
    vw->m_speaker_distance = &vision_tab.Add("Speaker Dist [m]", 0.0).WithPosition(9,4).WithSize(3,2);
    vw->m_x_dev_widget = &vision_tab.Add("Estimator X std dev [m]", Vision::kStdDevX).WithPosition(0,8).WithSize(3,2);
    vw->m_y_dev_widget = &vision_tab.Add("Estimator Y std dev [m]", Vision::kStdDevY).WithPosition(3,8).WithSize(3,2);
    vw->m_theta_dev_widget = &vision_tab.Add("Estimator Theta std dev [m]", Vision::kStdDevAngle).WithPosition(6,8).WithSize(3,2);
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
        frc::SwerveDriveOdometry<4>* odometry = m_drivebase->GetOdometry();
        VisionTabWidgets* vw = m_vision_widgets;

        vw->m_estimator_x->GetEntry()->SetDouble(swerve_pose.X().value());
        vw->m_estimator_y->GetEntry()->SetDouble(swerve_pose.Y().value());
        vw->m_estimator_heading->GetEntry()->SetDouble(swerve_pose.Rotation().Degrees().value());

        vw->m_odometry_x->GetEntry()->SetDouble(odometry->GetPose().X().value());
        vw->m_odometry_y->GetEntry()->SetDouble(odometry->GetPose().Y().value());
        vw->m_odometry_heading->GetEntry()->SetDouble(odometry->GetPose().Rotation().Degrees().value());

        vw->m_field_widget->SetRobotPose(swerve_pose);

        vw->m_has_pose->GetEntry()->SetBoolean(m_drivebase->GetVision().GetUpdatingPose());
        bool has_pose = m_drivebase->GetVision().GetLatestPhotonPose().has_value();
        vw->m_targets->GetEntry()->SetBoolean(has_pose);
        if (has_pose) {
            frc::Pose2d vision_pose = m_drivebase->GetVision().GetLatestPhotonPose().value().estimatedPose.ToPose2d();
            vw->m_photon_x->GetEntry()->SetDouble(vision_pose.X().value());
            vw->m_photon_y->GetEntry()->SetDouble(vision_pose.Y().value());
            vw->m_photon_heading->GetEntry()->SetDouble(vision_pose.Rotation().Degrees().value());
            vw->m_target_area->GetEntry()->SetDouble(m_drivebase->GetVision().GetLatestPhotonResult().GetBestTarget().GetArea());
        } else {
            vw->m_photon_x->GetEntry()->SetDouble(0.0);
            vw->m_photon_y->GetEntry()->SetDouble(0.0);
            vw->m_photon_heading->GetEntry()->SetDouble(0.0);
            vw->m_target_area->GetEntry()->SetDouble(0.0);
        }

        // Speaker targetting
        double ambiguity;
        units::degree_t angle;
        units::meter_t distance;
        m_drivebase->GetVision().GetSpeakerTargetInfo(ambiguity, angle, distance);
        vw->m_speaker_pose_ambiguity->GetEntry()->SetDouble(ambiguity);
        vw->m_speaker_angle->GetEntry()->SetDouble(angle.value());
        vw->m_speaker_distance->GetEntry()->SetDouble(distance.value());
    }
}

wpi::array<double, 3> SwerveDrivebaseShuffleboard::GetStdDevs() {
    wpi::array<double, 3> std_devs = {0.0,0.0,0.0};
    VisionTabWidgets* vw = m_vision_widgets;
    std_devs[0] = vw->m_x_dev_widget->GetEntry()->GetDouble(Vision::kStdDevX);
    std_devs[1] = vw->m_y_dev_widget->GetEntry()->GetDouble(Vision::kStdDevY);
    std_devs[2] = vw->m_theta_dev_widget->GetEntry()->GetDouble(Vision::kStdDevAngle);
    return std_devs;
}

units::degree_t SwerveDrivebaseShuffleboard::GetToeInAngle() {
    return (units::degree_t)m_drivebase_widgets->m_toe_in_widget->GetEntry()->GetDouble(0.0);
}

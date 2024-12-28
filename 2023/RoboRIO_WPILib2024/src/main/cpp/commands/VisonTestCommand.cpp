//==============================================================================
// VisionTestCommand.cpp
//==============================================================================

#include "VisionTestCommand.h"

#include "../subsystems/Manipulator.h"
#include "../subsystems/SwerveDrivebase.h"
#include "../util/KoalafiedUtilities.h"
#include "LogVisionCommand.h"

#include "LogDrivebaseCommand.h"
#include "LogModulesCommand.h"
#include "PathPlanner.h"
#include "TestTurnCommand.h"

#include <frc/Filesystem.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardContainer.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Commands.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/path/PathPlannerPath.h>

#include <filesystem>
#include <iostream>

using namespace pathplanner;


namespace KU = KoalafiedUtilities;

namespace {

}


const VisionTestCommand::ChooserValue<VisionTestCommand::PathType> VisionTestCommand::kPathTypeMapping[] = {
    { VisionTestCommand::PathType::Vertical,   "Vertical"      },
    { VisionTestCommand::PathType::Horizontal, "Horizontal"   },
};
const int VisionTestCommand::kPathTypeMappingCount = sizeof(VisionTestCommand::kPathTypeMapping) / sizeof(VisionTestCommand::kPathTypeMapping[0]);

//==============================================================================
// Construction

VisionTestCommand::VisionTestCommand() {
    m_shuffleboard_widgets = nullptr;
}

//==============================================================================
// Virtual Functions from IAutonomousProvider

const char* VisionTestCommand::Name() {
    return "Vision Test";
}

int VisionTestCommand::SetupDashboard(frc::ShuffleboardTab& auto_tab, int x_pos, int y_pos) {
    // Create a enw tab to put the controls on
    frc::ShuffleboardTab& vison_test_tab = frc::Shuffleboard::GetTab("Vision Test");
    m_shuffleboard_widgets = new ShuffleboardWidgets;
    ShuffleboardWidgets* sw = m_shuffleboard_widgets;

    // Controller Layout
    //
    //      0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16
    //   0  |--|--|--|--|--|--|--|--|--|--|--|--|
    //   1  |  Path Type      |                 |--|--|--|--|
    //   2  |                 |                 |Direction P|
    //   3  |--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|
    //   4  |Path Length|Max Speed  |           |Direction D|
    //   5  |--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|
    //   6  |Path Width |Max Accel. |           |Rotation P |
    //   7  |--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|
    //   8  |Path Rot.  |           |           |Rotation D |
    //   9  |--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|
    //

    // Setup a drop down for choosing the type of path to run
    for (int i = 0; i < kPathTypeMappingCount; i++) {
        m_shuffleboard_widgets->m_path_chooser.AddOption(kPathTypeMapping[i].label, kPathTypeMapping[i].value);
    }

    m_shuffleboard_widgets->m_path_chooser.SetDefaultOption(kPathTypeMapping[0].label, kPathTypeMapping[0].value);
    vison_test_tab.Add("Path Type", m_shuffleboard_widgets->m_path_chooser).WithPosition(0, 0).WithSize(6, 3);

    // Add the widgets for the test parameters with reasonable starting default values
    sw->m_length = &vison_test_tab.Add("Path Length (m)", 3.0).WithPosition(0, 3).WithSize(4, 2);
    sw->m_width = &vison_test_tab.Add("Path Width (m)", 2.0).WithPosition(0,5).WithSize(4, 2);
    sw->m_rotatation = &vison_test_tab.Add("Path Rotation (deg)", 0.0).WithPosition(0,7).WithSize(4, 2);

    sw->m_max_speed = &vison_test_tab.Add("Max Speed (mps)", 1.0).WithPosition(4,3).WithSize(4, 2);
    sw->m_max_acceleration = &vison_test_tab.Add("Max Accleeration (mps2)", 1.0).WithPosition(4, 5).WithSize(4, 2);
    sw->m_repeat_count = &vison_test_tab.Add("Repeat Count", 1.0).WithPosition(4, 7).WithSize(4, 2);

    sw->m_direction_p = &vison_test_tab.Add("Direction P", 2.0).WithPosition(12, 1).WithSize(4, 2);
    sw->m_direction_d = &vison_test_tab.Add("Direction D", 0.0).WithPosition(12, 3).WithSize(4, 2);
    sw->m_rotation_p = &vison_test_tab.Add("Rotation P", 2.0).WithPosition(12, 5).WithSize(4, 2);
    sw->m_rotation_d = &vison_test_tab.Add("Rotation D", 0.0).WithPosition(12, 7).WithSize(4, 2);

    // Return we have used no space as we put all the controls on a separate tab
    return 0;
}

frc2::CommandPtr VisionTestCommand::CreateAutonomousCommand(SwerveDrivebase& drivebase, Manipulator& manipulator) {
    ShuffleboardWidgets* sw = m_shuffleboard_widgets;

    // Extract path parameters from the dashboard
    units::meter_t length = 
        (units::meter_t)KU::Clamp(sw->m_length->GetEntry()->GetDouble(3.0), 0.5, 5.0);
    units::meter_t width = 
        (units::meter_t)KU::Clamp(sw->m_width->GetEntry()->GetDouble(2.0), -5.0, 5.0);
    units::degree_t rotatation = 
        (units::degree_t)KU::Clamp(sw->m_rotatation->GetEntry()->GetDouble(0.0), -180.0, 180.0);


    // Extract trajectory generation parameters from the dashboard
    units::meters_per_second_t max_velocity =
        (units::meters_per_second_t)KU::Clamp(sw->m_max_speed->GetEntry()->GetDouble(1.0), 0.5, 3.0);
    units::meters_per_second_squared_t max_acceleration =
        (units::meters_per_second_squared_t)KU::Clamp(sw->m_max_acceleration->GetEntry()->GetDouble(0.5), 0.25, 3.0);
    int repeat_count = KU::Clamp<int>((int)sw->m_repeat_count->GetEntry()->GetDouble(1), 1, 10);

    PathType path_type = m_shuffleboard_widgets->m_path_chooser.GetSelected();


    return CreatePathPlannerCommand(drivebase, path_type, length, width, rotatation, max_velocity, max_acceleration, repeat_count);
            

    // Return a command that does nothing
    return frc2::CommandPtr(frc2::InstantCommand([] {  }, {  }));
}

frc2::CommandPtr VisionTestCommand::CreatePathPlannerCommand(SwerveDrivebase& drivebase, PathType path_type, units::meter_t length,
                                                                   units::meter_t width, units::degree_t rotation,
                                                                   units::meters_per_second_t max_velocity,
                                                                   units::meters_per_second_squared_t max_acceleration,
                                                                   int repeat_count) {

    ShuffleboardWidgets* sw = m_shuffleboard_widgets;
    LogVisionCommand::Parameters parameters;
    parameters.m_max_velocity = max_velocity;
    parameters.m_max_acceleration = max_acceleration;
    switch(path_type) {
        case PathType::Vertical: parameters.m_path_type = "Vertical"; break;
        case PathType::Horizontal: parameters.m_path_type = "Horizontal"; break;
    } 

    PathPlanner::SetupSwerveAuto(drivebase, nullptr, 
                                    KU::Clamp(sw->m_direction_p->GetEntry()->GetDouble(2.0), 0.0, 5.0), 
                                    KU::Clamp(sw->m_direction_d->GetEntry()->GetDouble(0.0), 0.0, 5.0), 
                                    KU::Clamp(sw->m_rotation_p->GetEntry()->GetDouble(2.0), 0.0, 5.0), 
                                    KU::Clamp(sw->m_rotation_d->GetEntry()->GetDouble(0.0), 0.0, 5.0));

    std::vector<std::shared_ptr<PathPlannerPath>> path_segments;
    PathConstraints constraints(max_velocity, max_acceleration, 360_deg_per_s, 720_deg_per_s_sq);
    std::string auto_file_path;
    frc::Pose2d start;
    frc::Pose2d finish;
    switch (path_type) {
        case PathType::Vertical: {
            // Create a vector of bezier points from poses. Each pose represents one waypoint.
            // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.

            //                        Length
            //              <------------------------->
            //
            //          ^   S=========================F
            //   width  |    Start                     Finish
            //          v   o
            //               Origin


            std::vector<frc::Pose2d> poses{
                frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
                frc::Pose2d(0_m, width+0.01_m, frc::Rotation2d(0_deg)),
                frc::Pose2d(length, width+0.01_m, frc::Rotation2d(0_deg)),
                frc::Pose2d(0_m, width+0.01_m, frc::Rotation2d(0_deg)),
                frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
            };
            
            path_segments.push_back(GeneratePathSegment(constraints, rotation, std::span<frc::Pose2d>(poses).subspan(0,2)));
            path_segments.push_back(GeneratePathSegment(constraints, rotation, std::span<frc::Pose2d>(poses).subspan(1,3)));
            path_segments.push_back(GeneratePathSegment(constraints, rotation, std::span<frc::Pose2d>(poses).subspan(3,2)));


            start = frc::Pose2d(0_m, width, frc::Rotation2d(0_deg));
            finish = frc::Pose2d(length, width, frc::Rotation2d(0_deg));


            break;
        }
        case PathType::Horizontal: {
            std::vector<frc::Pose2d> poses{
                frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
                frc::Pose2d(length+0.01_m, 0_m, frc::Rotation2d(0_deg)),
                frc::Pose2d(length+0.01_m, -width, frc::Rotation2d(0_deg)),
                frc::Pose2d(length+0.01_m, width, frc::Rotation2d(0_deg)),
                frc::Pose2d(length+0.01_m, 0_m, frc::Rotation2d(0_deg)),
                frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
            };
            path_segments.push_back(GeneratePathSegment(constraints, rotation, std::span<frc::Pose2d>(poses).subspan(0,2)));
            path_segments.push_back(GeneratePathSegment(constraints, rotation, std::span<frc::Pose2d>(poses).subspan(1,3)));
            path_segments.push_back(GeneratePathSegment(constraints, rotation, std::span<frc::Pose2d>(poses).subspan(3,2)));

            break;
        }
    }

    // NOTE: The initial pose is not just 'getInitialState().pose'. We must form from the pose from
    //       translation and the holonomic rotation, otherwise we start our robot facing along the path
    //       rather than in the direction we want.
    // frc::Pose2d initial_pose(trajectory.getInitialState().pose.Translation(),
    //                          trajectory.getInitialState().holonomicRotation);// No direct replacement for initial holonomicRotation
    if (path_segments[0]) {
        // First reset the pose of the robot to the start of the trajectory, then following the trajectory while
        // logging at the same time. Keep logging for 1 second after the end of the path so we see whether the
        // robot stops properly.
        frc::Pose2d origin;
        return  frc2::cmd::Sequence(
                frc2::CommandPtr(frc2::InstantCommand([&drivebase, origin] { drivebase.ResetPose(origin); }, { &drivebase })),
                frc2::CommandPtr(AutoBuilder::followPath(GeneratePathSegment(constraints, origin, start))),
                frc2::cmd::Deadline(
                    AutoBuilder::followPath(GeneratePathSegment(constraints, start, finish)),
                    frc2::CommandPtr(std::unique_ptr<frc2::Command>(new LogVisionCommand(&drivebase, nullptr, path_segments[0], parameters)))),
                frc2::CommandPtr(AutoBuilder::followPath(GeneratePathSegment(constraints, finish, origin))),
                frc2::cmd::Wait(1_s),
                frc2::CommandPtr(frc2::InstantCommand([this, &drivebase] {PrintFinalPosition(drivebase);}, {}))
            );
        // frc::Pose2d initial_pose(path_segments[0].get()->getPreviewStartingHolonomicPose()); 
        // return  frc2::cmd::Sequence(
        //         frc2::CommandPtr(frc2::InstantCommand([&drivebase, initial_pose] { drivebase.ResetPose(initial_pose); }, { &drivebase })),
        //         frc2::CommandPtr(AutoBuilder::followPath(path_segments[0])),
        //         frc2::cmd::Deadline(
        //             AutoBuilder::followPath(path_segments[1]),
        //             frc2::CommandPtr(std::unique_ptr<frc2::Command>(new LogVisionCommand(&drivebase, nullptr, path_segments[0], parameters)))),
        //         frc2::CommandPtr(AutoBuilder::followPath(path_segments[2])),
        //         frc2::cmd::Wait(1_s),
        //         frc2::CommandPtr(frc2::InstantCommand([this, &drivebase] {PrintFinalPosition(drivebase);}, {}))
        //     );
    } else {
        return frc2::CommandPtr(frc2::InstantCommand([] {  }, {  }));

    }
}

std::shared_ptr<PathPlannerPath> VisionTestCommand::GeneratePathSegment(PathConstraints constraints, 
                                                                        units::degree_t rotation, 
                                                                        std::span<frc::Pose2d> poses) {
    std::shared_ptr<PathPlannerPath> path;
    std::vector<frc::Pose2d> pose_vector;
    pose_vector.assign(poses.begin(), poses.end());
    std::vector<frc::Translation2d> bezierPoints = PathPlannerPath::bezierFromPoses(pose_vector);
    // Create the path using the bezier points created above
    // We make a shared pointer here since the path following commands require a shared pointer
    path = std::make_shared<PathPlannerPath>(
        bezierPoints,
        constraints, // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
        GoalEndState(0.0_mps, frc::Rotation2d(rotation)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
    );

    // Prevent the path from being flipped as the coordinates are already correct
    path->preventFlipping = true;
    return path;
}

std::shared_ptr<PathPlannerPath> VisionTestCommand::GeneratePathSegment(PathConstraints constraints, 
                                                                        frc::Pose2d start, frc::Pose2d finish) {
    std::shared_ptr<PathPlannerPath> path;
    std::vector<frc::Pose2d> pose_vector {start, finish};
    std::vector<frc::Translation2d> bezierPoints = PathPlannerPath::bezierFromPoses(pose_vector);
    // Create the path using the bezier points created above
    // We make a shared pointer here since the path following commands require a shared pointer
    path = std::make_shared<PathPlannerPath>(
        bezierPoints,
        constraints, // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
        GoalEndState(0.0_mps, frc::Rotation2d()) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
    );

    // Prevent the path from being flipped as the coordinates are already correct
    path->preventFlipping = true;
    return path;
}

void VisionTestCommand::PrintFinalPosition(SwerveDrivebase& drivebase) {
    frc::Pose2d odo_pose = drivebase.GetOdometry()->GetPose();
    frc::Pose2d vision_pose = drivebase.GetVision().GetPose();
    std::cout << "Odometry:  X: " << odo_pose.X().value() << " Y: " << odo_pose.Y().value() << " Rot.: " << odo_pose.Rotation().Degrees().value() << "\n";
    std::cout << "Vision:  X: " << vision_pose.X().value() << " Y: " << vision_pose.Y().value() << " Rot.: " << vision_pose.Rotation().Degrees().value() << "\n";
}
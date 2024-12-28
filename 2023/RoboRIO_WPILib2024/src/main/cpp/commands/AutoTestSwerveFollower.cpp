//==============================================================================
// AutoTestSwerveFollower.cpp
//==============================================================================

#include "AutoTestSwerveFollower.h"

#include "AutoBalanceCommand.h"
#include "LogDrivebaseCommand.h"
#include "LogModulesCommand.h"
#include "LogPathPlannerCommand.h"
#include "TestTurnCommand.h"

#include "../pathfollower/PathBuilder.h"
#include "../pathfollower/RobotPath.h"
#include "../pathfollower/SwerveTrajectoryGenerator.h"
#include "../subsystems/Manipulator.h"
#include "../subsystems/SwerveDrivebase.h"
#include "../util/KoalafiedUtilities.h"

#include <frc/Filesystem.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardContainer.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Commands.h>

#include <pathplanner/lib/path/PathPlannerPath.h>
//#include <pathplanner/lib/auto/SwerveAutoBuilder.h>
#include <pathplanner/lib/auto/AutoBuilder.h>

#include <filesystem>
#include <iostream>

using namespace pathplanner;


namespace KU = KoalafiedUtilities;

namespace {

}

//==============================================================================
// Static Variables

const AutoTestSwerveFollower::ChooserValue<AutoTestSwerveFollower::PathType> AutoTestSwerveFollower::kPathTypeMapping[] = {
    { AutoTestSwerveFollower::PathType::Forward, "Forward"   },
    { AutoTestSwerveFollower::PathType::Turn,    "Turn"      },
    { AutoTestSwerveFollower::PathType::Slalom,  "Slalom"    },
    { AutoTestSwerveFollower::PathType::TestTurn,"Test Turn" },
};
const int AutoTestSwerveFollower::kPathTypeMappingCount = sizeof(AutoTestSwerveFollower::kPathTypeMapping) / sizeof(AutoTestSwerveFollower::kPathTypeMapping[0]);

const AutoTestSwerveFollower::ChooserValue<AutoTestSwerveFollower::FollowerType> AutoTestSwerveFollower::kFollowerTypeMapping[] = {
    { AutoTestSwerveFollower::FollowerType::PathPlanner,    "PathPlanner"    },
    { AutoTestSwerveFollower::FollowerType::WPILib,         "WPILib"         },
    { AutoTestSwerveFollower::FollowerType::SwerveFollower, "SwerveFollower" },
};
const int AutoTestSwerveFollower::kFollowerTypeMappingCount = sizeof(AutoTestSwerveFollower::kFollowerTypeMapping) / sizeof(AutoTestSwerveFollower::kFollowerTypeMapping[0]);

//==============================================================================
// Construction

AutoTestSwerveFollower::AutoTestSwerveFollower() {
    m_shuffleboard_widgets = nullptr;
}

//==============================================================================
// Virtual Functions from IAutonomousProvider

const char* AutoTestSwerveFollower::Name() {
    return "Auto Test";
}

int AutoTestSwerveFollower::SetupDashboard(frc::ShuffleboardTab& auto_tab, int x_pos, int y_pos) {
    // Create a enw tab to put the controls on
    frc::ShuffleboardTab& auto_test_tab = frc::Shuffleboard::GetTab("Auto Test");
    m_shuffleboard_widgets = new ShuffleboardWidgets;
    ShuffleboardWidgets* sw = m_shuffleboard_widgets;

    // Controller Layout
    //
    //      0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16
    //   0  |--|--|--|--|--|--|--|--|--|--|--|--|
    //   1  |  Path Type      | Follower Type   |--|--|--|--|
    //   2  |                 |                 |Direction P|
    //   3  |--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|
    //   4  |Path Length|Max Speed  |Look ahead |Direction D|
    //   5  |--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|
    //   6  |Path Width |Max Accel. |Look Fact. |Rotation P |
    //   7  |--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|
    //   8  |Path Rot.  |           |Corr.Blend |Rotation D |
    //   9  |--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|
    //

    // Setup a drop down for choosing the type of path to run
    for (int i = 0; i < kPathTypeMappingCount; i++) {
        m_shuffleboard_widgets->m_path_chooser.AddOption(kPathTypeMapping[i].label, kPathTypeMapping[i].value);
    }
    GetPathPlannerFilePaths(m_path_planner_names);
    for (int i = 0; i < (int)m_path_planner_names.size(); i++) {
        m_shuffleboard_widgets->m_path_chooser.AddOption(m_path_planner_names[i], (PathType)((int)PathType::FilePath + i));
    }
    m_shuffleboard_widgets->m_path_chooser.SetDefaultOption(kPathTypeMapping[0].label, kPathTypeMapping[0].value);
    auto_test_tab.Add("Path Type", m_shuffleboard_widgets->m_path_chooser).WithPosition(0, 0).WithSize(6, 3);

    // Setup a drop down for choosing the type of path to run
    for (int i = 0; i < kFollowerTypeMappingCount; i++) {
        m_shuffleboard_widgets->m_follower_chooser.AddOption(kFollowerTypeMapping[i].label, kFollowerTypeMapping[i].value);
    } 
    m_shuffleboard_widgets->m_follower_chooser.SetDefaultOption(kFollowerTypeMapping[0].label, kFollowerTypeMapping[0].value);
    auto_test_tab.Add("Follower Type", m_shuffleboard_widgets->m_follower_chooser).WithPosition(6, 0).WithSize(6, 3);


    // Add the widgets for the test parameters with reasonable starting default values
    sw->m_length = &auto_test_tab.Add("Path Length (m)", 3.0).WithPosition(0, 3).WithSize(4, 2);
    sw->m_width = &auto_test_tab.Add("Path Width (m)", 2.0).WithPosition(0,5).WithSize(4, 2);
    sw->m_rotatation = &auto_test_tab.Add("Path Rotation (deg)", 0.0).WithPosition(0,7).WithSize(4, 2);

    sw->m_max_speed        = &auto_test_tab.Add("Max Speed (mps)", 1.0).WithPosition(4,3).WithSize(4, 2);
    sw->m_max_acceleration = &auto_test_tab.Add("Max Accleeration (mps2)", 1.0).WithPosition(4, 5).WithSize(4, 2);
    sw->m_repeat_count     = &auto_test_tab.Add("Repeat Count", 1.0).WithPosition(4, 7).WithSize(4, 2);

    sw->m_lookahead_distance = &auto_test_tab.Add("Look ahead distance (m)", 0.5).WithPosition(8,3).WithSize(4, 2);
    sw->m_lookahead_factor = &auto_test_tab.Add("Lookahead factor", 1.5).WithPosition(8,5).WithSize(4, 2);
    sw->m_correction_blend = &auto_test_tab.Add("Correction blend", 0.5).WithPosition(8,7).WithSize(4, 2);

    sw->m_direction_p = &auto_test_tab.Add("Direction P", 2.0).WithPosition(12, 1).WithSize(4, 2);
    sw->m_direction_d = &auto_test_tab.Add("Direction D", 0.0).WithPosition(12, 3).WithSize(4, 2);
    sw->m_rotation_p = &auto_test_tab.Add("Rotation P", 2.0).WithPosition(12, 5).WithSize(4, 2);
    sw->m_rotation_d = &auto_test_tab.Add("Rotation D", 0.0).WithPosition(12, 7).WithSize(4, 2);

    // Return we have used no space as we put all the controls on a separate tab
    return 0;
}

frc2::CommandPtr AutoTestSwerveFollower::CreateAutonomousCommand(SwerveDrivebase& drivebase, Manipulator& manipulator) {
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

    FollowerType follower_type = m_shuffleboard_widgets->m_follower_chooser.GetSelected();
    switch (follower_type) {
        case FollowerType::WPILib:
            return CreateWPILibCommand(drivebase, path_type, length, width, rotatation, max_velocity, max_acceleration);
        case FollowerType::PathPlanner:
            return CreatePathPlannerCommand(drivebase, path_type, length, width, rotatation, max_velocity, max_acceleration, repeat_count);
        case FollowerType::SwerveFollower:
            return CreateSwerveFollowerCommand(drivebase, path_type, length, width, rotatation, max_velocity, max_acceleration);
    }

    // Return a command that does nothing
    return frc2::CommandPtr(frc2::InstantCommand([] {  }, {  }));
}

//==============================================================================
// Create WPILIb Commands

frc2::CommandPtr AutoTestSwerveFollower::CreateWPILibCommand(SwerveDrivebase& drivebase, PathType path_type, units::meter_t length,
                                                             units::meter_t width, units::degree_t rotatation,
                                                             units::meters_per_second_t max_velocity,
                                                             units::meters_per_second_squared_t max_acceleration) {
//     switch (path_type) {
//         default:
//         case PathType::Forward:
//             break;
// //            return drivebase.CreateSwerveTrajectoryCommand(CreateForwardSwerveTrajectory(parameters, length, 0_deg, rotatation));
//         case PathType::Slalom:
//             return drivebase.CreateSwerveTrajectoryCommand(CreateSlalomSwerveTrajectory(2_m, 1_m, 1_m));
//         case PathType::TestTurn:
//             return frc2::cmd::Deadline(
//                 frc2::CommandPtr(std::unique_ptr<frc2::Command>(new TestTurnCommand(&drivebase))),
//                 frc2::CommandPtr(std::unique_ptr<frc2::Command>(new LogModulesCommand(&drivebase))));
//     }

    // Return a command that does nothing
    return frc2::CommandPtr(frc2::InstantCommand([] {  }, {  }));
}


//==============================================================================
// Create PathPlanner Commands

frc2::CommandPtr AutoTestSwerveFollower::CreatePathPlannerCommand(SwerveDrivebase& drivebase, PathType path_type, units::meter_t length,
                                                                   units::meter_t width, units::degree_t rotatation,
                                                                   units::meters_per_second_t max_velocity,
                                                                   units::meters_per_second_squared_t max_acceleration,
                                                                   int repeat_count) {

    std::shared_ptr<PathPlannerPath> trajectory;
    switch (path_type) {
        case PathType::Forward:
            // trajectory = pathplanner::generatePath(
            //     PathConstraints(max_velocity, max_acceleration), 
            //     // position, heading(direction of travel), holonomic rotation
            //     PathPoint(frc::Translation2d(0_m, 0_m),    frc::Rotation2d(0_deg), frc::Rotation2d(0_deg)),
            //     PathPoint(frc::Translation2d(length, 0_m), frc::Rotation2d(0_deg), frc::Rotation2d(rotatation)) 
            // );
            // break;
            return frc2::CommandPtr(frc2::InstantCommand([] {  }, {  }));
        case PathType::Turn: {
            // frc::Rotation2d turn_rotation(width < 0_m ? -90_deg : 90_deg);
            // trajectory = pathplanner::generatePath(
            //     PathConstraints(max_velocity, max_acceleration), 
            //     // position, heading(direction of travel), holonomic rotation
            //     PathPoint(frc::Translation2d(0_m, 0_m), frc::Rotation2d(0_deg), frc::Rotation2d(0_deg)),
            //     PathPoint(frc::Translation2d(length, width), turn_rotation, frc::Rotation2d(rotatation))
            // );
            // break;
            return frc2::CommandPtr(frc2::InstantCommand([] {  }, {  }));
        }
        case PathType::Slalom:
            // trajectory = pathplanner::generatePath(
            //     PathConstraints(max_velocity, max_acceleration), 
            //     // position, heading(direction of travel), holonomic rotation
            //     PathPoint(frc::Translation2d(0_m, 0_m), frc::Rotation2d(0_deg), frc::Rotation2d(0_deg)),
            //     PathPoint(frc::Translation2d(length, width), frc::Rotation2d(0_deg), frc::Rotation2d(rotatation))
            // );
            // break;
            return frc2::CommandPtr(frc2::InstantCommand([] {  }, {  }));
        default:
            // Otherwise it must be one of the path files, so calculate the index and load it
            int file_path_index = (int)path_type - (int)PathType::FilePath;
            if (file_path_index < 0 || file_path_index >= (int)m_path_planner_names.size()) {
                std::cout << "ERROR: Illegal path file index\n";
                return frc2::CommandPtr(frc2::InstantCommand([] {  }, {  }));
            }
            std::cout << "Loading path file " << m_path_planner_names[file_path_index] << "\n";
            trajectory = PathPlannerPath::fromPathFile(m_path_planner_names[file_path_index]);
            break;
    }

    ShuffleboardWidgets* sw = m_shuffleboard_widgets;
    LogPathPlannerCommand::Parameters parameters;
    parameters.m_max_velocity = max_velocity;
    parameters.m_max_acceleration = max_acceleration;
    parameters.m_direction_p = KU::Clamp(sw->m_direction_p->GetEntry()->GetDouble(2.0), 0.0, 5.0);
    parameters.m_direction_d = KU::Clamp(sw->m_direction_d->GetEntry()->GetDouble(0.0), 0.0, 5.0);
    parameters.m_rotation_p = KU::Clamp(sw->m_rotation_p->GetEntry()->GetDouble(2.0), 0.0, 5.0);
    parameters.m_rotation_d = KU::Clamp(sw->m_rotation_d->GetEntry()->GetDouble(0.0), 0.0, 5.0);

    std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap;

    AutoBuilder::configureHolonomic(
        [&drivebase]() { return drivebase.GetPose(); }, // Function to supply current robot pose
        [&drivebase](auto initPose) { drivebase.ResetPose(initPose); }, // Function used to reset odometry at the beginning of auto
        [&drivebase] () {return drivebase.GetChassisSpeeds(); },
        [&drivebase](auto chassis_speeds) { drivebase.Drive(chassis_speeds, false); },
        HolonomicPathFollowerConfig(
            PIDConstants(parameters.m_direction_p, 0.0, parameters.m_direction_d), // PID constants to correct for translation error (used to create the X and Y PID controllers)
            PIDConstants(parameters.m_rotation_p, 0.0, parameters.m_rotation_d),   // PID constants to correct for rotation error (used to create the rotation controller)
            RC::kMaxSpeed,
            RC::kDriveBaseRadius,
            ReplanningConfig()),
        // Output function that accepts robot relative ChassisSpeeds
        [] () {return false; },
        { &drivebase } // Drive requirements, usually just a single drive subsystem
    );

    // NOTE: The initial pose is not just 'getInitialState().pose'. We must form from the pose from
    //       translation and the holonomic rotation, otherwise we start our robot facing along the path
    //       rather than in the direction we want.
    // frc::Pose2d initial_pose(trajectory.getInitialState().pose.Translation(),
    //                          trajectory.getInitialState().holonomicRotation);// No direct replacement for initial holonomicRotation
    
    frc::Pose2d initial_pose(trajectory.get()->getPreviewStartingHolonomicPose()); 
    if (repeat_count > 1) {
//    if (initial_pose == end_pose && repeat_count > 1) {
        // First reset the pose of the robot to the start of the trajectory, then following the trajectory while
        // logging at the same time. Keep logging for 1 second after the end of the path so we see whether the
        // robot stops properly.
    
        std::cout << "=================================================================\n";
        std::cout << "Repeat Count " << repeat_count << "\n";
        std::vector<frc2::CommandPtr> commands;
        commands.push_back(frc2::CommandPtr(frc2::InstantCommand([&drivebase, initial_pose] { drivebase.ResetPose(initial_pose); }, { &drivebase })));
        for (int i = 0; i < repeat_count; i++) {
            commands.push_back(frc2::CommandPtr(frc2::InstantCommand([&drivebase, i] {
                 std::cout << "Path Started: " << (i + 1) << " Pose ("
                           << drivebase.GetPose().X().value() << ", "
                           << drivebase.GetPose().Y().value() << ", "
                           << drivebase.GetPose().Rotation().Degrees().value() << ")\n";
                 }, { &drivebase })));
            commands.push_back(AutoBuilder::followPath(trajectory));
            commands.push_back(frc2::CommandPtr(frc2::InstantCommand([&drivebase, i] {
                 std::cout << "Path Finished: " << (i + 1) << " Pose ("
                           << drivebase.GetPose().X().value() << ", "
                           << drivebase.GetPose().Y().value() << ", "
                           << drivebase.GetPose().Rotation().Degrees().value() << ")\n";
                  }, { &drivebase })));
            commands.push_back(frc2::cmd::Wait(1_s));
        }
        return  frc2::cmd::Sequence(std::move(commands));
    } else {
        // First reset the pose of the robot to the start of the trajectory, then following the trajectory while
        // logging at the same time. Keep logging for 1 second after the end of the path so we see whether the
        // robot stops properly.
        return  frc2::cmd::Sequence(
                frc2::CommandPtr(frc2::InstantCommand([&drivebase, initial_pose] { drivebase.ResetPose(initial_pose); }, { &drivebase })),
                //frc2::cmd::Deadline(
                    frc2::cmd::Sequence(
                        AutoBuilder::followPath(trajectory),
                        frc2::cmd::Wait(1_s)
                    )
                    //frc2::CommandPtr(std::unique_ptr<frc2::Command>(new LogPathPlannerCommand(&drivebase, trajectory, parameters)))) 
            );
    }
}

void AutoTestSwerveFollower::GetPathPlannerFilePaths(std::vector<std::string>& path_names) {

    std::filesystem::path pathfinder_dir(frc::filesystem::GetDeployDirectory());
    pathfinder_dir /= "pathplanner";
 
    // Looping until all the items of the directory are
    // exhausted
    for (const auto& entry : std::filesystem::directory_iterator(pathfinder_dir)) {
        if (entry.path().extension() == ".path") {
            path_names.push_back(entry.path().stem().string());
        }
    }
}


//==============================================================================
// Creating Commands

frc2::CommandPtr AutoTestSwerveFollower::CreateSwerveFollowerCommand(SwerveDrivebase& drivebase, PathType path_type, units::meter_t length,
                                                                     units::meter_t width, units::degree_t rotatation,
                                                                     units::meters_per_second_t max_velocity,
                                                                     units::meters_per_second_squared_t max_acceleration) {


    // SwerveTrajectoryGenerator::Parameters parameters;
    // parameters.m_path_point_spacing = 0.1_m;
    // parameters.m_max_velocity =
    //     (units::meters_per_second_t)KU::Clamp(sw->m_max_speed->GetEntry()->GetDouble(1.0), 0.5, 3.0);
    // parameters.m_max_acceleration =
    // parameters.m_max_velocity_curve = 1.0_mps;

//     switch (path_type) {
//         default:
//         case PathType::Forward:
//             break;
// //            return drivebase.CreateSwerveTrajectoryCommand(CreateForwardSwerveTrajectory(parameters, length, 0_deg, rotatation));
//         case PathType::Slalom:
//             return drivebase.CreateSwerveTrajectoryCommand(CreateSlalomSwerveTrajectory(2_m, 1_m, 1_m));
//         case PathType::TestTurn:
//             return frc2::cmd::Deadline(
//                 frc2::CommandPtr(std::unique_ptr<frc2::Command>(new TestTurnCommand(&drivebase))),
//                 frc2::CommandPtr(std::unique_ptr<frc2::Command>(new LogModulesCommand(&drivebase))));
//     }

    // Return a command that does nothing
    return frc2::CommandPtr(frc2::InstantCommand([] {  }, {  }));
}

SwerveTrajectory AutoTestSwerveFollower::CreateForwardSwerveTrajectory(const SwerveTrajectoryGenerator::Parameters& parameters, 
    units::meter_t distance, units::degree_t start_rotation, units::degree_t end_rotation) {


    std::vector<Bezier3_2D<units::meter_t> > bezier_list {
        { { 0.0_m, 0.0_m }, { distance * 0.25, 0.0_m }, { distance * 0.75, 0.0_m }, { distance, 0.0_m }}
    };
    std::vector<units::degree_t> angles {
        {  start_rotation, end_rotation }
    };

    SwerveTrajectory trajectory;
    SwerveTrajectoryGenerator::Generate(trajectory, bezier_list, angles, parameters);

    return trajectory;
}

SwerveTrajectory AutoTestSwerveFollower::CreateSlalomSwerveTrajectory(units::meter_t distance, units::meter_t width, units::meter_t length) {
    SwerveTrajectoryGenerator::Parameters parameters;
    parameters.m_path_point_spacing = 0.1_m;
    parameters.m_max_velocity = 1.0_mps;
    parameters.m_max_acceleration = 1.0_mps_sq;
    parameters.m_max_velocity_curve = 1.0_mps;

    const double SLALOM_FACTOR = 0.7;
    RobotPath path;
    path.m_angles.push_back(0_deg);
    PathBuilder::AddStraight(path, 1_m, { 0_m, 0_m }, { 1_m, 0_m });
    path.m_angles.push_back(0_deg);
    PathBuilder::AddSlalomLeft(path, 1_m, 1_m, SLALOM_FACTOR);
    path.m_angles.push_back(0_deg);
    PathBuilder::AddSlalomRight(path, 2_m, 2_m, SLALOM_FACTOR);
    path.m_angles.push_back(0_deg);


    SwerveTrajectory trajectory;
    SwerveTrajectoryGenerator::Generate(trajectory, path.m_bezier_list, path.m_angles, parameters);
    return trajectory;

}


frc2::CommandPtr AutoTestSwerveFollower::CreateLogDrivebase(SwerveDrivebase& drivebase) {
    return frc2::CommandPtr(std::unique_ptr<frc2::Command>(new LogDrivebaseCommand(&drivebase)));
}

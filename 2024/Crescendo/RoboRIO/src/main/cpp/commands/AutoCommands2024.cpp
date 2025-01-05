//==============================================================================
// AutoCommands2024.cpp
//==============================================================================

#include "AutoCommands2024.h"

#include "LogPathPlannerCommand.h"
#include "PathPlanner.h"
#include "../subsystems/Manipulator.h"
#include "../subsystems/SwerveDrivebase.h"

#include <frc/DriverStation.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/Commands.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/path/PathPlannerPath.h>

#include <iostream>


using namespace pathplanner;

namespace {

// The strategy for autonomous mode
enum class Strategy {
    Auto1NoteBottomDelayLeave,
    Auto1NoteMiddleStay,
    Auto2NoteMiddle,
    Auto2NoteTopLeave,
    Auto2NoteBottomLeave,
    Auto3NoteBottomMiddle,
    Auto3NoteBottomMiddleLeave,
    Auto4NoteBottomMiddleTop,
    Auto4NoteBottomMiddleTopLeave,

//'1 Note Bottom Sub to Delay Leave.auto'  '2 Note Middle.auto'               '3 Note Bottom Middle.auto'
//'1 Note Middle Sub.auto'                 '2 Note Top.auto'                  '4 Note Leave.auto'
//'2 Note Bottom.auto'                     '3 Note Bottom Middle Leave.auto'  '4 Note.auto'
};

// Smart dashboard chooser for the autonomouse strategy
frc::SendableChooser<Strategy> ms_strategy_chooser;

// Build a single structure to keep all the mappings in one place to reduce risk of errors
struct AutoModeMapping {
    std::string label;
    Strategy strategy;
};

// Define the mapping between the labels on the dashboard and the strategy
const AutoModeMapping kAutoModes[] = {
    { "Auto1NoteBottomDelayLeave",     Strategy::Auto1NoteBottomDelayLeave     },
    { "Auto1NoteMiddleStay",           Strategy::Auto1NoteMiddleStay           },
    { "Auto2NoteMiddle",               Strategy::Auto2NoteMiddle               },
    { "Auto2NoteTopLeave",             Strategy::Auto2NoteTopLeave             },
    { "Auto2NoteBottomLeave",          Strategy::Auto2NoteBottomLeave          },
    { "Auto3NoteBottomMiddle",         Strategy::Auto3NoteBottomMiddle         },
    { "Auto3NoteBottomMiddleLeave",    Strategy::Auto3NoteBottomMiddleLeave    },
    { "Auto4NoteBottomMiddleTop",      Strategy::Auto4NoteBottomMiddleTop      },
    { "Auto4NoteBottomMiddleTopLeave", Strategy::Auto4NoteBottomMiddleTopLeave },
};


// Compute the number of entries so we can create the string array for populating the default dashboard
const int kAutoModeCount = sizeof(kAutoModes) / sizeof(AutoModeMapping);

}


//==============================================================================
// Construction

AutoCommands2024::AutoCommands2024() {

}

//==============================================================================
// Virtual Functions from IAutonomousProvider

const char* AutoCommands2024::Name() {
    return "Auto 2024";
}

int AutoCommands2024::SetupDashboard(frc::ShuffleboardTab& auto_tab, int x_pos, int y_pos) {
    // Setup the chooser for determining the strategy for the autonomous period
   	for (int i = 0; i < kAutoModeCount; i++) {
    	ms_strategy_chooser.AddOption(kAutoModes[i].label, kAutoModes[i].strategy);
    } 
    ms_strategy_chooser.SetDefaultOption(kAutoModes[0].label, kAutoModes[0].strategy);
    auto_tab.Add("Auto 2024", ms_strategy_chooser).WithPosition(x_pos, y_pos).WithSize(SHUFFLEBOARD_WIDTH, 3);


    return SHUFFLEBOARD_WIDTH;
}

frc2::CommandPtr AutoCommands2024::CreateAutonomousCommand(SwerveDrivebase& drivebase, Manipulator& manipulator) {

    Strategy strategy = ms_strategy_chooser.GetSelected();
    switch (strategy) {
        default:
        // DANGER! This is wrong! It needs to set the pose for the alliance colour
        //     return frc2::CommandPtr(frc2::InstantCommand([&drivebase] { drivebase.ResetPose(frc::Pose2d{0_m, 0_m, 180_deg}); }, { &drivebase }));
        case Strategy::Auto1NoteBottomDelayLeave: return CreateAutoNoteCommand(drivebase, manipulator, "1 Note Bottom Sub to Delay Leave");
        case Strategy::Auto1NoteMiddleStay: return CreateAutoNoteCommand(drivebase, manipulator, "1 Note Middle Sub");
        case Strategy::Auto2NoteMiddle: return CreateAutoNoteCommand(drivebase, manipulator, "2 Note Middle");
        case Strategy::Auto2NoteTopLeave: return CreateAutoNoteCommand(drivebase, manipulator, "2 Note Top Leave");
        case Strategy::Auto2NoteBottomLeave: return CreateAutoNoteCommand(drivebase, manipulator, "2 Note Bottom Leave");
        case Strategy::Auto3NoteBottomMiddle: return CreateAutoNoteCommand(drivebase, manipulator, "3 Note Bottom Middle");
        case Strategy::Auto3NoteBottomMiddleLeave: return CreateAutoNoteCommand(drivebase, manipulator, "3 Note Bottom Middle Leave");
        case Strategy::Auto4NoteBottomMiddleTop: return CreateAutoNoteCommand(drivebase, manipulator, "4 Note");
        case Strategy::Auto4NoteBottomMiddleTopLeave: return CreateAutoNoteCommand(drivebase, manipulator, "4 Note Leave");

        //'1 Note Bottom Sub to Delay Leave.auto'  '2 Note Middle.auto'               '3 Note Bottom Middle.auto'
        //'1 Note Middle Sub.auto'                 '2 Note Top.auto'                  '4 Note Leave.auto'
        //'2 Note Bottom.auto'                     '3 Note Bottom Middle Leave.auto'  '4 Note.auto'

        // DANGER! Paths do not set the initial pose and so mess up horribly. Needs to be flipped appropriately.
    }
    // Return a command that does nothing
    std::cout << "ERROR: Defaulting to do nothing auto command\n";
    return frc2::CommandPtr(frc2::InstantCommand([] {  }, {  }));
}


//==============================================================================
// Creating Commands


frc2::CommandPtr AutoCommands2024::CreateAutoNoteCommand(SwerveDrivebase& drivebase, Manipulator& manipulator, std::string auto_name) {

    std::vector<std::shared_ptr<PathPlannerPath>> auto_paths = PathPlannerAuto::getPathGroupFromAutoFile(auto_name);

    LogPathPlannerCommand::Parameters parameters;
    parameters.m_max_velocity = 1_mps;
    parameters.m_max_acceleration = 0.5_mps_sq;
    parameters.m_direction_p = PathPlanner::kDirectionP;
    parameters.m_direction_d = PathPlanner::kDirectionD;
    parameters.m_rotation_p = PathPlanner::kRotationP;
    parameters.m_rotation_d = PathPlanner::kRotationP;

    return 
        frc2::cmd::Sequence(
            //frc2::RunCommand([&manipulator] { manipulator.DoAutoShooting(); }, { &manipulator }).WithTimeout(2_s),
            //frc2::InstantCommand([&manipulator] { manipulator.StopAuto(); }, { &manipulator }).ToPtr(),
            frc2::cmd::Deadline(
                CreatePathPlannerAutoCommand(drivebase, auto_name),
                frc2::CommandPtr(std::unique_ptr<frc2::Command>(new LogPathPlannerCommand(&drivebase, &manipulator, auto_paths, parameters)))
            ),
            frc2::InstantCommand([&manipulator] {
                manipulator.StopPrefire();
                manipulator.StopAuto();
            }, { &manipulator }).ToPtr()
        );

}

frc2::CommandPtr AutoCommands2024::CreatePathPlannerAutoCommand(SwerveDrivebase& drivebase, std::string auto_name) {

    std::shared_ptr<PathPlannerPath> trajectory;
    std::cout << "Loading auto file " << auto_name << "\n";
    try {
        return PathPlannerAuto(auto_name).ToPtr();
    } catch (const std::exception& e) {
        std::cout << "Exception loading file " << e.what() << "\n";
        return frc2::CommandPtr(frc2::InstantCommand([] {  }, {  }));
    }
}

// frc2::CommandPtr AutoCommands2024::CreatePathNoteCommand(SwerveDrivebase& drivebase, Manipulator& manipulator,
//                                                          std::string path_name, units::second_t delay) {
//     units::meters_per_second_t max_velocity = 1_mps;
//     units::meters_per_second_squared_t max_acceleration = 0.5_mps_sq;
//     return 
//         frc2::cmd::Sequence(
//             frc2::RunCommand([&manipulator] { manipulator.DoAutoShooting(); }, { &manipulator }).WithTimeout(2_s),
//             frc2::cmd::Wait(delay),
//             //frc2::InstantCommand([&manipulator] { manipulator.StopAuto(); }, { &manipulator }).ToPtr(),
//             frc2::cmd::Deadline(
//                 CreatePathPlannerPathCommand(drivebase, path_name, delay, max_velocity, max_acceleration),
//                 frc2::RunCommand([&manipulator] { manipulator.DoAutoIntakeAndShoot(); }, { &manipulator }).ToPtr()
//             ),
//             frc2::RunCommand([&manipulator] { manipulator.DoAutoIntakeAndShoot(); }, { &manipulator }).WithTimeout(2_s),
//             frc2::InstantCommand([&manipulator] { manipulator.StopAuto(); }, { &manipulator }).ToPtr()
//         );
// }


// frc2::CommandPtr AutoCommands2024::CreatePathPlannerPathCommand(SwerveDrivebase& drivebase, std::string path_name,
//                                                                 units::second_t delay,
//                                                                 units::meters_per_second_t max_velocity,
//                                                                 units::meters_per_second_squared_t max_acceleration) {


                                                
                                                
//     std::shared_ptr<PathPlannerPath> trajectory;
//     std::cout << "Loading path file " << path_name << "\n";
//     try {
//         trajectory = PathPlannerPath::fromPathFile(path_name);
//     } catch (const std::exception& e) {
//         std::cout << "Exception loading file " << e.what() << "\n";
//         return frc2::CommandPtr(frc2::InstantCommand([] {  }, {  }));
//     }

//     // NOTE: The initial pose is not just 'getInitialState().pose'. We must form from the pose from
//     //       translation and the holonomic rotation, otherwise we start our robot facing along the path
//     //       rather than in the direction we want.
//     // frc::Pose2d initial_pose(trajectory.getInitialState().pose.Translation(),
//     //                          trajectory.getInitialState().holonomicRotation);// No direct replacement for initial holonomicRotation

   
//     frc::Pose2d initial_pose(trajectory.get()->getPreviewStartingHolonomicPose());
//     return  frc2::cmd::Sequence(
//             //frc2::CommandPtr(frc2::InstantCommand([&drivebase, initial_pose] { drivebase.ResetPose(initial_pose); }, { &drivebase })),
//             //frc2::cmd::Deadline(
//                 frc2::cmd::Sequence(
//                     AutoBuilder::followPath(trajectory),
//                     frc2::cmd::Wait(1_s)
//                 )
//                 //frc2::CommandPtr(std::unique_ptr<frc2::Command>(new LogPathPlannerCommand(&drivebase, trajectory, parameters)))) 
//         );
// }

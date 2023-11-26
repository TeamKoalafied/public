//==============================================================================
// AutoCommands2023.cpp
//==============================================================================

#include "AutoCommands2023.h"

#include "AutoBalanceCommand.h"
#include "../subsystems/Manipulator.h"
#include "../subsystems/SwerveDrivebase.h"

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/Commands.h>


namespace {

// The strategy for autonomous mode
enum class Strategy {
    Nothing,
    PlaceCone,
    PlaceConeBackAwayShort,
    PlaceConeBackAwayLong,
    Balance,
    PlaceConeBalance,
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
    { "Nothing",                Strategy::Nothing },
    { "Place Cone",             Strategy::PlaceCone },
    { "Place Cone Back Short",  Strategy::PlaceConeBackAwayShort },
    { "Place Cone Back Long",   Strategy::PlaceConeBackAwayLong },
    { "Balance",                Strategy::Balance },
    { "Place Cone Balance",     Strategy::PlaceConeBalance },
};

// Compute the number of entries so we can create the string array for populating the default dashboard
const int kAutoModeCount = sizeof(kAutoModes) / sizeof(AutoModeMapping);

}


//==============================================================================
// Construction

AutoCommands2023::AutoCommands2023() {

}

//==============================================================================
// Virtual Functions from IAutonomousProvider

const char* AutoCommands2023::Name() {
    return "Auto 2023";
}

int AutoCommands2023::SetupDashboard(frc::ShuffleboardTab& auto_tab, int x_pos, int y_pos) {
    // Setup the chooser for determining the strategy for the autonomous period
   	for (int i = 0; i < kAutoModeCount; i++) {
    	ms_strategy_chooser.AddOption(kAutoModes[i].label, kAutoModes[i].strategy);
    } 
    ms_strategy_chooser.SetDefaultOption(kAutoModes[0].label, kAutoModes[0].strategy);
    auto_tab.Add("Auto 2023", ms_strategy_chooser).WithPosition(x_pos, y_pos).WithSize(SHUFFLEBOARD_WIDTH, 3);

    AutoBalanceCommand::SetupDashboard();

    return SHUFFLEBOARD_WIDTH;
}

frc2::CommandPtr AutoCommands2023::CreateAutonomousCommand(SwerveDrivebase& drivebase, Manipulator& manipulator) {
    Strategy strategy = ms_strategy_chooser.GetSelected();
    switch (strategy) {
        default:
        case Strategy::Nothing:
            return frc2::CommandPtr(frc2::InstantCommand([&drivebase] { drivebase.ResetPose(frc::Pose2d{0_m, 0_m, 180_deg}); }, { &drivebase }));
        case Strategy::PlaceCone:
            return CreatePlaceConeCommand(drivebase, manipulator);
        case Strategy::PlaceConeBackAwayShort:
            // About 6'6" to the line so robot should back away 6'6"' - 3' + 4' = 7'6" = 90"
            // Add an extra 4' because the WPILib swerve follower does not work properly
            return CreatePlaceConeBackAwayCommand(drivebase, manipulator, 90_in);
        case Strategy::PlaceConeBackAwayLong:
            // About 12' to the line so robot should back away 12' - 3' + 4' = 13' = 156"
            // Add an extra 4' because the WPILib swerve follower does not work properly
            return CreatePlaceConeBackAwayCommand(drivebase, manipulator, 156_in);
        case Strategy::Balance:
            return CreateBalanceCommand(drivebase);
        case Strategy::PlaceConeBalance:
            return frc2::cmd::Sequence(
                    CreatePlaceConeCommand(drivebase, manipulator),
                    CreateBalanceCommand(drivebase)
                );
    }
}


//==============================================================================
// Creating Commands


frc2::CommandPtr AutoCommands2023::CreatePlaceConeCommand(SwerveDrivebase& drivebase, Manipulator& manipulator) {
    // Note that when going to a given position the GoToPosition() function needs to be called every update
    // period so that the movement completes and is accurate. The PID parameters and method change when it is
    // close to the goal.

    return frc2::cmd::Sequence(
        // Start going to the level 3 cone place and wait until it is there
        frc2::FunctionalCommand(
            [] {}, // Initialize()
            [&manipulator] { manipulator.GoToPosition(Manipulator::Position::Level3ConePlace); }, // Execute()
            [] (bool interrupted) {}, // End()
            [&manipulator] { return manipulator.IsAtPosition(Manipulator::Position::Level3ConePlace, true); }, // IsFinished()
            { &manipulator } // Requirements
        ),
        // Run the intake to drop the cone for half a second
        frc2::RunCommand([&manipulator] { manipulator.DropGamePiece(); }, { &manipulator }).WithTimeout(1_s),
        frc2::InstantCommand([&manipulator] { manipulator.StopIntake(); }, { &manipulator }),
        // Go to the stowed position and wait until it is there
        frc2::FunctionalCommand(
            [] {}, // Initialize()
            [&manipulator] { manipulator.GoToPosition(Manipulator::Position::Stowed); }, // Execute()
            [] (bool interrupted) {}, // End()
            [&manipulator] { return manipulator.IsAtPosition(Manipulator::Position::Stowed, true); }, // IsFinished()
            { &manipulator } // Requirements
        )
        );
}

frc2::CommandPtr AutoCommands2023::CreatePlaceConeBackAwayCommand(SwerveDrivebase& drivebase, Manipulator& manipulator, units::inch_t distance) {
    // Setup the config with velocity and acceleration. Drive the path in reverse!
    units::meters_per_second_t max_velocity = 1.0_mps;
	units::meters_per_second_squared_t max_acceleration = 0.5_mps_sq;
    frc::TrajectoryConfig config(max_velocity, max_acceleration);
    config.SetKinematics(drivebase.GetKinematics());
    config.SetReversed(true);

    frc::Trajectory back_away_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        // Start at the origin facing the -X direction
        frc::Pose2d{0_m, 0_m, 180_deg},
        {frc::Translation2d{distance / 2, 0_m}},
        frc::Pose2d{distance, 0_m, 180_deg},
        // Pass the config
        config);

    return frc2::cmd::Sequence(
        CreatePlaceConeCommand(drivebase, manipulator),
        drivebase.CreateTrajectoryCommand(back_away_trajectory)
        );
   
}

frc2::CommandPtr AutoCommands2023::CreateBalanceCommand(SwerveDrivebase& drivebase) {
    return frc2::CommandPtr(std::unique_ptr<frc2::CommandBase>(new AutoBalanceCommand(&drivebase)));
}

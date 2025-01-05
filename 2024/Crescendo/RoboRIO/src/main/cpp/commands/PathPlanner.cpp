//==============================================================================
// PathPlanner.cpp
//==============================================================================

#include "PathPlanner.h"

#include "../subsystems/Manipulator.h"
#include "../subsystems/SwerveDrivebase.h"

#include <frc/DriverStation.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/NamedCommands.h>

using namespace pathplanner;

void PathPlanner::SetupSwerveAuto(SwerveDrivebase& drivebase, Manipulator* manipulator) {
    SetupSwerveAuto(drivebase, manipulator, kDirectionP, kDirectionD, kRotationP, kRotationD);
}

void PathPlanner::SetupSwerveAuto(SwerveDrivebase& drivebase, Manipulator* manipulator, double direction_P, double direction_D,
                                  double rotation_P, double rotation_D) {
    AutoBuilder::configureHolonomic(
        [&drivebase]() { return drivebase.GetPose(); }, // Function to supply current robot pose
        [&drivebase](auto initPose) { drivebase.ResetPose(initPose); }, // Function used to reset odometry at the beginning of auto
        [&drivebase] () { return drivebase.GetChassisSpeeds(); },
        [&drivebase](auto chassis_speeds) { drivebase.Drive(chassis_speeds, false); },
        HolonomicPathFollowerConfig(
            PIDConstants(direction_P, 0.0, direction_D), // PID constants to correct for translation error (used to create the X and Y PID controllers)
            PIDConstants(rotation_P, 0.0, rotation_D),   // PID constants to correct for rotation error (used to create the rotation controller)
            RC::kMaxSpeed,
            RC::kDriveBaseRadius,
            ReplanningConfig()),
        []() {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            auto alliance = frc::DriverStation::GetAlliance();
            if (alliance) {
                return alliance.value() == frc::DriverStation::Alliance::kRed;
            }
            return false;
        },
        { &drivebase } // Drive requirements, usually just a single drive subsystem
    );

    if (manipulator == nullptr) return;
    

    NamedCommands::registerCommand("ShootStartNote", std::move(
        frc2::RunCommand([manipulator] {
            manipulator->DoCustomPrefire(2700_rpm, 63_deg);
            manipulator->DoAutoShooting();
        }, { manipulator }).WithTimeout(2_s)));

    // NamedCommands::registerCommand("Prefire", std::move(
    //     frc2::InstantCommand([manipulator] {
    //         manipulator->DoCustomPrefire(1500_rpm, 30_deg);
    //     }, { manipulator }).ToPtr()));

    NamedCommands::registerCommand("Intake", std::move(
        frc2::RunCommand([manipulator] {
            manipulator->DoAutoIntake();
            manipulator->DoCustomPrefire(2700_rpm, 54_deg);
        }, { manipulator }).ToPtr()));

    NamedCommands::registerCommand("Shoot", std::move(
        frc2::RunCommand([manipulator] {
            manipulator->DoAutoShooting();
        }, { manipulator }).WithTimeout(0.5_s)));

    // NamedCommands::registerCommand("Stop", std::move(
    //     frc2::InstantCommand([manipulator] {
    //         manipulator->StopPrefire();
    //         manipulator->StopAuto();
    //     }, { manipulator }).ToPtr()));


}

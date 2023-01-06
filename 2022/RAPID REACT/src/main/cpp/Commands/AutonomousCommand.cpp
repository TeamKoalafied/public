//==============================================================================
// AutonomousCommand.cpp
//==============================================================================

#include "AutonomousCommand.h"

#include "ChallengePaths.h"

#include "DrivePathFollower.h"
//#include "MechanismController2020.h"
//#include "PathFollower/PathPointsFollower.h"

#include "RobotPath/Bezier3.h"
#include "RobotPath/MotionProfile.h"
#include "RobotPath/Point2D.h"
#include "RobotPath/PathSegment.h"
#include "RobotPath/RobotPath.h"

//#include "PathFollower/PathfinderFollower.h"
//#include "PathFollower/PathPointsFollower.h"

//#include "../Subsystems/DriveBase.h"


#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <iostream>
#include <sstream>

//==============================================================================
// This is an anonymous namespace, which means that all the stuff in it is only
// available in this file
namespace {
// The strategy for autonomous mode
enum class Strategy {
    kNone,                  // No autonomous command - fallback in case of errors
    kShootAndMoveForeward,  // Shoot our initial 3 balls and move foreward off the tarmac
    kTwoBallAuto,           // Move forward to pick up a second ball and then shoot both balls
};

// Smart dashboard chooser for the autonomouse strategy
frc::SendableChooser<Strategy> ms_strategy_chooser;

// Build a single structure to keep all the mappings in one place to reduce risk of errors
struct AutoModeMapping {
    std::string label;
    Strategy strategy;
};

// Define the mapping between the labels on the dashboard and the position and strategy
const AutoModeMapping kAutoModes[] = {
        { "None",           Strategy::kNone                 },
        { "Shoot And Move", Strategy::kShootAndMoveForeward },
        { "2 Ball",         Strategy::kTwoBallAuto          },
};

// Compute the number of entries so we can create the string array for populating the default dashboard
const int kAutoModeCount = sizeof(kAutoModes) / sizeof(AutoModeMapping);

}


//==========================================================================
// Dashboard Setup

void AutonomousCommand::SetupAutonomousDashboard() {
    // Setup the chooser for determining the strategy for the autonomous period
    for (int i = 0; i < kAutoModeCount; i++) {
        ms_strategy_chooser.AddOption(kAutoModes[i].label, kAutoModes[i].strategy);
    } 
    ms_strategy_chooser.SetDefaultOption(kAutoModes[0].label, kAutoModes[0].strategy);
    frc::SmartDashboard::PutData("Autonomous Strategy", &ms_strategy_chooser);

    // Setup the inputs for initial delay and offset. Both default to 0.
    frc::SmartDashboard::PutNumber("Autonomous Delay Sec", 0.0);
}

void AutonomousCommand::UpdateDashboard() {
    // Get the strategy, delay and offset from the dashboard
    Strategy strategy = ms_strategy_chooser.GetSelected();
    double delay_s = frc::SmartDashboard::GetNumber("Autonomous Delay Sec", 0.0);

    // Format a description of the settings back to the dashboard. This allow us to check that
    // the dashboard is working and has go the right settings
    std::string strategy_label = "<Error";
    for (int i = 0; i < kAutoModeCount; i++) {
        if (kAutoModes[i].strategy == strategy) {
            strategy_label = kAutoModes[i].label;
        }
    }
	std::ostringstream strategy_description;
  	strategy_description << strategy_label << " Delay: " << delay_s;
    frc::SmartDashboard::PutString("Autonomous Setting", strategy_description.str());
}


frc::Command* AutonomousCommand::CreateAutonomousCommand() {

    // Get the strategy, delay and offset from the dashboard
    Strategy strategy = ms_strategy_chooser.GetSelected();
    double delay_s = frc::SmartDashboard::GetNumber("Autonomous Delay Sec", 0.0);

    // TODO Subtract the shooter wheel ramp up time from the delay

    // Create the appropriate path for the strategy
    RobotPath* robot_path = NULL;
    switch (strategy) {
        case Strategy::kNone:
            return nullptr;
        default:
        case Strategy::kShootAndMoveForeward:
            robot_path = CreateShootAndMoveForewardPath(delay_s);
            break;
        case Strategy::kTwoBallAuto:
            robot_path = CreateTwoBallPath(delay_s);
            break;
    }

    // Get the maximum velocity and acceleration from the dashboard
    double max_velocity = frc::SmartDashboard::GetNumber("AutoMaxV", 0.5);
    if (max_velocity < 0.1) max_velocity = 0.1;
    if (max_velocity > 3.0) max_velocity = 3.0;
    double max_acceleration = frc::SmartDashboard::GetNumber("AutoMaxA", 0.25);
    if (max_acceleration < 0.1) max_acceleration = 0.1;
    if (max_acceleration > 3.0) max_acceleration = 3.0;

    // Create the path follower and drive command from the path
    PathFollower* path_follower = DrivePathFollower::CreatePurePursuitFollower(robot_path, max_velocity, max_acceleration);
    return new DrivePathFollower(path_follower);
}

//==========================================================================
// Path Creation

RobotPath* AutonomousCommand::CreateShootAndMoveForewardPath(double delay_s) {
    // Create and name the robot path
    RobotPath* robot_path = new RobotPath();
    robot_path->m_name = "ShootAndMoveForewards";

    // Add the initial delay, if any, and shooting of the initial ball 
    AddDelaySegment(robot_path, delay_s);
//   AddCommandSegment(robot_path, "ShootManual6"); // Won't work as the turret need to rotate
    AddCommandSegment(robot_path, "Shoot");
//    AddDelaySegment(robot_path, 10.0);

    // Add a segment to move the robot forewards 8 feet. This is away from the target, which
    // is 10 feet from the start line.
    const double INCH = 0.0254;
    const double FOOT = 12*INCH;
    const double FORWARD_DISTANCE = 8*FOOT;
    Bezier3 path;
    path.m_point1.Set(0.0, 0.0);
    path.m_point2.Set(FORWARD_DISTANCE * 0.25, 0.0);
    path.m_point3.Set(FORWARD_DISTANCE * 0.75, 0.0);
    path.m_point4.Set(FORWARD_DISTANCE, 0.0);	
    PathSegment* path_segment = new PathSegment();
    path_segment->m_name = "Straight";
    path_segment->m_path_definition.push_back(path);
    path_segment->m_reverse = false;
    robot_path->m_path_segments.push_back(path_segment);

    return robot_path;
}

RobotPath* AutonomousCommand::CreateTwoBallPath(double delay_s) {
    // Create and name the robot path
    RobotPath* robot_path = new RobotPath();
    robot_path->m_name = "ShootAndMoveBackwards";

    // Add the initial delay, if any, and shooting of the initial 3 balls 
    AddDelaySegment(robot_path, delay_s);

    // Add a segment to move the robot forewards 8 feet. This is away from the target, which
    // is 10 feet from the start line.
    const double INCH = 0.0254;
    const double FOOT = 12*INCH;
    const double FORWARD_DISTANCE = 8*FOOT;
    Bezier3 path;
    path.m_point1.Set(0.0, 0.0);
    path.m_point2.Set(FORWARD_DISTANCE * 0.25, 0.0);
    path.m_point3.Set(FORWARD_DISTANCE * 0.75, 0.0);
    path.m_point4.Set(FORWARD_DISTANCE, 0.0);	
    PathSegment* path_segment = new PathSegment();
    path_segment->m_name = "Straight";
    path_segment->m_path_definition.push_back(path);
    path_segment->m_reverse = false;
    path_segment->m_mechanism_actions.push_back(MechanismAction("StartIntaking", 1.0));
    robot_path->m_path_segments.push_back(path_segment);

    // 2s delay for the intake to finish
    AddDelaySegment(robot_path, 2.0);

    // Shoot the balls from 14 feet
    AddCommandSegment(robot_path, "ShootManual14");

    AddDelaySegment(robot_path, 5.0);
    AddCommandSegment(robot_path, "StopShooter");

    return robot_path;
}


//==========================================================================
// Path Creation Helpers

void AutonomousCommand::AddDelaySegment(RobotPath* robot_path, double delay_s) {
    if (delay_s > 0.0) {
        PathSegment* delay_segment = new PathSegment();
        delay_segment->m_name = "Delay";
        delay_segment->m_reverse = false;
    	delay_segment->m_mechanism_actions.push_back(MechanismAction("None", delay_s));
        robot_path->m_path_segments.push_back(delay_segment);
    }
}

void AutonomousCommand::AddCommandSegment(RobotPath* robot_path, const char* command) {
    PathSegment* delay_segment = new PathSegment();
    delay_segment->m_name = command;
    delay_segment->m_reverse = false;
    delay_segment->m_mechanism_actions.push_back(MechanismAction(command, 0.0));
    robot_path->m_path_segments.push_back(delay_segment);
}

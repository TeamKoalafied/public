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
    kShoot,			        // Shoot our initial 3 balls and do nothing else
    kShootAndMoveBackward,  // Shoot our initial 3 balls and move backward out of the way (towards target)
    kShootAndMoveForeward,  // Shoot our initial 3 balls and move foreward out of the way (away from target)
    kShootAndTrench,        // Shoot our initial 3 balls then get 3 balls from the trench and shoot them
    kShootAndShield,        // Shoot our initial 3 balls then get 3 balls from under the shield generator and shoot them
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
        { "Shoot",                  Strategy::kShoot },
        { "Shoot Towards Target",   Strategy::kShootAndMoveBackward },
        { "Shoot Away From Target", Strategy::kShootAndMoveForeward },
        { "Shoot Trench",           Strategy::kShootAndTrench },
        { "Shoot Shield",           Strategy::kShootAndShield },
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
    frc::SmartDashboard::PutNumber("Autonomous Trench Offset Inch", 0.0);
}

void AutonomousCommand::UpdateDashboard() {
    // Get the strategy, delay and offset from the dashboard
    Strategy strategy = ms_strategy_chooser.GetSelected();
    double delay_s = frc::SmartDashboard::GetNumber("Autonomous Delay Sec", 0.0);
    double trench_offset_inch = frc::SmartDashboard::GetNumber("Autonomous Trench Offset Inch", 0.0);

    // Format a description of the settings back to the dashboard. This allow us to check that
    // the dashboard is working and has go the right settings
    std::string strategy_label = "<Error";
    for (int i = 0; i < kAutoModeCount; i++) {
        if (kAutoModes[i].strategy == strategy) {
            strategy_label = kAutoModes[i].label;
        }
    }
	std::ostringstream strategy_description;
  	strategy_description << strategy_label << " Delay: " << delay_s << " Offset: " << trench_offset_inch;
    frc::SmartDashboard::PutString("Autonomous Setting", strategy_description.str());
}


frc::Command* AutonomousCommand::CreateAutonomousCommand() {

    // Get the strategy, delay and offset from the dashboard
    Strategy strategy = ms_strategy_chooser.GetSelected();
    double delay_s = frc::SmartDashboard::GetNumber("Autonomous Delay Sec", 0.0);
    double trench_offset_inch = frc::SmartDashboard::GetNumber("Autonomous Trench Offset Inch", 0.0);

    // TODO Subtract the shooter wheel ramp up time from the delay

    // Create the appropriate path for the strategy
    RobotPath* robot_path = NULL;
    switch (strategy) {
        default:
        case Strategy::kShoot:
            robot_path = CreateShootPath(delay_s, trench_offset_inch);
            break;
        case Strategy::kShootAndMoveBackward:
            robot_path = CreateShootAndMoveBackwardPath(delay_s, trench_offset_inch);
            break;
        case Strategy::kShootAndMoveForeward:
            robot_path = CreateShootAndMoveForewardPath(delay_s, trench_offset_inch);
            break;
        case Strategy::kShootAndTrench:
            robot_path = CreateShootAndTrenchPath(delay_s, trench_offset_inch);
            break;
        case Strategy::kShootAndShield:
            robot_path = CreateShootAndShieldPath(delay_s, trench_offset_inch);
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

RobotPath* AutonomousCommand::CreateShootPath(double delay_s, double trench_offset_inch) {
    // Create and name the robot path
    RobotPath* robot_path = new RobotPath();
    robot_path->m_name = "Shoot";

    // Add the initial delay, if any, and shooting of the initial 3 balls 
    AddDelaySegment(robot_path, delay_s);
    AddShootSegment(robot_path);

    // Return the robot path
    return robot_path;
}

RobotPath* AutonomousCommand::CreateShootAndMoveBackwardPath(double delay_s, double trench_offset_inch) {
    // Create and name the robot path
    RobotPath* robot_path = new RobotPath();
    robot_path->m_name = "ShootAndMoveBackwards";

    // Add the initial delay, if any, and shooting of the initial 3 balls 
   AddDelaySegment(robot_path, delay_s);
   AddShootSegment(robot_path);

    // Add a segment to move the robot backwards 8 feet. This is towards the target, which
    // is 10 feet from the start line.
    const double INCH = 0.0254;
    const double FOOT = 12*INCH;
    Bezier3 path;
    path.m_point1.Set(-0.0, 0.0); // FIRST ONE HAS TO BE A NEGATIVE ZERO
    path.m_point2.Set(-2*FOOT, 0.0);
    path.m_point3.Set(-6*FOOT, 0.0);
    path.m_point4.Set(-8*FOOT, 0.0);	
    PathSegment* path_segment = new PathSegment();
    path_segment->m_name = "Straight";
    path_segment->m_path_definition.push_back(path);
    path_segment->m_reverse = true;
    robot_path->m_path_segments.push_back(path_segment);

    return robot_path;
}

RobotPath* AutonomousCommand::CreateShootAndMoveForewardPath(double delay_s, double trench_offset_inch) {
    // Create and name the robot path
    RobotPath* robot_path = new RobotPath();
    robot_path->m_name = "ShootAndMoveBackwards";

    // Add the initial delay, if any, and shooting of the initial 3 balls 
   AddDelaySegment(robot_path, delay_s);
   AddShootSegment(robot_path);

    // Add a segment to move the robot forewards 8 feet. This is away from the target, which
    // is 10 feet from the start line.
    const double INCH = 0.0254;
    const double FOOT = 12*INCH;
    Bezier3 path;
    path.m_point1.Set(0.0, 0.0);
    path.m_point2.Set(2*FOOT, 0.0);
    path.m_point3.Set(6*FOOT, 0.0);
    path.m_point4.Set(8*FOOT, 0.0);	
    PathSegment* path_segment = new PathSegment();
    path_segment->m_name = "Straight";
    path_segment->m_path_definition.push_back(path);
    path_segment->m_reverse = false;
    robot_path->m_path_segments.push_back(path_segment);

    return robot_path;
}

RobotPath* AutonomousCommand::CreateShootAndTrenchPath(double delay_s, double trench_offset_inch) {
    // Create and name the robot path
    RobotPath* robot_path = new RobotPath();
    robot_path->m_name = "ShootAndTrench";

    // Add the initial delay, if any, and shooting of the initial 3 balls 
    AddDelaySegment(robot_path, delay_s);
    AddShootSegment(robot_path);

    // See Layout and Markings Diagram page 5 https://firstfrc.blob.core.windows.net/frc2021/PlayingField/2021LayoutMarkingDiagram.pdf

    //                  Trench Balls
    //        |             o    o    o     66.91" - Distance sideways from robot at centre of target   
    //        |             122.62"   194.62"      - Distance from initiation line
    //   >    * Robot    
    //        | Initiation Line
    //
    // Robot starts on the initiation line with its back bumper just over the initiation line (most
    // of the robot further way from the target).

    const double INCH = 0.0254;
	const double ROBOT_LENGTH = 37 * INCH;

    // Distances of landmarks down the field from the initiation line
    const double BALL1_DISTANCE = 122.62 * INCH;
    const double BALL3_DISTANCE = 194.62 * INCH;

    // Distances of landmarks across the field from the directly in front of the target
    const double TRENCH_DISTANCE = 66.91 * INCH;

    // Position of the corner of the trench
    const double TRENCH_CORNER_DISTANCE_X = 86.62 * INCH;
    const double TRENCH_CORNER_DISTANCE_Y = 39.16 * INCH;

    // Extra distance to go to ensure picking up the 3rd trench ball
    const double EXTRA_PICKUP_DISTANCE = 10 * INCH;

    // Extra distance to go sideways towards the trench to account for the fact that the centre of
    // the in take is not in the centre of the robot
    const double INTAKE_OFFSET_DISTANCE = 3 * INCH;

    // Create a segment for picking up the balls
    PathSegment* pickup_segment = new PathSegment();
    pickup_segment->m_name = "Pickup Balls";
    pickup_segment->m_reverse = false;
    robot_path->m_path_segments.push_back(pickup_segment);

    // Curve left to the first ball, then forwards to pick up the three balls
    Point2D start;
    Point2D direction;
    GetStartPosition(trench_offset_inch, start, direction);
    Point2D end(BALL1_DISTANCE - ROBOT_LENGTH + EXTRA_PICKUP_DISTANCE, TRENCH_DISTANCE + INTAKE_OFFSET_DISTANCE); // End of curve is the position of the first ball
    double end_heading = 0; // Pointing down the field along the line of balls 
    ChallengePaths::AddSegment(pickup_segment, end, end_heading, start, direction);
    ChallengePaths::AddStraight(pickup_segment, BALL3_DISTANCE - BALL1_DISTANCE);
    pickup_segment->m_mechanism_actions.push_back(MechanismAction("StartIntaking", 1.0));

    // Create a segment to drive back to shoot
    PathSegment* return_segment = new PathSegment();
    return_segment->m_name = "GoToShooting";
    return_segment->m_reverse = true;
    robot_path->m_path_segments.push_back(return_segment);

    // Drive back to the position of the corner of the trench be closer to the target
    ChallengePaths::GetCurrent(pickup_segment, start, direction);
    direction = -direction;
    Point2D trench_corner(TRENCH_CORNER_DISTANCE_X, TRENCH_CORNER_DISTANCE_Y);
    double trench_corner_heading_degrees = GetTargetHeadingDegrees(trench_corner);
    ChallengePaths::AddSegment(return_segment, trench_corner, trench_corner_heading_degrees, start, direction);
    return_segment->m_mechanism_actions.push_back(MechanismAction("StopIntaking", 1.5));
    return_segment->m_mechanism_actions.push_back(MechanismAction("PrepareShooter", 1.6));

    // Rotate to face the target and shoot the balls
    AddRotateToTargetSegment(robot_path);
    AddShootSegment(robot_path);

    return robot_path;
}

RobotPath* AutonomousCommand::CreateShootAndShieldPath(double delay_s, double trench_offset_inch) {
    // Create and name the robot path
    RobotPath* robot_path = new RobotPath();
    robot_path->m_name = "Shoot";

    // Add the initial delay, if any, and shooting of the initial 3 balls 
    AddDelaySegment(robot_path, delay_s);
    AddShootSegment(robot_path);

    // See Layout and Markings Diagram page 6 https://firstfrc.blob.core.windows.net/frc2021/PlayingField/2021LayoutMarkingDiagram.pdf
    //
    //        | Initiation Line           
    //   >    * Robot    
    //        |               _
    //        |              |_|          o  Ball 3
    //        |                      o
    //        |                 o  Ball 1
    //
    // Robot starts on the initiation line with its back bumper just over the initiation line (most
    // of the robot further way from the target).

    const double INCH = 0.0254;
	const double ROBOT_LENGTH = 37 * INCH;

    // Position of the first and third balls (second is half way between). Y is negative as these
    // balls are away from the trench.
    const double BALL1_POSITION_X = 119.73 * INCH;
    const double BALL1_POSITION_Y = -42.99 * INCH;
    const double BALL3_POSITION_X = 156.69 * INCH;
    const double BALL3_POSITION_Y = -27.68 * INCH;

    // Position to shoot the balls from
    const double SHOOTING_POSITION_X = 60 * INCH;
    const double SHOOTING_POSITION_Y = -44 * INCH;

    // Extra distance to go to ensure picking up the 3rd ball
    const double EXTRA_PICKUP_DISTANCE = 10 * INCH;

    // Extra distance to go sideways towards the trench to account for the fact that the centre of
    // the in take is not in the centre of the robot
   // const double INTAKE_OFFSET_DISTANCE = 3 * INCH;


    Point2D ball1_position(BALL1_POSITION_X, BALL1_POSITION_Y);
    Point2D ball3_position(BALL3_POSITION_X, BALL3_POSITION_Y);

    Point2D ball_direction = ball3_position - ball1_position;
    ball_direction.Normalize();

    Point2D ball_pickup_position = ball1_position - ROBOT_LENGTH * ball_direction;
    double ball_pickup_distance = (ball3_position - ball1_position).Length() + EXTRA_PICKUP_DISTANCE;
    double ball_pickup_heading = ::atan2(ball_direction.y, ball_direction.x) * 180.0 / M_PI;

    // Create a segment for picking up the balls
    PathSegment* pickup_segment = new PathSegment();
    pickup_segment->m_name = "Pickup Balls";
    pickup_segment->m_reverse = false;
    robot_path->m_path_segments.push_back(pickup_segment);

    // Curve left to the first ball, then forwards to pick up the three balls
    Point2D start;
    Point2D direction;
    GetStartPosition(trench_offset_inch, start, direction);
    ChallengePaths::AddSegment(pickup_segment, ball_pickup_position, ball_pickup_heading, start, direction);
    ChallengePaths::AddStraight(pickup_segment, ball_pickup_distance);
    pickup_segment->m_mechanism_actions.push_back(MechanismAction("StartIntaking", 1.0));

    // Create a segment to drive back to shoot
    PathSegment* return_segment = new PathSegment();
    return_segment->m_name = "GoToShooting";
    return_segment->m_reverse = true;
    robot_path->m_path_segments.push_back(return_segment);

    // Drive back to the position of the corner of the trench be closer to the target
    ChallengePaths::GetCurrent(pickup_segment, start, direction);
    direction = -direction;
    Point2D shooting_position(SHOOTING_POSITION_X, SHOOTING_POSITION_Y);
    double shooting_position_heading_degrees = GetTargetHeadingDegrees(shooting_position);
    ChallengePaths::AddSegment(return_segment, shooting_position, shooting_position_heading_degrees, start, direction);
    return_segment->m_mechanism_actions.push_back(MechanismAction("StopIntaking", 1.5));
    return_segment->m_mechanism_actions.push_back(MechanismAction("PrepareShooter", 1.6));

    // Rotate to face the target and shoot the balls
    AddRotateToTargetSegment(robot_path);
    AddShootSegment(robot_path);

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

void AutonomousCommand::AddShootSegment(RobotPath* robot_path) {
    PathSegment* delay_segment = new PathSegment();
    delay_segment->m_name = "Shoot";
    delay_segment->m_reverse = false;
    delay_segment->m_mechanism_actions.push_back(MechanismAction("Shoot", 0.0));
    robot_path->m_path_segments.push_back(delay_segment);
}

void AutonomousCommand::AddRotateToTargetSegment(RobotPath* robot_path) {
    PathSegment* find_segment = new PathSegment();
    find_segment->m_name = "RotateToTarget";
    find_segment->m_reverse = false;
    find_segment->m_drivebase_action = "RotateToTarget";
    robot_path->m_path_segments.push_back(find_segment);
}

void AutonomousCommand::GetStartPosition(double trench_offset_inch, Point2D& start, Point2D& direction) {
    // We assume that the robot starts on the trench line (x coordinate of 0) and that it faces towards
    // the centre of the inner port.
    const double INCH = 0.0254;
    start.Set(0, trench_offset_inch * INCH);
    double start_heading_degrees = GetTargetHeadingDegrees(start);
    // double start_heading_degrees = ::atan2(trench_offset_inch * INCH, INNER_PORT_DISTANCE) * 180 / M_PI;
    direction = Point2D::UnitVectorDegrees(start_heading_degrees);
}

double AutonomousCommand::GetTargetHeadingDegrees(const Point2D& position) {
    const double INCH = 0.0254;

    const double INNER_PORT_DEPTH = 29.25 * INCH;  // 2ft. 5¼ in. ref 3.4.1.3
    const double INITIATION_LINE_DISTANCE = 120 * INCH;  // 10ft.  ref 3.2
    const double INNER_PORT_DISTANCE = INNER_PORT_DEPTH + INITIATION_LINE_DISTANCE;

    return ::atan2(position.y, position.x + INNER_PORT_DISTANCE) * 180 / M_PI;
}

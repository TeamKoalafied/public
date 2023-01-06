//==============================================================================
// Manipulator.cpp
//==============================================================================

#include "Manipulator.h"

#include "DriveBase.h"

#include "Mechanisms/Hood.h"
#include "Mechanisms/Indexer.h"
#include "Mechanisms/Intake.h"
#include "Mechanisms/Shooter.h"
#include "Mechanisms/Turret.h"
#include "Mechanisms/DistanceSensor.h"

#include "../RobotConfiguration.h"
#include "../KoalafiedUtilities.h"

#include <frc/Joystick.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
#include <math.h>

namespace RC = RobotConfiguration;


//==============================================================================
// Construction

Manipulator::Manipulator() :
    TSingleton<Manipulator>(this),
    JoystickSubsystem("Manipulator", RC::kJoystickPortOperator) {

    m_shooter = new Shooter;
    m_hood = new Hood;
    m_intake = new Intake;
    m_indexer = new Indexer;
    m_turret = new Turret;

    //m_distance_sensor = new DistanceSensor;

    m_state = State::Idle;
    m_automatic_tracking = false;

    m_find_target_control = NULL;
    m_haptic_controller = NULL;

    m_manual_shooting_distance_ft = 10;
}

Manipulator::~Manipulator() {
    Shutdown();
}


//==============================================================================
// frc::Subsystem Function Overrides

void Manipulator::Periodic() {
    // Perform the update for each of the mechanisms
    m_shooter->Periodic();
    m_hood->Periodic();
    m_intake->Periodic();
    m_indexer->Periodic();
    m_turret->Periodic();
    //m_distance_sensor->Periodic(true);

    // Update the currect heading to the target
    m_find_target_control->UpdateTargetHeading();

    // Perform the update for whatever state we are in
    switch (m_state) {
        case State::Intaking: UpdateIntakingState(); break;
        case State::Rejecting: UpdateIntakingState(); break;
        case State::Shooting: UpdateShootingState(); break;
        case State::ShootingManual: UpdateShootingState(); break;
        case State::PrepareShooting: UpdateShootingState(true); break;
        case State::Idle:
            if (m_automatic_tracking)  UpdateIdleTargetTracking();
            break;
    }
    
    // Update the haptic feedback
    m_haptic_controller->Periodic();
}

//==========================================================================
// Joystick Operation (from JoystickSubsystem)

void Manipulator::JoystickControlStarted() {
    JoystickSubsystem::JoystickControlStarted();
}

void Manipulator::DoJoystickControl() {
    frc::Joystick* joystick = GetJoystick();

    // IMPORTANT: Only one of the following lines should ever be uncommented at a time
    DoManualJoystickControl(joystick);
    //DoShooterCalibrationJoystickControl(joystick);
    
//    m_shooter->TestDriveShooter(joystick);
//    m_hood->TestDriveHood(joystick);
//    m_indexer->TestDriveIndexer(joystick);
//    m_turret->TestDriveTurret(joystick);
//     m_intake->TestDriveIntake(joystick);
    // m_kicker->TestDriveKicker(joystick);
}

void Manipulator::JoystickControlStopped() {
    JoystickSubsystem::JoystickControlStopped();
}


//==============================================================================
// Setup and Shutdown

void Manipulator::Setup() {
    std::cout << "Manipulator::Setup()\n";

    m_find_target_control = new FindTargetControl(DriveBase::GetInstance(), *this);
    m_haptic_controller = new HapticController(GetJoystick());
 

	frc::SmartDashboard::PutNumber("Shooter RPM", 5700.0);
	frc::SmartDashboard::PutNumber("Manual Distance (ft)", m_manual_shooting_distance_ft);
	frc::SmartDashboard::PutNumber("Intake Drive %", 45.0);
    frc::SmartDashboard::PutNumber("Target RPM", 3000);
    frc::SmartDashboard::PutNumber("Target Angle", 25);


    // Setup all the mechanisms
    m_shooter->Setup();
    m_hood->Setup();
    m_intake->Setup();
    m_indexer->Setup();
    m_turret->Setup();
    //m_distance_sensor->Setup();


}

void Manipulator::Shutdown() {
    std::cout << "Manipulator::Shutdown()\n";

    // Shutdown all the mechanisms
    m_shooter->Shutdown();
    m_hood->Shutdown();
    m_intake->Shutdown();
    m_indexer->Shutdown();
    m_turret->Shutdown();
    //m_distance_sensor->Shutdown();
}

//==========================================================================
// Autonomous Control

double Manipulator::GetTurretAngleDegrees() {
    return m_turret->GetTurretAngleDegrees();
}

bool Manipulator::SetTurretAngleDegrees(double angle_degrees, double max_speed) {
    return m_turret->SetTurretAngleDegrees(angle_degrees, max_speed);
}


//==========================================================================
// Joystick Control

void Manipulator::DoManualJoystickControl(frc::Joystick* joystick) {

    // Check for buttons that enter diffent operations
    bool intake_button = joystick->GetRawButton(RC::kJoystickAButton);
    bool reject_button = joystick->GetRawButton(RC::kJoystickXButton);
    bool shoot_button = joystick->GetRawButton(RC::kJoystickYButton);
    bool shoot_manual_button = joystick->GetRawButton(RC::kJoystickBButton);

    // Calculate the new state based on the buttons pressed. The states are tested
    // in the order of climbing, intaking, shooting. The first one found is used
    // and the others are ignored. (Operator should not be pressing multiple buttons!)
    State new_state = State::Idle;
    if (intake_button) {
        new_state = State::Intaking;
    } else if (shoot_button) {
        new_state = State::Shooting;
    } else if (shoot_manual_button) {
        new_state = State::ShootingManual;
    } else if (reject_button) {
        new_state = State::Rejecting;
    }

    // Update the state if required
    if (new_state != m_state) {
        ChangeState(new_state);
    }

    if (m_state == State::Idle) {
        // Manual control of the indexer
        double manual_indexer_drive = joystick->GetRawAxis(RC::kJoystickLeftYAxis);
        if (fabs(manual_indexer_drive) < RC::kJoystickDeadzone) manual_indexer_drive = 0.0;
        m_indexer->ManualDriveIndexer(manual_indexer_drive, manual_indexer_drive);

        // Manual control of the hood
        double manual_hood_drive = -joystick->GetRawAxis(RobotConfiguration::kJoystickRightYAxis);
        if (fabs(manual_hood_drive) < RC::kJoystickDeadzone) manual_hood_drive = 0.0;
        manual_hood_drive *= 0.2;
        m_hood->OpenLoop(manual_hood_drive);

        // Manual control of the turret. If the user does manual turret control it turns off
        // automatic tracking (otherwise they would fight each other to control the turret).
        double manual_turret_drive = -joystick->GetRawAxis(RobotConfiguration::kJoystickRightXAxis);
        if (fabs(manual_turret_drive) < RC::kJoystickDeadzone) manual_turret_drive = 0.0;
//        manual_turret_drive *= 0.8;
        // TODO Control manual turret speed sensibly
        // For now set a slow turret speed to above any change of damaging the unbilical
        manual_turret_drive *= 0.4;
        if (!m_automatic_tracking || manual_turret_drive != 0) {
            m_automatic_tracking = false;
            m_turret->OpenLoop(manual_turret_drive);
        }

        // Manual control of the shooting distance
        if (joystick->GetRawButtonPressed(RC::kJoystickLTrigButton)) {
            m_manual_shooting_distance_ft--;
            if (m_manual_shooting_distance_ft < 4) m_manual_shooting_distance_ft = 4;
        	frc::SmartDashboard::PutNumber("Manual Distance (ft)", m_manual_shooting_distance_ft);
        } else if (joystick->GetRawButtonPressed(RC::kJoystickRTrigButton)) {
            m_manual_shooting_distance_ft++;
            if (m_manual_shooting_distance_ft > 25) m_manual_shooting_distance_ft = 25;
        	frc::SmartDashboard::PutNumber("Manual Distance (ft)", m_manual_shooting_distance_ft);
        }
    }
}

void Manipulator::DoShooterCalibrationJoystickControl(frc::Joystick* joystick) {

     m_automatic_tracking = false;


    if (joystick->GetRawButton(RobotConfiguration::kJoystickAButton)) {
        // Get the shooter RPM and angle we want to shoot with
        double target_rpm = frc::SmartDashboard::GetNumber("Target RPM", 3000.0);
        double target_angle = frc::SmartDashboard::GetNumber("Target Angle", 25.0);

        // Run the shooter and the set the hood angle
        bool shooter_at_speed = m_shooter->DriveShooterClosedLoop(target_rpm);
        bool hood_at_angle = m_hood->SetHoodAngleDegrees(target_angle);

        // If the speed and hood angle are ready for shooting run the indexer and kicker to
        // move the ball up into the shooter
        if (shooter_at_speed && hood_at_angle) {
            m_indexer->ManualDriveIndexer(1.0, 1.0);
        } else {
             m_indexer->ManualDriveIndexer(0.0, 0.0);
        }
    } else {
        m_shooter->DriveShooterOpenLoop(0);

        // Manual control of the indexer. We want up on the joystick (-ve) to move the balls up
        double manual_indexer_drive = joystick->GetRawAxis(RC::kJoystickLeftYAxis);
        if (fabs(manual_indexer_drive) < RC::kJoystickDeadzone) manual_indexer_drive = 0.0;
        m_indexer->ManualDriveIndexer(-manual_indexer_drive, -manual_indexer_drive);

        // Manual control of the hood
        double manual_hood_drive = joystick->GetRawAxis(RobotConfiguration::kJoystickRightYAxis);
        if (fabs(manual_hood_drive) < RC::kJoystickDeadzone) manual_hood_drive = 0.0;
        manual_hood_drive *= 0.2;
        m_hood->OpenLoop(manual_hood_drive);

        // Manual control of the turret
        double manual_turret_drive = joystick->GetRawAxis(RobotConfiguration::kJoystickRightXAxis);
        if (fabs(manual_turret_drive) < RC::kJoystickDeadzone) manual_turret_drive = 0.0;
        manual_turret_drive *= 0.2;
        m_turret->OpenLoop(manual_turret_drive);
    }
}


//==========================================================================
// State Management

void Manipulator::ChangeState(State new_state) {
    // If the state is not changing do nothing
    if (new_state == m_state) return;

    // Leave the current state
    switch (m_state) {
        case State::Intaking: LeaveIntakingState(); break;
        case State::Rejecting: LeaveIntakingState(); break;
        case State::Shooting: LeaveShootingState(); break;
        case State::ShootingManual: LeaveShootingState(); break;
        case State::PrepareShooting: LeaveShootingState(true); break;
        case State::Idle: break;
    }

    // Record the new state as active
    m_state = new_state;

    // Enter the new state
    switch (m_state) {
        case State::Intaking: EnterIntakingState(); break;
        case State::Rejecting: EnterIntakingState(); break;
        case State::Shooting: EnterShootingState(); break;
        case State::ShootingManual: EnterShootingState(); break;
        case State::PrepareShooting: EnterShootingState(true); break;
        case State::Idle: break;
    }

    // Display the state on the dashboard
    switch (m_state) {
        case State::Intaking: frc::SmartDashboard::PutString("m_state", "Intaking"); break;
        case State::Rejecting: frc::SmartDashboard::PutString("m_state", "Rejecting"); break;
        case State::Shooting: frc::SmartDashboard::PutString("m_state", "Shooting"); break;
        case State::ShootingManual: frc::SmartDashboard::PutString("m_state", "ShootingManual"); break;
        case State::PrepareShooting: frc::SmartDashboard::PutString("m_state", "PrepareShooting"); break;
        case State::Idle:     frc::SmartDashboard::PutString("m_state", "Idle");     break;
    }
}

//==========================================================================
// Intaking State

const double INTAKE_INTAKE_SPEED = 0.5;
const double INTAKE_REJECT_SPEED = -0.5;
const double INDEXER_INTAKE_SPEED = 0.5;
const double INDEXER_REJECT_SPEED = -0.5;

void Manipulator::EnterIntakingState() {

    switch (m_state) {
        case State::Intaking: {
            // When intaking drive the intake and just the bottom of the indexer
            m_intake->Extend();

	        double drive = frc::SmartDashboard::GetNumber("Intake Drive %", 45.0)/100.0;
            m_intake->ManualDriveIntake(drive);
            m_indexer->ManualDriveIndexer(INDEXER_INTAKE_SPEED, 0.0);
            break;
        }
        case State::Rejecting:
            // When rejecting drive the intake and both the bottom and top of the indexer
            // m_intake->ManualDriveIntake(INTAKE_REJECT_SPEED);
            m_indexer->ManualDriveIndexer(INDEXER_REJECT_SPEED, INDEXER_REJECT_SPEED);
            break;
        default:
            break;
    }
}

void Manipulator::LeaveIntakingState() {
    m_intake->Retract();
    m_intake->ManualDriveIntake(0.0);
    m_indexer->ManualDriveIndexer(0.0, 0.0);    
}

void Manipulator::UpdateIntakingState() {
    // If we have a distance sensor we should drive the indexer when we sense a ball

    // If the intake has high current it has captured a ball. Start the intake retracting to
    // release the pressure on the ball and help it flick into the indexer. When the current
    // drops the intake is extended again. This method seems to compensate for the fact that
    // our intake meachanism geometry is not perfect.
    if (m_state == State::Intaking) {
        if (m_intake->HasHighCurrent()) {
            m_intake->Retract();
        } else {
            m_intake->Extend();
        }
    }
}


//==========================================================================
// Shooting State

// const double Manipulator::kIndexerDriveUpVelocity = 0.15;
// const double Manipulator::kIndexerDriveBackVelocity = -0.065;
// const double Manipulator::kKickerShootTimeS = 0.2;
// const double Manipulator::kKickerReturnTimeS = 0.2;
// const double Manipulator::kDriveUpTimeMaxS = 1.0;
// const double Manipulator::kDriveBackTimeS = 0.1;
// const double Manipulator::kShootBallDetectInches = 3.5;
// const double Manipulator::kShootErrorPercentage = 2.0;


void Manipulator::EnterShootingState(bool prepare) {
    m_shooter_on_target = false;
    m_shoot_timer.Start();
    m_shoot_timer.Reset();

    // Turn on automatic target tracking if we do automatic shooting
    // TODO Restore automatic shooting
    // For now leave automatic tracking off as it is not working.
    //m_automatic_tracking = m_state == State::Shooting;
}

void Manipulator::LeaveShootingState(bool prepare) {
    // Stop all the mechanisms we have been using, except that we do not
    // stop the shooter if we have been preparing to shoot.
    if (!prepare) m_shooter->DriveShooterOpenLoop(0);
}

void Manipulator::UpdateShootingState(bool prepare) {

    // If the shooter is already on target do nothing. This means that once think we are ready to
    // shoot the balls we just continue shooting. This just good stable behaviour.
    if (m_shooter_on_target) return;

    // Determine the speed and angle settings to use for shooting
    ShooterSetting setting;
    bool facing_target = false;
    switch (m_state) {
        case State::Shooting: {
            double distance_ft = 24.0;
            // FindTargetControl::FindResult find_result = m_find_target_control->RotateToTargetIdle(distance_ft);
             setting = GetShooterSettingForDistance(distance_ft * 12.0);
            facing_target = true;
            // find_result == FindTargetControl::FindResult::kOnTargetVision ||
            //                 find_result == FindTargetControl::FindResult::kOnTargetNoVision;
            break;
        }
        case State::ShootingManual: {
            double manual_target_distance_inch = frc::SmartDashboard::GetNumber("Manual Distance (ft)", 10.0) * 12.0;
            setting = GetShooterSettingForDistance(manual_target_distance_inch);
            facing_target = true;
            break;
        }
        case State::PrepareShooting:
            setting = GetShooterSettingForDistance(120.0);
            break;
        default:
            return;
    }

    // Display the shooter RPM and angle we want to shoot with
    frc::SmartDashboard::PutNumber("Target RPM", setting.m_shooter_speed_rpm);
    frc::SmartDashboard::PutNumber("Target Angle", setting.m_hood_angle_degrees);

    // Run the shooter and the set the hood angle
    bool shooter_at_speed = m_shooter->DriveShooterClosedLoop(setting.m_shooter_speed_rpm);
    bool hood_at_angle = m_hood->SetHoodAngleDegrees(setting.m_hood_angle_degrees);

    // If we are facing the target and the speed and hood angle are ready for shooting run
    // the indexer and kicker to move the ball up into the shooter
    if (shooter_at_speed && hood_at_angle && facing_target) {
        m_shooter_on_target = true;

        m_indexer->ManualDriveIndexer(1.0, 1.0);
    }
}

Manipulator::ShooterSetting Manipulator::GetShooterSettingForDistance(double distance_inch) {

    // Table of shooter wheel speeds in rpm for different target distances
    ShooterSetting SETTINGS_TABLE[] = {
        {  72.0, 4000, 25 }, //    6'
        {  96.0, 4100, 28 }, //    8'
        { 120.0, 4350, 31 }, //   10'
        { 144.0, 4500, 34 }, //   12'
        { 168.0, 4700, 37 }, //   14'
        { 192.0, 4900, 40 }, //   16'
        { 216.0, 5100, 43 }, //   18'
        { 240.0, 5300, 46 }, //   20'
        { 264.0, 5500, 49 }, //   22'
        { 288.0, 5700, 50 }, //   24'
        { 312.0, 6000, 50 }, //   26'

    };
    int SETTINGS_TABLE_SIZE = sizeof(SETTINGS_TABLE)/sizeof(SETTINGS_TABLE[0]);

    // Look up the shooter settings for the target distance in the table
    if (distance_inch < SETTINGS_TABLE[0].m_hub_distance_inch) {
        // If the distance is less than the first entry in the table, then use the first entry
        return SETTINGS_TABLE[0];
    }
    else if (distance_inch >= SETTINGS_TABLE[SETTINGS_TABLE_SIZE - 1].m_hub_distance_inch) {
        // If the distance is greater than the last entry in the table, then use the last entry
        return SETTINGS_TABLE[SETTINGS_TABLE_SIZE - 1];
    }
    else {
        // Search the table for the index of the end of the table segment to use
        int table_index = 1;
        while (distance_inch > SETTINGS_TABLE[table_index].m_hub_distance_inch) {
            table_index++;
            if (table_index >= SETTINGS_TABLE_SIZE) return SETTINGS_TABLE[SETTINGS_TABLE_SIZE - 1];
        }

        // Get the distance, speed and hood angle at the beginning and end of the table segment to use
        double distance0_inch = SETTINGS_TABLE[table_index - 1].m_hub_distance_inch;
        double distance1_inch = SETTINGS_TABLE[table_index].m_hub_distance_inch;
        double speed0_rpm = SETTINGS_TABLE[table_index - 1].m_shooter_speed_rpm;
        double speed1_rpm = SETTINGS_TABLE[table_index].m_shooter_speed_rpm;
        double angle0_degrees = SETTINGS_TABLE[table_index - 1].m_hood_angle_degrees;
        double angle1_degrees = SETTINGS_TABLE[table_index].m_hood_angle_degrees;

        // Calculate the shooter speed and hood angle for our exact distance using linear interpolation
        double fraction = (distance_inch - distance0_inch)/(distance1_inch - distance0_inch);
        ShooterSetting interpolated_setting;
        interpolated_setting.m_hub_distance_inch = distance_inch;
        interpolated_setting.m_shooter_speed_rpm = speed0_rpm * (1.0 - fraction) + speed1_rpm * fraction;
        interpolated_setting.m_hood_angle_degrees = angle0_degrees * (1.0 - fraction) + angle1_degrees * fraction;
        return interpolated_setting;
    }
}

void Manipulator::UpdateIdleTargetTracking() {
    double distance_ft;
    m_find_target_control->RotateToTargetIdle(distance_ft);

    m_manual_shooting_distance_ft = (int)(distance_ft + 0.5);
}

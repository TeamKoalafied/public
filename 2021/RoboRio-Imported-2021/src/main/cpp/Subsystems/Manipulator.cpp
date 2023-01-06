//==============================================================================
// Manipulator.cpp
//==============================================================================

#include "Manipulator.h"

#include "Mechanisms/Shooter.h"
#include "Mechanisms/Indexer.h"
#include "Mechanisms/Winch.h"
#include "Mechanisms/Intake.h"
#include "Mechanisms/Kicker.h"

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
    m_indexer = new Indexer;
    m_winch = new Winch;
    m_intake = new Intake;
    m_kicker = new Kicker;

    m_distanceSensor = new DistanceSensor;

    m_state = State::Idle;

    m_find_target_control = NULL;
    m_haptic_controller = NULL;
}

Manipulator::~Manipulator() {
    Shutdown();
}


//==============================================================================
// frc::Subsystem Function Overrides

void Manipulator::Periodic() {
    // Perform the update for each of the mechanisms
    m_shooter->Periodic();
    m_indexer->Periodic();
    m_winch->Periodic();
    m_intake->Periodic();
    m_kicker->Periodic();
    m_distanceSensor->Periodic(true);

    // Perform the update for whatever state we are in
    switch (m_state) {
        case State::Intaking: UpdateIntakingState(); break;
        case State::Shooting: UpdateShootingState(); break;
        case State::PrepareShooting: UpdateShootingState(true); break;
        case State::Climbing: UpdateClimbingState(); break;
        case State::Idle: break;
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
    
    // m_shooter->TestDriveShooter(joystick);
    // m_indexer->TestDriveIndexer(joystick);
    // m_winch->TestDriveWinch(joystick);
    // m_intake->TestDriveIntake(joystick);
    // m_kicker->TestDriveKicker(joystick);
}

void Manipulator::JoystickControlStopped() {
    JoystickSubsystem::JoystickControlStopped();
}


//==============================================================================
// Setup and Shutdown

void Manipulator::Setup(const FindTargetControl* find_target_control) {
    std::cout << "Manipulator::Setup()\n";

    m_find_target_control = find_target_control;
    m_haptic_controller = new HapticController(GetJoystick());
 

	frc::SmartDashboard::PutNumber("Shooter RPM", 5700.0);
    std::cout << "Jr;;perh" << std::endl;

    // Setup all the mechanisms
    m_shooter->Setup();
    m_indexer->Setup();
    m_winch->Setup();
    m_intake->Setup();
    m_kicker->Setup();

    m_distanceSensor->Setup();
}

void Manipulator::Shutdown() {
    std::cout << "Manipulator::Shutdown()\n";

    // Shutdown all the mechanisms
    m_shooter->Shutdown();
    m_indexer->Shutdown();
    m_winch->Shutdown();
    m_intake->Shutdown();
    m_kicker->Shutdown();

    m_distanceSensor->Shutdown();
}

//==========================================================================
// Mechanism Access
// void Manipulator::ExtendIntake() {
//     m_intake->Extend();
// }

// void Manipulator::RetractIntake() {
//     m_intake->Retract();
// }

// void Manipulator::RunIndexForward() {
//     m_indexer->VelocityDriveIndexer(0.09);
// }

// void Manipulator::RunIndexBack() {
//     m_indexer->VelocityDriveIndexer(-0.065);
// }


//==========================================================================
// Joystick Control

void Manipulator::DoManualJoystickControl(frc::Joystick* joystick) {

    // Check for buttons that enter diffent operations
    bool shoot_button = joystick->GetRawButton(RC::kJoystickYButton);
    bool intake_button = joystick->GetRawButton(RC::kJoystickAButton);
    bool climb_button = joystick->GetRawButton(RC::kJoystickLTrigButton);

    // Calculate the new state based on the buttons pressed. The states are tested
    // in the order of climbing, intaking, shooting. The first one found is used
    // and the others are ignored. (Operator should not be pressing multiple buttons!)
    State new_state = State::Idle;
    if (climb_button) {
        new_state = State::Climbing;
    }
    else if (intake_button) {
        new_state = State::Intaking;
    }
    else if (shoot_button) {
        new_state = State::Shooting;
    }

    // Update the state if required
    if (new_state != m_state) {
        ChangeState(new_state);
    }

    if (m_state == State::Idle) {
        UpdateClimbingState(true);

        double leftYAxisJoystickValue = joystick->GetRawAxis(RC::kJoystickLeftYAxis);
        if (fabs(leftYAxisJoystickValue) < RC::kJoystickDeadzone) leftYAxisJoystickValue = 0.0;
        m_indexer->VelocityDriveIndexer(leftYAxisJoystickValue * 0.4);
   }

/* TODO This is all for debugging and should probably be permanently removed
    // If in the idle state allow special override controls
    if (m_state == State::Idle) {

        // Run indexer and intake together
        if (joystick->GetPOV(0) == RC::kJoystickPovLeft) {
            m_indexer->ManualDriveIndexer(0.5);
            m_intake->Run();
        } else {
            m_intake->Stop();
            m_indexer->ManualDriveIndexer(0);
        }

        // Shoot, then kick
        if (joystick->GetRawButton(RC::kJoystickBButton)) {
            double shooter_wheel_rpm = (frc::SmartDashboard::GetNumber("Shooter RPM", 6000.0));
            m_shooter->DriveShooterClosedLoop(shooter_wheel_rpm);
        } else {
            m_shooter->DriveShooterOpenLoop(0);
        }

        // Comment out while testing climber - Phil
        // double rightYAxisJoystickValue = joystick->GetRawAxis(RC::kJoystickRightYAxis);
        // if (fabs(rightYAxisJoystickValue) < RC::kJoystickDeadzone) rightYAxisJoystickValue = 0.0;
        // m_shooter->ManualDriveShooter(rightYAxisJoystickValue);
      
        double leftYAxisJoystickValue = joystick->GetRawAxis(RC::kJoystickLeftYAxis);
        if (fabs(leftYAxisJoystickValue) < RC::kJoystickDeadzone) leftYAxisJoystickValue = 0.0;
        m_indexer->VelocityDriveIndexer(leftYAxisJoystickValue * 0.4);

        if (joystick->GetPOV(0) == RC::kJoystickPovDown) {
            m_kicker->SetStop();
        }
        if (joystick->GetPOV(0) == RC::kJoystickPovUp) {
            m_kicker->SetShoot();
        }
    }
    */
}


//==========================================================================
// State Management

void Manipulator::ChangeState(State new_state) {
    // If the state is not changing do nothing
    if (new_state == m_state) return;

    // Leave the current state
    switch (m_state) {
        case State::Intaking: LeaveIntakingState(); break;
        case State::Shooting: LeaveShootingState(); break;
        case State::PrepareShooting: LeaveShootingState(true); break;
        case State::Climbing: LeaveClimbingState(); break;
        case State::Idle: break;
    }

    // Record the new state as active
    m_state = new_state;

    // Enter the new state
    switch (m_state) {
        case State::Intaking: EnterIntakingState(); break;
        case State::Shooting: EnterShootingState(); break;
        case State::PrepareShooting: EnterShootingState(true); break;
        case State::Climbing: EnterClimbingState(); break;
        case State::Idle: break;
    }

    // Display the state on the dashboard
    switch (m_state) {
        case State::Intaking: frc::SmartDashboard::PutString("m_state", "Intaking"); break;
        case State::Shooting: frc::SmartDashboard::PutString("m_state", "Shooting"); break;
        case State::PrepareShooting: frc::SmartDashboard::PutString("m_state", "PrepareShooting"); break;
        case State::Climbing: frc::SmartDashboard::PutString("m_state", "Climbing"); break;
        case State::Idle:     frc::SmartDashboard::PutString("m_state", "Idle");     break;
    }
}

//==========================================================================
// Intaking State

void Manipulator::EnterIntakingState() {
    m_intake->Extend();
    m_intake->Run();
}

void Manipulator::LeaveIntakingState() {
    m_intake->Retract();
    m_intake->Stop();
    m_indexer->ManualDriveIndexer(0);    
}

void Manipulator::UpdateIntakingState() {
    const double kBallDistance = 7;
    // If we sense the ball run the indexer for a short time
    if (m_distanceSensor->GetIntakeDistance() < kBallDistance) {
        m_indexer->VelocityDriveIndexer(0.1);

        // m_indexer->ManualDriveIndexer(0.5);
    }  else {
        m_indexer->ManualDriveIndexer(0);
    }
}


//==========================================================================
// Shooting State

const double Manipulator::kIndexerDriveUpVelocity = 0.15;
const double Manipulator::kIndexerDriveBackVelocity = -0.065;
const double Manipulator::kKickerShootTimeS = 0.2;
const double Manipulator::kKickerReturnTimeS = 0.2;
const double Manipulator::kDriveUpTimeMaxS = 1.0;
const double Manipulator::kDriveBackTimeS = 0.1;
const double Manipulator::kShootBallDetectInches = 3.5;
const double Manipulator::kShootErrorPercentage = 2.0;


void Manipulator::EnterShootingState(bool prepare) {

    // TODO at the start of the game we should start assuming there is a ball in the kicker and
    // maybe at other times too.

    m_shooting_state = ShootingState::DrivingBallsUp;
    m_indexer->VelocityDriveIndexer(0.15);
    m_shoot_timer.Start();
    m_shoot_timer.Reset();
}

void Manipulator::LeaveShootingState(bool prepare) {
    // Stop all the mechanisms we have been using, except that we do not
    // stop the shooter if we have been preparing to shoot.
    if (!prepare) m_shooter->DriveShooterOpenLoop(0);
    m_kicker->SetStop();
    m_indexer->ManualDriveIndexer(0);    
}

void Manipulator::UpdateShootingState(bool prepare) {

    // If we need to turn to the target do that

    // Calculate the rpm required for the current distance to target
    // TODO Just use the dashboard for now so we can experiment
//    double target_shooter_wheel_rpm = GetShooterWheelTargetRpm();
    double target_shooter_wheel_rpm = frc::SmartDashboard::GetNumber("Shooter RPM", 6000.0);
    frc::SmartDashboard::PutNumber("Shooter Target RPM", target_shooter_wheel_rpm);
    m_shooter->DriveShooterClosedLoop(target_shooter_wheel_rpm);
 
    // Update depending on the state of the shooter
    switch (m_shooting_state) {
        case ShootingState::BallInKicker: {
            m_indexer->ManualDriveIndexer(0); // Open loop control so we don't get weird oscillations

            // If we are only preparing to shoot we stop in this state
            if (prepare) break;

            // If we are on target, up to speed and there is a ball in the kicker then kick it!
            double current_shooter_wheel_rpm = m_shooter->GetShooterRPM();
            double shooter_error_percent = 100.0*fabs((current_shooter_wheel_rpm - target_shooter_wheel_rpm)/target_shooter_wheel_rpm);

            if ((shooter_error_percent < kShootErrorPercentage) || ((shooter_error_percent < 10.0) && (m_shoot_timer.Get() > 5))) {
                std::cout << "Desired Shooter RPM: " << target_shooter_wheel_rpm << std::endl;
                std::cout << "Actual Shooter RPM: " << current_shooter_wheel_rpm << std::endl;
                m_kicker->SetShoot();
                m_shooting_state = ShootingState::KickingBall;
                m_shoot_timer.Reset();
            }
            break;
        }
        case ShootingState::DrivingBallsUp: {
            // If a ball is detected in the kicker by the distance sensor then be can settle the balls back
            double shooter_distance = m_distanceSensor->GetShooterDistance();
            if (shooter_distance <= 3.5 && shooter_distance >= 0) {
                m_indexer->VelocityDriveIndexer(kIndexerDriveBackVelocity);
                m_shooting_state = ShootingState::SettlingBallsBack;
                m_shoot_timer.Reset();
            }

            // If we drive balls up for 1s and there is no ball detected jump straight to
            // trying to shoot (probably we are empty)
            if (m_shoot_timer.Get() > kDriveUpTimeMaxS) {
                m_indexer->ManualDriveIndexer(0.0);
                m_shooting_state = ShootingState::BallInKicker;
                m_shoot_timer.Reset();
            }
            break;
        }
        case ShootingState::SettlingBallsBack:
            // After 100ms of driving back we are ready to shoot
            if (m_shoot_timer.Get() > kDriveBackTimeS) {
                m_indexer->VelocityDriveIndexer(-0.065); // SHOULD BE 0 surely?
                m_shooting_state = ShootingState::BallInKicker;
                m_shoot_timer.Reset();
            }
            break;
        case ShootingState::KickingBall:
            // After 200ms the balls should have been grabbed by the shooter wheel so return the kicker to is normal position
            if (m_shoot_timer.Get() > kKickerShootTimeS) {
                m_kicker->SetStop();

                m_shooting_state = ShootingState::KickerReturn;
                m_shoot_timer.Reset();
                
                m_ball_shoot_count++;
                std::cout << "Ball shot " << m_ball_shoot_count << "\n";
            }
            break;
        case ShootingState::KickerReturn:
            // After 200ms the kicker should be back in position so start moving the next ball up
            if (m_shoot_timer.Get() > 0.2) {
                m_shooting_state = ShootingState::DrivingBallsUp;
                m_indexer->VelocityDriveIndexer(kIndexerDriveUpVelocity);
                m_shoot_timer.Reset();
            }
            break;
    }
}

double Manipulator::GetShooterWheelTargetRpm() {
    // Table of shooter wheel speeds in rpm for different target distances
    const double INCH = 0.0254;
    const double FOOT = 12*INCH;
    double SPEED_TABLE[][2] = {
        10*FOOT, 6000.0,        // 10feet 6000rpm
        13*FOOT, 5700.0,        // 13feet 5700rpm
        16*FOOT, 5700.0,        // 16feet 5700rpm
        18*FOOT, 5700.0,        // 18feet 5700rpm
        21*FOOT, 5700.0,        // 21feet 5700rpm
    };
    int SPEED_TABLE_SIZE = sizeof(SPEED_TABLE)/sizeof(SPEED_TABLE[0]);

    // Get the distance to the target from the vision
    double target_m;
    if (m_find_target_control->GetTargetDistance(target_m)) {
        // Look up the shooter speed for the target distance in the table
        if (target_m < SPEED_TABLE[0][0]) {
            // If the distance is less than the first entry in the table, then use the speed for the first entry
            return SPEED_TABLE[0][1];
        }
        else if (target_m >= SPEED_TABLE[SPEED_TABLE_SIZE - 1][0]) {
            // If the distance is greater than the last entry in the table, then use the speed for the last entry
            return SPEED_TABLE[SPEED_TABLE_SIZE - 1][1];
        }
        else {
            // Search the table for the index of the end of the table segment to use
            int table_index = 1;
            while (target_m < SPEED_TABLE[table_index][0]) {
                table_index++;
                if (table_index >= SPEED_TABLE_SIZE) return SPEED_TABLE[SPEED_TABLE_SIZE - 1][1];
            }

            // Get the distance and speed and the beginning and end of the table segment to use
            double distance0_m = SPEED_TABLE[table_index - 1][0];
            double distance1_m = SPEED_TABLE[table_index][0];
            double speed0_rpm = SPEED_TABLE[table_index - 1][1];
            double speed1_rpm = SPEED_TABLE[table_index][1];

            // Calculate the shooter speed for our exact distance using linear interpolation
            double fraction = (target_m - distance0_m)/(distance1_m - distance0_m); 
            return speed0_rpm * (1.0 - fraction) + speed1_rpm * fraction;
        }
    }
    else {
        // Distance to the target is not valid. Just use the speed from the dashboard
        return frc::SmartDashboard::GetNumber("Shooter RPM", 6000.0);
    }
}


//==========================================================================
// Climbing State

void Manipulator::EnterClimbingState() {
    m_winch->BrakeOff();
}

void Manipulator::LeaveClimbingState() {
    m_winch->BrakeOn();
}

void Manipulator::UpdateClimbingState(bool climb_only) {
    // Drive the winch with the right Y axis, applying a normal dead zone and a speed limit
    // Up on the joystick (-ve) extended the climber.
    double joystick_value = GetJoystick()->GetRawAxis(RC::kJoystickRightYAxis);
    if (fabs(joystick_value) < RC::kJoystickDeadzone) joystick_value = 0.0;
    joystick_value *= RC::kWinchSpeedFraction;

    // If we are only allowed to climb then do nothing if the operator is trying to extend the climber
    if (climb_only && joystick_value < 0) return;

    // Drive the climber. +ve is extending the climber so we must reverse the sign.
    m_winch->ManualDriveWinch(-joystick_value);
}


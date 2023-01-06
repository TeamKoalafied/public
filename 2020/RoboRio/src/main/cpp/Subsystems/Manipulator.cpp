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
#include <iomanip>
#include <iostream>
#include <math.h>
#include <sstream>

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
}

Manipulator::~Manipulator() {
    Shutdown();
}


//==============================================================================
// frc::Subsystem Function Overrides

void Manipulator::Periodic() {
    m_shooter->Periodic();
    m_indexer->Periodic();
    m_winch->Periodic();
    m_intake->Periodic();
    m_kicker->Periodic();
    m_distanceSensor->Periodic(true);
    switch (m_state) {
        case State::Intaking: UpdateIntakingState(); break;
        case State::Shooting: UpdateShootingState(); break;
        case State::Climbing: UpdateClimbingState(); break;
        case State::Idle: break;
    }    
    
}

//==========================================================================
// Joystick Operation (from JoystickSubsystem)

void Manipulator::JoystickControlStarted() {
    JoystickSubsystem::JoystickControlStarted();
}

void Manipulator::DoJoystickControl() {
    frc::Joystick* joystick = GetJoystick();

    // IMPORTANT: Only one of thes following lines should ever be uncommented at a time
    DoManualJoystickControl(joystick);
// 
// 
    
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

void Manipulator::Setup() {
    std::cout << "Manipulator::Setup()\n";
    frc::SmartDashboard::PutNumber("dRPM", 4000.0);
    // Setup all the mechanisms
    m_shooter->Setup();
    m_indexer->Setup();
    m_winch->Setup();
    m_intake->Setup();
    m_kicker->Setup();

    m_distanceSensor->Setup();

    m_shooting_test_led_output = new frc::DigitalOutput(0);
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

double Manipulator::GetDriveBaseSlowDownFactor() {
    const double kLowSlowExtensionInch = 10.0;
    const double kHighSlowExtensionInch = 20.0;
    const double kSlowDownFactor = 0.3;

    double winch_extension_inch = m_winch->GetWinchPositionInch();
    if (winch_extension_inch < kLowSlowExtensionInch) {
        // Below the low extension go at full speed
        return 1.0;
    } else if (winch_extension_inch < kHighSlowExtensionInch) {
        // Between the low and high extension interpolate between full and slow speed
        double pos = (winch_extension_inch - kLowSlowExtensionInch) / (kHighSlowExtensionInch - kLowSlowExtensionInch);
        return (1.0 - pos)*1.0 + pos*kSlowDownFactor;
    } else {
        // Above the high extension use the slow speed
        return kSlowDownFactor;
    }
}

void Manipulator::ExtendIntake() {
    m_intake->Extend();
}

void Manipulator::RetractIntake() {
    m_intake->Retract();
}

void Manipulator::RunIndexForward() {
    m_indexer->VelocityDriveIndexer(0.09);
}

void Manipulator::RunIndexBack() {
    m_indexer->VelocityDriveIndexer(-0.065);
}

void Manipulator::Shoot() {
    double required_rpm = (frc::SmartDashboard::GetNumber("dRPM", 4000.0));
    m_shooter->DriveShooterRpm(required_rpm);

    while (!(m_shooter->ShooterAtSpeed(required_rpm))) {
        continue;    
    }

    m_kicker->SetShoot();
    m_shooting_timer.Reset();
}

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

    // If in the idle state allow special override controls
    if (m_state == State::Idle) {

        // HERE

        double dRPM = (frc::SmartDashboard::GetNumber("dRPM", 4000.0));

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
            m_shooter->DriveShooterRpm(dRPM);
        } else {
            m_shooter->ManualDriveShooter(0);
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
        case State::Climbing: LeaveClimbingState(); break;
        case State::Idle: break;
    }

    // Record the new state as active
    m_state = new_state;

    // Enter the new state
    switch (m_state) {
        case State::Intaking: EnterIntakingState(); break;
        case State::Shooting: EnterShootingState(); break;
        case State::Climbing: EnterClimbingState(); break;
        case State::Idle: break;
    }

    // Display the state on the dashboard
    switch (m_state) {
        case State::Intaking: frc::SmartDashboard::PutString("m_state", "Intaking"); break;
        case State::Shooting: frc::SmartDashboard::PutString("m_state", "Shooting"); break;
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
}

void Manipulator::UpdateIntakingState() {
    const double kBallDistance = 6;
    // If we sense the ball run the indexer for a short time
    if (m_distanceSensor->GetIntakeDistance() < kBallDistance) {
        m_indexer->VelocityDriveIndexer(0.09);
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
const double Manipulator::kDriveBackTimeS = 0.2;
const double Manipulator::kShootBallDetectInches = 3.5;


void Manipulator::EnterShootingState() {

    // TODO at the start of the game we should start assuming there is a ball in the kicker and
    // maybe at other times too.

    SetupShootingLogging();

    m_shooting_timer.Start();
    m_indexer->VelocityDriveIndexer(kIndexerDriveUpVelocity);
    m_shooting_state = ShootingState::DrivingBallsUp;
    LogEnterShootingState(ShootingState::DrivingBallsUp);
    m_shooting_timer.Reset();
}

void Manipulator::LeaveShootingState() {
    m_shooter->ManualDriveShooter(0);
    m_kicker->SetStop();
    m_shooting_test_led_output->Set(false);

    OutputShootingLog();
}

void Manipulator::UpdateShootingState() {

    // If we need to turn to the target do that

    // Calculate the rpm required for the current distance to target
    double required_rpm = (frc::SmartDashboard::GetNumber("dRPM", 4000.0));
    m_shooter->DriveShooterRpm(required_rpm);


    switch (m_shooting_state) {
        case ShootingState::BallInKicker:
            // If we are on target, up to speed and there is a ball in the kicker then kick it!
            if (m_shooter->ShooterAtSpeed(required_rpm)) {
                std::cout << "Desired " << required_rpm << std::endl;
                std::cout << "Actual " << m_shooter->getRPM() << std::endl;
                m_kicker->SetShoot();
                m_shooting_state = ShootingState::KickingBall;
                LogEnterShootingState(ShootingState::KickingBall);
                m_shooting_timer.Reset();

                // Indicate shooting with an LED for testing
                m_shooting_test_led_output->Set(true);
            }
            break;
        case ShootingState::DrivingBallsUp: {
            // If a ball is detected in the kicker by the distance sensor then be can settle the balls back
            double shooter_distance = m_distanceSensor->GetShooterDistance();
            if (shooter_distance <= 3.5 && shooter_distance >= 0) {
                m_indexer->VelocityDriveIndexer(kIndexerDriveBackVelocity);

                // Move the the 'SettlingBallsBack' state
                m_shooting_state = ShootingState::SettlingBallsBack;
                LogEnterShootingState(ShootingState::SettlingBallsBack);
                m_shooting_timer.Reset();
            }

            // If we drive balls up for 1s and there is no ball detected jump straight to
            // trying to shoot (probably we are empty)
            if (m_shooting_timer.Get() > kDriveUpTimeMaxS) {
                m_indexer->ManualDriveIndexer(0.0);

                // Move the the 'BallInKicker' state
                m_shooting_state = ShootingState::BallInKicker;
                LogEnterShootingState(ShootingState::BallInKicker);
                m_shooting_timer.Reset();
            }
            break;
        }
        case ShootingState::SettlingBallsBack:
            // After 100ms of driving back we are ready to shoot
            if (m_shooting_timer.Get() > kDriveBackTimeS) {
                m_indexer->VelocityDriveIndexer(-0.065);

                // Move the the 'BallInKicker' state
                m_shooting_state = ShootingState::BallInKicker;
                LogEnterShootingState(ShootingState::BallInKicker);
                m_shooting_timer.Reset();
            }
            break;
        case ShootingState::KickingBall:
            // After 200ms the balls should have been grabbed by the shooter wheel so return the kicker to is normal position
            if (m_shooting_timer.Get() > kKickerShootTimeS) {
                m_kicker->SetStop();

                // Clear the shooting indicator LED
                m_shooting_test_led_output->Set(false);

                // Move the the 'KickerReturn' state
                m_shooting_state = ShootingState::KickerReturn;
                LogEnterShootingState(ShootingState::KickerReturn);
                m_shooting_timer.Reset();
            }
            break;
        case ShootingState::KickerReturn:
            // After 200ms the kicker should be back in position so start moving the next ball up
            if (m_shooting_timer.Get() > kKickerReturnTimeS) {
                m_indexer->VelocityDriveIndexer(kIndexerDriveUpVelocity);

                // Move the the 'DrivingBallsUp' state
                m_shooting_state = ShootingState::DrivingBallsUp;
                LogEnterShootingState(ShootingState::DrivingBallsUp);
                m_shooting_timer.Reset();
            }
            break;
    }
}

void Manipulator::SetupShootingLogging() {
    m_shooting_log_timer.Reset();
    m_shooting_log_timer.Start();
    m_shooting_sample_list.clear();
    m_shooting_sample_list.reserve(30);

    m_indexer->ZeroIndexerPosition();
}

void Manipulator::LogEnterShootingState(ShootingState shooting_state) {
    ShootingDataSample sample;
	sample.m_time_s = m_shooting_log_timer.Get();
    sample.m_shooting_state = shooting_state;
	sample.m_shooter_rpm = m_shooter->GetShooterRpm();
    sample.m_indexer_position_inch = m_indexer->GetIndexerPositionInch();
    m_shooting_sample_list.push_back(sample);
}

void Manipulator::OutputShootingLog() {
    // Output the recorded sample as tab delimited data, with a header row and one sample per row.
    // Acculumate the output in a buffer and do a single log to the console (should be faster,
    // but have not tested it)
    std::stringstream buffer;
    buffer << "Time\tType\nShooter\tIndexer\n";
    int total_samples = m_shooting_sample_list.size();
    for (int i = 0; i < total_samples; i++) {
        const ShootingDataSample& sample = m_shooting_sample_list[i];
        buffer << std::fixed << std::setprecision(2) << sample.m_time_s << "\t";
        buffer << GetShootingStateName(sample.m_shooting_state) << "\t";
        buffer << std::fixed << std::setprecision(0) << sample.m_shooter_rpm << "\t";
        buffer << std::fixed << std::setprecision(2) << sample.m_indexer_position_inch << "\n";
    }
    std::cout << buffer.str();
}

const char* Manipulator::GetShootingStateName(ShootingState shooting_state) {
    switch (shooting_state) {
        case BallInKicker:      return "BallInKicker";
        case DrivingBallsUp:    return "DrivingBallsUp";
        case SettlingBallsBack: return "SettlingBallsBack";
        case KickingBall:       return "KickingBall";
        case KickerReturn:      return "KickerReturn";
    }
    return "<Unknown>";
}



//==========================================================================
// Climbing State

void Manipulator::EnterClimbingState() {
    m_winch->BrakeOff();
}

void Manipulator::LeaveClimbingState() {
    m_winch->BrakeOn();
}

void Manipulator::UpdateClimbingState() {
    // Drive the winch with the right Y axis, applying a normal dead zone and a 50% speed limit
    double joystick_value = GetJoystick()->GetRawAxis(RC::kJoystickRightYAxis);
    if (fabs(joystick_value) < RC::kJoystickDeadzone) joystick_value = 0.0;
    joystick_value *= 0.5;
    // temporarily change direction - Phil
    m_winch->ManualDriveWinch(-joystick_value);
}


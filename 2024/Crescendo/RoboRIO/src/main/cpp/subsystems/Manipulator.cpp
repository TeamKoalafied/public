//==============================================================================
// Manipulator.cpp
//==============================================================================

#include "Manipulator.h"

#include "SwerveDrivebase.h"
#include "ManipulatorShuffleboard.h"
#include "Mechanisms/Intake.h"
#include "Mechanisms/Diverter.h"
#include "Mechanisms/Shooter.h"
#include "Mechanisms/Lift.h"
#include "Mechanisms/Trampler.h"
#include "Mechanisms/Pivot.h"
#include "Mechanisms/Winch.h"

#include "../subsystems/Vision.h"

#include "../RobotConfiguration.h"
#include "../util/KoalafiedUtilities.h"
#include "../util/Logging.h"

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardContainer.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/Joystick.h>
#include <frc/MathUtil.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <math.h>

namespace RC = RobotConfiguration;


//==============================================================================
// Construction

Manipulator::Manipulator() {

    m_intake = new Intake;
    m_diverter = new Diverter;
    m_shooter = new Shooter;
    m_lift = new Lift;
    m_laser1 = new grpl::LaserCan(RC::kLaserCanId);
    m_laser2 = new grpl::LaserCan(24);
    m_trampler = new Trampler;
    m_pivot = new Pivot;
    m_winch = new Winch;

    m_shuffleboard = new ManipulatorShuffleboard(this);

    // m_find_target_control = NULL;
    m_haptic_controller = NULL;

    m_state = State::Idle;
}

Manipulator::~Manipulator() {
    Shutdown();
}


//==============================================================================
// frc::Subsystem Function Overrides

void Manipulator::Periodic() {
    // Perform the update for each of the mechanisms
    m_intake->Periodic();
    m_diverter->Periodic();
    m_shooter->Periodic();
    m_lift->Periodic();
    m_trampler->Periodic();
    m_pivot->Periodic();
    m_winch->Periodic(m_shuffleboard);
    m_shuffleboard->Update();
    
    // Update the haptic feedback
    m_haptic_controller->Periodic();


    m_distance_to_target = m_drivebase->GetDistanceToTarget();

}
//==============================================================================
// Setup and Shutdown

void Manipulator::Setup(SwerveDrivebase* drivebase) {
    std::cout << "Manipulator::Setup()\n";
    m_drivebase = drivebase;

    m_controller = new frc::XboxController(RC::kJoystickPortOperator);
    m_haptic_controller = new HapticController(m_controller);

    m_laser1->set_ranging_mode(grpl::LaserCanRangingMode::Short);
    m_laser1->set_timing_budget(grpl::LaserCanTimingBudget::TB20ms);
    m_laser1->set_roi(grpl::LaserCanROI{ 8, 8, 16, 16 });

    m_laser2->set_ranging_mode(grpl::LaserCanRangingMode::Short);
//    m_laser2->set_timing_budget(grpl::LaserCanTimingBudget::TB20ms);
    m_laser2->set_timing_budget(grpl::LaserCanTimingBudget::TB50ms);
    m_laser2->set_roi(grpl::LaserCanROI{ 8, 8, 16, 16 });

    // Setup all the mechanisms
    m_intake->Setup();
    m_diverter->Setup();
    m_shooter->Setup();
    m_lift->Setup();
    m_trampler->Setup();
    m_pivot->Setup();
    m_winch->Setup();
    m_shuffleboard->Setup();

    SetDefaultCommand(frc2::RunCommand(
        [this] {
        DoJoystickControl();
        }, { this }));

    frc::SmartDashboard::PutNumber("AAATestIntakeSpeed", 0.5);
    frc::SmartDashboard::PutNumber("AAATestShooterScale", 0.5);  
    frc::SmartDashboard::PutNumber("AAATestShooterRPM", 3000.0);   
    frc::SmartDashboard::PutNumber("AAADiverterSpeed", 0.5);
    frc::SmartDashboard::PutNumber("AAALiftScale", 0.3);
    frc::SmartDashboard::PutNumber("AAAIntakePOVScale", 0.5);  
    frc::SmartDashboard::PutNumber("AAATramplerPOVScale", 0.5);   
    frc::SmartDashboard::PutNumber("AAADiverterPOVScale", 0.5);
    frc::SmartDashboard::PutNumber("AAAWinchPOVScale",0.5);
    frc::SmartDashboard::PutNumber("AAAPivotAngle",19.7);
}

void Manipulator::Shutdown() {
    std::cout << "Manipulator::Shutdown()\n";

    // TODO Do we need any shutdown functions

    // Shutdown all the mechanisms
    // m_shooter->Shutdown();
    // m_hood->Shutdown();
    m_intake->Shutdown();
    // m_indexer->Shutdown();
    // m_turret->Shutdown();
    //m_distance_sensor->Shutdown();
}

void Manipulator::TeleopInit() {

}

void Manipulator::DisabledInit() {

}

//==============================================================================
// Manipulator State

units::inch_t Manipulator::GetLiftExtension() {
    return m_lift->GetExtensionInch();
}

double Manipulator::GetDrivebaseSlowdown() const  {
    const double FULL_LIFT_SLOWDOWN = 0.3;
    double lift_fraction = m_lift->GetExtensionInch() / RC::kLiftMaximumExtensionInch;

    return 1.0 * (1.0 - lift_fraction) + FULL_LIFT_SLOWDOWN * lift_fraction;

}

double Manipulator::GetLaser1() const {
    std::optional<grpl::LaserCanMeasurement> measurement = m_laser1->get_measurement();
    if (measurement.has_value() && measurement.value().status == grpl::LASERCAN_STATUS_VALID_MEASUREMENT) {
        return measurement.value().distance_mm;
    } else {
        return -1;
    }
    return 0;
}

double Manipulator::GetLaser2() const {
    std::optional<grpl::LaserCanMeasurement> measurement = m_laser2->get_measurement();
    if (measurement.has_value() && measurement.value().status == grpl::LASERCAN_STATUS_VALID_MEASUREMENT) {
        return measurement.value().distance_mm;
    } else {
        return -1;
    }
    return 0;
}

//==========================================================================
// Manipulator Control

void Manipulator::SetShootingDistance(units::inch_t distance_from_sub_woofer) {
    ShooterSetting setting = GetShooterSettingForDistance(distance_from_sub_woofer);
    m_shuffleboard->SetTuningShooterParameters(setting.pivot_angle, setting.shooter_speed);
}

units::degree_t Manipulator::GetManualPivotAngle() const {
    return m_shuffleboard->GetPivotAngle();
}

units::revolutions_per_minute_t Manipulator::GetManualShooterRPM() const {
    return m_shuffleboard->GetShooterRPM();
}

void Manipulator::DoTargetingPrefire(units::inch_t distance_to_target) {
    m_prefiring = true;
    m_prefire_setting = GetShooterSettingForDistance(distance_to_target);

    m_shooter->DriveShooterClosedLoop(m_prefire_setting.shooter_speed);
    m_pivot->DriveToAngleDegrees(m_prefire_setting.pivot_angle, 1.0, Pivot::DriveToMethod::Blend);
}

void Manipulator::DoLobbingPrefire(units::inch_t lob_distance) {
    m_prefiring = true;
    m_prefire_setting = GetLobSettingForDistance(lob_distance);

    m_shooter->DriveShooterClosedLoop(m_prefire_setting.shooter_speed);
    m_pivot->DriveToAngleDegrees(m_prefire_setting.pivot_angle, 1.0, Pivot::DriveToMethod::Blend);
}

void Manipulator::DoCustomPrefire(units::revolutions_per_minute_t shooter_speed, units::degree_t pivot_angle) {
    m_prefiring = true;
    m_prefire_setting = ShooterSetting{ 0_m, shooter_speed, pivot_angle };

    m_shooter->DriveShooterClosedLoop(m_prefire_setting.shooter_speed);
    m_pivot->DriveToAngleDegrees(m_prefire_setting.pivot_angle, 1.0, Pivot::DriveToMethod::Blend);
}

void Manipulator::StopPrefire() {
    m_prefiring = false;
    m_shooter->DriveShooterClosedLoop(0_rpm);
}

bool Manipulator::PrefireReady() const {
    // If we are not pre-firing then we are read
    if (!m_prefiring) return false;

    // We are ready if the shooter speed and pivot angle are within tolerance
    units::revolutions_per_minute_t shooter_error = units::math::abs(m_prefire_setting.shooter_speed - m_shooter->GetSpeed());
    units::degree_t pivot_error = units::math::abs(m_prefire_setting.pivot_angle  - m_pivot->GetAngle());
    return shooter_error < kShooterTolerance && pivot_error < kPivotTolerance;
}

bool Manipulator::GetPrefire(units::revolutions_per_minute_t& shooter_speed, units::degree_t& pivot_angle) const {
    shooter_speed = m_prefire_setting.shooter_speed;
    pivot_angle = m_prefire_setting.pivot_angle;
    return m_prefiring;
}


//==========================================================================
// Autonomous Control

void Manipulator::DoAutoIntake() {
    DoState(State::IntakingShooter);
}

void Manipulator::DoAutoShooting() {
    DoState(State::Shooting);
}

void Manipulator::DoAutoIntakeAndShoot() {
    DoState(State::IntakeAndShooting);
}

void Manipulator::StopAuto() {
    DoState(State::Idle);
}


//==========================================================================
// Joystick Control

void Manipulator::DoJoystickControl() {

    if (!m_controller->IsConnected()) return;


    // IMPORTANT: Only one of the following lines should ever be uncommented at a time.
    // The TestDrive*() functions are only used during development and testing.
    DoManualJoystickControl(m_controller);

    // TrapTest(m_controller);
    //m_pivot->TestDrivePivot(m_controller);
    // m_lift->TestDriveLift(m_controller);
}

void Manipulator::DoManualJoystickControl(frc::XboxController* joystick) {

    // Check for each state button being pressed, otherwise do manual control of lift/intake/winch
     if (joystick->GetAButton()) {
        DoState(State::IntakingShooter);
    } else if (joystick->GetBButton()) {
        DoState(State::IntakingTramper);
    } else if (joystick->GetYButton()) {
        DoState(State::ShootingManual);
    } else if (joystick->GetXButton()) {
        m_trampler->ManualDriveTrampler(0.3);
    } else {
        // Exit any other state the manipulator might be in
        DoState(State::Idle);

        // Use POV to move the lift to set positions
        const double kLiftVelocityFactor = 0.9;
        switch (m_pov_filter.Filter(joystick->GetPOV())) {
            case RC::kJoystickPovUp: m_lift->SetExtensionInch(18.5_in, kLiftVelocityFactor); // Full hieght (trap)
                break;
            case RC::kJoystickPovRight: m_lift->SetExtensionInch(15_in, kLiftVelocityFactor); // Amp
                break;
            case RC::kJoystickPovDown: m_lift->SetExtensionInch(0_in, kLiftVelocityFactor); // Stowed
                break;
            case RC::kJoystickPovLeft: m_lift->SetExtensionInch(5_in, kLiftVelocityFactor); // Preparing to climb
                break;
            default: 
                double lift_drive = 0;
                double left_bumper = m_controller->GetLeftTriggerAxis();
                double right_bumper = m_controller->GetRightTriggerAxis();

                // Only accept one bumper being pressed, and scale the bumper values differently to prevent driving against the bottom limit at speed
                if (left_bumper > 0 && right_bumper == 0) {
                    lift_drive = -0.17*left_bumper;
                }
                if (left_bumper == 0 && right_bumper > 0) {
                    lift_drive = 0.5*right_bumper;
                }
                // If one of the bumper buttons is being pressed, drive the lift, else hold the current position
                if (lift_drive != 0) {
                    m_lift->ManualDriveLift(lift_drive);
                } else {
                    // Only set the hold position once to prevent slipping and resetting position each cycle
                    if (!m_lift->IsExtensionSet()) {
                        m_lift->SetExtensionInch(m_lift->GetExtensionInch(), kLiftVelocityFactor);
                    }
                }
                break;
        }


        // Use the left Y joystick to manually run the note moving mechanisms in and out
        double handler_drive = frc::ApplyDeadband(joystick->GetLeftY(),RC::kJoystickDeadzone);

        // Detect whether the last automatic intaking was to the shooter or lift, and run the motors
        // so that up is 'further into' the robot, and down is to eject the note.
        bool tramp = m_diverter_position == DiverterPosition::Lift;

        double diverter_drive = tramp ? -0.5*handler_drive : 0.5*handler_drive;
        m_diverter->ManualDriveDiverter(diverter_drive);

        double intake_drive = -0.5*handler_drive;
        m_intake->ManualDriveIntake(intake_drive);

        double trap_drive = tramp ? -0.5*handler_drive : 0;
        m_trampler->ManualDriveTrampler(trap_drive);

        // Use the right Y joystick to run the winch up and down
        double winch_drive = frc::ApplyDeadband(joystick->GetRightY(),RC::kJoystickDeadzone);
        winch_drive = KoalafiedUtilities::PowerAdjust(winch_drive, 2.0);

        // If the winch is being driven, move it, else hold position
        if (winch_drive != 0) {
            m_winch->ManualDriveWinch(winch_drive);
        } else if (!m_winch->IsExtensionSet()) {
            m_winch->HoldPosition(m_winch->GetWinchPosition());
        }
        
        // Move the pivot if the winch is moving to clear space for the chain
        if (fabs(winch_drive) > 0) {
            m_pivot->DriveToAngleDegrees(64.0_deg, 1.0, Pivot::DriveToMethod::Blend);
        }


        // Use POV to choose which mechanism is driven by the right joystick y-axis, and set others to zero
        // DoPOVSelectControl(joystick);
    }
}


void Manipulator::DoPOVSelectControl(frc::XboxController* joystick) {
        double pov_select_drive = frc::ApplyDeadband(joystick->GetRightY(), RC::kJoystickDeadzone);
        double diverter_drive = 0, intake_drive = 0, trampler_drive = 0, winch_drive = 0;

        switch (joystick->GetPOV()) {
            case RC::kJoystickPovUp:
                diverter_drive = pov_select_drive*frc::SmartDashboard::GetNumber("AAADiverterPOVScale",0.5);
                break;
            case RC::kJoystickPovDown:
                intake_drive = pov_select_drive*frc::SmartDashboard::GetNumber("AAAIntakePOVScale",0.5);
                break;
            case RC::kJoystickPovLeft:
                trampler_drive = pov_select_drive*frc::SmartDashboard::GetNumber("AAATramplerPOVScale",0.5);
                break;
            case RC::kJoystickPovRight:
                winch_drive = pov_select_drive*frc::SmartDashboard::GetNumber("AAAWinchPOVScale",0.5);
                break;
        }

        // Only use POV control if the x button isnt being pressed, to prevent overwriting the x button with 0.
        m_diverter->ManualDriveDiverter(diverter_drive);
        m_intake->ManualDriveIntake(intake_drive);
        m_trampler->ManualDriveTrampler(trampler_drive);
        m_winch->ManualDriveWinch(winch_drive);
}

void Manipulator::TrapTest(frc::XboxController* joystick) {
    if (joystick->GetYButton()) {
        m_lift->SetExtensionInch(14_in, 1.0);
    } else if (joystick->GetAButton()) {
        m_lift->SetExtensionInch(0_in, 1.0);
    } else if (joystick->GetBButton()) {
        m_lift->SetExtensionInch(10_in, 1.0);
    } else {
        double lift_drive = 0;
        double left_bumper = m_controller->GetLeftTriggerAxis();
        double right_bumper = m_controller->GetRightTriggerAxis();
        if (left_bumper > 0 && right_bumper == 0) {
            lift_drive = -left_bumper;
        }
        if (left_bumper == 0 && right_bumper > 0) {
            lift_drive = right_bumper;
        }

        if (!m_lift->IsExtensionSet() || lift_drive != 0) {
            lift_drive *= 0.3;
            m_lift->ManualDriveLift(lift_drive);
        }
    }

    double winch_drive = frc::ApplyDeadband(joystick->GetRightY(), RC::kJoystickDeadzone) * frc::SmartDashboard::GetNumber("AAAWinchPOVScale",0.5);
    m_winch->ManualDriveWinch(winch_drive);

    double trap_drive = frc::ApplyDeadband(joystick->GetLeftY(), RC::kJoystickDeadzone) * frc::SmartDashboard::GetNumber("AAATramplerPOVScale",0.5);
    m_trampler->ManualDriveTrampler(trap_drive);
}

//==============================================================================
// State Handling

void Manipulator::DoState(State state) {
    if (m_state != state) {
        ExitState();
        m_state = state;
        EnterState();
    }
    UpdateState();
}

void Manipulator::EnterState() {
    switch (m_state) {
        case State::Idle:
            break;
        case State::IntakingShooter: EnterIntakingShooterState();
            break;
        case State::IntakingTramper: EnterIntakingTramperState();
            break;
        case State::Shooting: EnterShootingState(false);
            break;
        case State::ShootingManual: EnterShootingState(true);
            break;
        case State::IntakeAndShooting: EnterIntakeAndShootingState();
            break;
        case State::SwapToShoot: EnterSwapToShootState();
            break;
        case State::SwapToTrap: EnterSwapToShootState();
            break;
    }
}

void Manipulator::UpdateState() {
    switch (m_state) {
        case State::Idle:
            break;
        case State::IntakingShooter: UpdateIntakingShooterState();
            break;
        case State::IntakingTramper: UpdateIntakingTramperState();
            break;
        case State::Shooting: UpdateShootingState(false);
            break;
        case State::ShootingManual: UpdateShootingState(true);
            break;
        case State::IntakeAndShooting: UpdateIntakeAndShootingState();
            break;
        case State::SwapToShoot: UpdateSwapToShootState();
            break;
        case State::SwapToTrap: UpdateSwapToTrapState();
            break;
    }
}

void Manipulator::ExitState() {
    switch (m_state) {
        case State::Idle:
            break;
        case State::IntakingShooter: ExitIntakingShooterState();
            break;
        case State::IntakingTramper: ExitIntakingTramperState();
            break;
        case State::Shooting: ExitShootingState(false);
            break;
        case State::ShootingManual: ExitShootingState(true);
            break;
        case State::IntakeAndShooting: ExitIntakeAndShootingState();
            break;
        case State::SwapToShoot: ExitSwapToShootState();
            break;
        case State::SwapToTrap: ExitSwapToTrapState();
            break;
    }
}

void Manipulator::EnterIntakingShooterState() {
    double intake_set_drive = m_shuffleboard->GetIntakeDrive();
    double diverter_set_drive = m_shuffleboard->GetDiverterDrive();
    m_diverter_position = DiverterPosition::Shooter;
    m_diverter->DivertToPath(false, diverter_set_drive);    
    m_intake->ManualDriveIntake(intake_set_drive);
    m_intake_state = IntakeState::Start;

    m_intake_current_state = IntakeCurrentState::Start;
    m_intake_current_timer.Reset();
    m_intake_current_timer.Start();

    StartLoggingIntaking();
}

void Manipulator::UpdateIntakingShooterState() {
    const units::second_t INITIAL_SPIKE_TIME = 0.3_s;
    const double HIGH_INDEXER_CURRENT_A = 12.0;
    const units::second_t EXTRA_INTAKE_TIME = 0.1_s;
    switch (m_intake_current_state) {
        case IntakeCurrentState::Start:
            if (m_intake_current_timer.Get() > INITIAL_SPIKE_TIME) {
                m_intake_current_state = IntakeCurrentState::Sensing;
            }
            break;
        case IntakeCurrentState::Sensing:
            if (fabs(m_diverter->GetCurrent()) > HIGH_INDEXER_CURRENT_A) {
                m_intake_current_state = IntakeCurrentState::CurrentSensed;
                m_intake_current_timer.Reset();
                m_intake_current_timer.Start();
            }
            break;
        case IntakeCurrentState::CurrentSensed:
            if (m_intake_current_timer.Get() > EXTRA_INTAKE_TIME) {
                m_intake_current_state = IntakeCurrentState::Stopped;

                m_diverter->DivertToPath(false, 0.0);    
                m_intake->ManualDriveIntake(0.0);
                m_haptic_controller->DoContinuousFeedback(0.5, 1.0);
            }
            break;
        case IntakeCurrentState::Stopped:
            // Nothing to do
            break;
    }


/*
    double lasercan_distance = GetLaser2();
    if (lasercan_distance < 0) lasercan_distance = 1000;
    switch (m_intake_state){
        case IntakeState::Start:
            if (lasercan_distance < RC::kLasercanCloseShootThreshold) m_intake_state = IntakeState::LeadingEdge;
            break;
        case IntakeState::LeadingEdge:
            if (lasercan_distance > RC::kLasercanFarThreshold) {
                m_intake_state = IntakeState::Middle;
                m_intake_state_timer.Reset();
                m_intake_state_timer.Start();
            }
            break;
        case IntakeState::Middle:
            // if (lasercan_distance < 40) m_intake_state = IntakeState::TrailingEdge;
            if (m_intake_state_timer.Get() > 0.1_s) {
                m_diverter->DivertToPath(false, 0.0);    
                m_intake->ManualDriveIntake(0.0);
                m_haptic_controller->DoContinuousFeedback(0.5, 1.0);
            }
            break;
        default:
            break;
    }*/
    UpdateLoggingIntaking();
}

void Manipulator::ExitIntakingShooterState() {
    m_diverter->DivertToPath(false, 0.0);    
    m_intake->ManualDriveIntake(0.0);
    StopLoggingIntaking(false);
}

void Manipulator::EnterIntakingTramperState() {
    double intake_set_drive = m_shuffleboard->GetIntakeDrive();
    double diverter_set_drive = m_shuffleboard->GetDiverterDrive();
    double tramper_set_drive = m_shuffleboard->GetTrapDrive();
    m_diverter_position = DiverterPosition::Lift;
    m_diverter->DivertToPath(true, diverter_set_drive);    
    m_intake->ManualDriveIntake(intake_set_drive);
    m_trampler->ManualDriveTrampler(tramper_set_drive);
    m_intake_state = IntakeState::Start;
    StartLoggingIntaking();
}

void Manipulator::UpdateIntakingTramperState() {
    double lasercan_distance = GetLaser2();
    if (lasercan_distance < 0) lasercan_distance = 1000;
    switch (m_intake_state) {
        case IntakeState::Start:
            if (lasercan_distance < RC::kLasercanCloseTrapThreshold) m_intake_state = IntakeState::LeadingEdge;
            break;
        case IntakeState::LeadingEdge:
            if (lasercan_distance > RC::kLasercanFarThreshold) m_intake_state = IntakeState::Middle;
            break;
        case IntakeState::Middle:
            if (lasercan_distance < RC::kLasercanCloseTrapThreshold) m_intake_state = IntakeState::TrailingEdge;
            break;
        case IntakeState::TrailingEdge:
        case IntakeState::Stopped:
            m_diverter->DivertToPath(true, 0.0);    
            m_intake->ManualDriveIntake(0.0);
            m_trampler->ManualDriveTrampler(0.0);
            m_haptic_controller->DoContinuousFeedback(0.5, 1.0);
            // Nothing to do
    }
    UpdateLoggingIntaking();
}

void Manipulator::ExitIntakingTramperState() {
    m_diverter->DivertToPath(false, 0.0);    
    m_intake->ManualDriveIntake(0.0);
    m_trampler->ManualDriveTrampler(0.0);
    StopLoggingIntaking(false);
}

void Manipulator::EnterShootingState(bool manual) {
    m_shooter_ready = false;
    // StartLoggingShooting();
}

void Manipulator::UpdateShootingState(bool manual) {
    using rpm = units::revolutions_per_minute_t;

    if (!m_shooter_ready) {
        rpm desired_speed;
        units::degree_t desired_angle;  
        if (m_prefiring) {
            desired_speed = m_prefire_setting.shooter_speed;
            desired_angle = m_prefire_setting.pivot_angle;
        } else { 
            if (manual) {
                desired_speed = m_shuffleboard->GetShooterRPM();
                desired_angle = m_shuffleboard->GetPivotAngle();
            } else {
                ShooterSetting setting = GetShooterSettingForDistance();
                desired_speed = setting.shooter_speed;
                desired_angle = setting.pivot_angle;
            }

            m_shooter->DriveShooterClosedLoop(desired_speed);
            m_pivot->DriveToAngleDegrees(desired_angle, 1.0, Pivot::DriveToMethod::MotionMagic);

        }
        rpm shooter_error = units::math::abs(desired_speed - m_shooter->GetSpeed());
        units::degree_t pivot_error = units::math::abs(desired_angle  - m_pivot->GetAngle());
        if (!m_shooter_ready) {
            if ((shooter_error < kShooterTolerance && pivot_error < kPivotTolerance) || m_shooter_timer.Get() > 1.5_s) {
                m_shooter_ready = true;
                m_diverter->KickToShooter();
            }
        }
        
    }

    // UpdateLoggingShooting();

    // using rpm = units::revolutions_per_minute_t;
    // rpm shooter_error = m_shuffleboard->GetShooterRPM() - m_shooter->GetSpeed();
    // ShooterSetting setting = GetShooterSettingForDistance();
    // m_pivot->DriveToAngleDegrees(setting.pivot_angle, 1.0, Pivot::DriveToMethod::MotionMagic);
    // units::degree_t pivot_error = setting.pivot_angle  - m_pivot->GetAngle();
    // if (units::math::abs(shooter_error) < 50_rpm && units::math::abs(pivot_error) < 1_deg && !m_shooter_ready) {
    //     m_shooter_ready = true;
    //     m_diverter->KickToShooter();
    // }
    // UpdateLoggingShooting();
}

void Manipulator::ExitShootingState(bool manual) {
    m_diverter->ManualDriveDiverter(0.0);
    if (!m_prefiring) {
        m_shooter->DriveShooterOpenLoop(0.0);
    }
    m_pivot->ManualDrivePivot(0.0);
    // StopLoggingShooting(false);
}

void Manipulator::EnterIntakeAndShootingState() {

}

void Manipulator::UpdateIntakeAndShootingState() {
    using rpm = units::revolutions_per_minute_t;
        rpm desired_speed;
        units::degree_t desired_angle;
        bool manual = false;
        if (manual) {
            desired_speed = m_shuffleboard->GetShooterRPM();
            desired_angle = m_shuffleboard->GetPivotAngle();
        } else {
            ShooterSetting setting = GetShooterSettingForDistance();
            desired_speed = setting.shooter_speed;
            desired_angle = setting.pivot_angle;
        }
//        desired_angle = 55_deg;
//        desired_speed = 1000_rpm;

        m_shooter->DriveShooterClosedLoop(desired_speed);
        m_pivot->DriveToAngleDegrees(desired_angle, 1.0, Pivot::DriveToMethod::MotionMagic);
        m_diverter->KickToShooter();

        double intake_set_drive = m_shuffleboard->GetIntakeDrive();
        m_intake->ManualDriveIntake(intake_set_drive);
}

void Manipulator::ExitIntakeAndShootingState() {
    m_intake->ManualDriveIntake(0.0);
    m_diverter->ManualDriveDiverter(0.0);
    m_shooter->DriveShooterOpenLoop(0.0);
    m_pivot->ManualDrivePivot(0.0);
}

void Manipulator::EnterSwapToShootState() {
    m_swap_state = SwapState::Start;
    m_swap_timer.Reset();
    m_swap_timer.Start();
    // StartLoggingIntaking();
}

void Manipulator::UpdateSwapToShootState() {
    switch (m_swap_state)
    {
    case SwapState::Start:
        m_intake->ManualDriveIntake(-0.5);
        m_diverter->ManualDriveDiverter(-0.5);
        m_trampler->ManualDriveTrampler(-0.5);
        if (fabs(m_trampler->GetCurrent()) < 10 && m_swap_timer.Get() > 0.2_s) {
            m_swap_state = SwapState::PastDiverter;
            m_swap_timer.Reset();
            m_swap_timer.Start();
        }
        break;
    case SwapState::PastDiverter:
        if (m_swap_timer.Get() > 0.2_s) {
            m_swap_state = SwapState::Reinsert;
            m_swap_timer.Reset();
            m_swap_timer.Start();
        }
        break;
    case SwapState::Reinsert:
        m_trampler->ManualDriveTrampler(0.0);
        m_intake->ManualDriveIntake(0.5);
        m_diverter->ManualDriveDiverter(0.5);
        if (m_swap_timer.Get() > 0.5_s) {
            m_swap_state = SwapState::Stopped;
            m_swap_timer.Stop();
        }
        break;
    case SwapState::Stopped:
        m_intake->ManualDriveIntake(0.0);
        m_diverter->ManualDriveDiverter(0.0);
        m_haptic_controller->DoSuccessFeedback();
        break;
    }
    // UpdateLoggingIntaking();
}

void Manipulator::ExitSwapToShootState() {
    m_diverter_position = DiverterPosition::Shooter;
    // StopLoggingIntaking(false);
}

void Manipulator::EnterSwapToTrapState() {
    m_swap_state = SwapState::Start;
    m_swap_timer.Reset();
    m_swap_timer.Start();
    // StartLoggingIntaking();
}

void Manipulator::UpdateSwapToTrapState() {
    switch (m_swap_state)
    {
    case SwapState::Start:
        m_intake->ManualDriveIntake(-0.5);
        m_diverter->ManualDriveDiverter(0.5);
        if (fabs(m_diverter->GetCurrent()) < 10 && m_swap_timer.Get() > 0.2_s) {
            m_swap_state = SwapState::PastDiverter;
            m_swap_timer.Reset();
            m_swap_timer.Start();
        }
        break;
    case SwapState::PastDiverter:
        if (m_swap_timer.Get() > 0.5_s) {
            m_swap_state = SwapState::Reinsert;
            m_swap_timer.Reset();
            m_swap_timer.Start();
        }
        break;
    case SwapState::Reinsert:
        m_intake->ManualDriveIntake(0.5);
        m_diverter->ManualDriveDiverter(-0.5);
        m_trampler->ManualDriveTrampler(0.5);
        if (m_swap_timer.Get() > 0.5_s) {
            m_swap_state = SwapState::Stopped;
            m_swap_timer.Stop();
        }
        break;
    case SwapState::Stopped:
        m_intake->ManualDriveIntake(0.0);
        m_diverter->ManualDriveDiverter(0.0);
        m_trampler->ManualDriveTrampler(0.0);
        m_haptic_controller->DoSuccessFeedback();
        break;
    }
    // UpdateLoggingIntaking();
}

void Manipulator::ExitSwapToTrapState() {
    m_diverter_position = DiverterPosition::Lift;
    // StopLoggingIntaking(false);
}

Manipulator::ShooterSetting Manipulator::GetShooterSettingForDistance() const {
    const frc::AprilTag* amp_april_tag = m_drivebase->GetVision().GetAprilTag(m_drivebase->GetVision().GetSpeakerCentreTagId());
    if (amp_april_tag == nullptr) {
        // This should be impossible
        return GetShooterSettingForDistance(0_m);
    }

    const frc::Pose2d& robot_pose = m_drivebase->GetVision().GetPose();
    units::meter_t distance = robot_pose.Translation().Distance(amp_april_tag->pose.ToPose2d().Translation());



    // units::inch_t distance_from_speaker_centre = m_distance_to_target;
    // units::inch_t distance_from_sub_woofer = distance_from_speaker_centre + 44_in;
    return GetShooterSettingForDistance(distance);
}

Manipulator::ShooterSetting Manipulator::GetShooterSettingForDistance(units::inch_t distance_from_april_tag) {

    // Table of shooter wheel speeds in rpm for different target distances
    ShooterSetting SETTINGS_TABLE[] = {
        // {  56.5_in, 2700_rpm, 63_deg },
        // { 76.5_in, 2500_rpm, 57_deg },
        // { 96.5_in, 2500_rpm, 51_deg },
        // { 106.5_in, 2500_rpm, 44_deg },
        // { 126.5_in, 2700_rpm, 40.5_deg },
        { 1.5_m, 2700_rpm, 63_deg },
        { 2.0_m, 2700_rpm, 54_deg },
        { 2.5_m, 3250_rpm, 44_deg },
        { 3.0_m, 4000_rpm, 38_deg },
        { 3.5_m, 4000_rpm, 33_deg },
    };
    int SETTINGS_TABLE_SIZE = sizeof(SETTINGS_TABLE)/sizeof(SETTINGS_TABLE[0]);

    return InterpolateShooterSettingForDistance(SETTINGS_TABLE, SETTINGS_TABLE_SIZE, distance_from_april_tag);
}

Manipulator::ShooterSetting Manipulator::GetLobSettingForDistance(units::inch_t lob_distance) {

    // Table of shooter wheel speeds in rpm for different target distances
    ShooterSetting SETTINGS_TABLE[] = {
        {  7.0_m, 3000_rpm, 60_deg },
        {  7.8_m, 3000_rpm, 55_deg },
        {  8.5_m, 3500_rpm, 45_deg },
        { 11.0_m, 4500_rpm, 45_deg },
    };
    int SETTINGS_TABLE_SIZE = sizeof(SETTINGS_TABLE)/sizeof(SETTINGS_TABLE[0]);

    return InterpolateShooterSettingForDistance(SETTINGS_TABLE, SETTINGS_TABLE_SIZE, lob_distance);
}

Manipulator::ShooterSetting Manipulator::InterpolateShooterSettingForDistance(ShooterSetting* table, int table_length, units::inch_t distance) {
    // Look up the shooter settings for the target distance in the table
    if (distance < table[0].distance) {
        // If the distance is less than the first entry in the table, then use the first entry
        return table[0];
    }
    else if (distance >= table[table_length - 1].distance) {
        // If the distance is greater than the last entry in the table, then use the last entry
        return table[table_length - 1];
    }
    else {
        // Search the table for the index of the end of the table segment to use
        int table_index = 1;
        while (distance > table[table_index].distance) {
            table_index++;
            if (table_index >= table_length) return table[table_length - 1];
        }

        // Get the distance, speed and hood angle at the beginning and end of the table segment to use
        units::inch_t distance0 = table[table_index - 1].distance;
        units::inch_t distance1 = table[table_index].distance;
        units::revolutions_per_minute_t speed0 = table[table_index - 1].shooter_speed;
        units::revolutions_per_minute_t speed1 = table[table_index].shooter_speed;
        units::degree_t angle0 = table[table_index - 1].pivot_angle;
        units::degree_t angle1 = table[table_index].pivot_angle;

        // Calculate the shooter speed and hood angle for our exact distance using linear interpolation
        double fraction = (distance - distance0)/(distance1 - distance0);
        ShooterSetting interpolated_setting;
        interpolated_setting.distance = distance;
        interpolated_setting.shooter_speed = speed0 * (1.0 - fraction) + speed1 * fraction;
        interpolated_setting.pivot_angle = angle0 * (1.0 - fraction) + angle1 * fraction;
        return interpolated_setting;
    }

}


//==============================================================================
// Logging

void Manipulator::StartLoggingIntaking(bool mech_sort) {
    m_mech_sort = mech_sort;
    StopLoggingIntaking(true);

    const char* const RESULT_FILENAME = "ManipulatorIntaking.csv";
    m_intaking_csv_log_file = new Logging::CsvFile();
    m_intaking_csv_log_file->OpenLogFile(RESULT_FILENAME, std::ios::out | std::ios::app);
    if (m_intaking_csv_log_file->Fail()) {
        // If opening the file fails stop logging
        std::cout << "Manipulator::StartIntaking() - Failed to open log file\n";
        StopLoggingIntaking(true);
        return;
    }
        // Set the precision for numbers
    m_intaking_csv_log_file->SetPrecision(3, true);
    *m_intaking_csv_log_file << "Time";

    if (m_mech_sort) {
        *m_intaking_csv_log_file << "Intake Current" << "Intake Drive";
        *m_intaking_csv_log_file << "Diverter Current" << "Diverter Drive";
        *m_intaking_csv_log_file << "Trapper Current";
        *m_intaking_csv_log_file << "Laser1 distance" << "Laser2 distance" << "State" << "CState";
    } else {
        *m_intaking_csv_log_file << "Intake Current" << "Diverter Current";
        *m_intaking_csv_log_file << "Intake Drive" << "Diverter Drive";
        *m_intaking_csv_log_file << "Laser1 distance" << "Laser2 distance" << "State" << "CState";
    }
    *m_intaking_csv_log_file << "\n";

    m_intake_timer.Reset();
    m_intake_timer.Start();

}

void Manipulator::UpdateLoggingIntaking() {
    if (m_intaking_csv_log_file == nullptr) return;
    
    double intake_current = m_intake->GetCurrent();
    double intake_drive = m_intake->GetOutput();
    double diverter_current = m_diverter->GetCurrent();
    double diverter_drive = m_diverter->GetOutput();
    double trapper_current = m_trampler->GetCurrent();
    double laser1_measurement = GetLaser1();
    double laser2_measurement = GetLaser2();
    *m_intaking_csv_log_file << m_intake_timer.Get().value();
    if (m_mech_sort) {
        *m_intaking_csv_log_file << intake_current << intake_drive;
        *m_intaking_csv_log_file << diverter_current << diverter_drive;
        *m_intaking_csv_log_file << trapper_current;
        *m_intaking_csv_log_file << laser1_measurement << laser2_measurement << (int)m_intake_state << (int)m_intake_current_state;
    } else {
        *m_intaking_csv_log_file << intake_current << diverter_current;
        *m_intaking_csv_log_file << intake_drive << diverter_drive;
        *m_intaking_csv_log_file << laser1_measurement << laser2_measurement << (int)m_intake_state << (int)m_intake_current_state;
    }
    *m_intaking_csv_log_file << "\n";    
}

void Manipulator::StopLoggingIntaking(bool cancelled) {
    if (m_intaking_csv_log_file != nullptr) {
    // Record if the movement was cancelled
        if (cancelled) {
            *m_intaking_csv_log_file << "CANCELLED" << "\n";
        }
    }
    delete m_intaking_csv_log_file;
    m_intaking_csv_log_file = nullptr;
}

void Manipulator::StartLoggingShooting(bool mech_sort) {
    m_mech_sort = mech_sort;
    StopLoggingShooting(true);

    m_shooter_timer.Reset();
    m_shooter_timer.Start();

    const char* const RESULT_FILENAME = "ManipulatorShooting.csv";
    m_shooting_csv_log_file = new Logging::CsvFile();
    m_shooting_csv_log_file->OpenLogFile(RESULT_FILENAME, std::ios::out | std::ios::app);
    if (m_shooting_csv_log_file->Fail()) {
        // If opening the file fails stop logging
        std::cout << "Manipulator::StartLoggingShooting() - Failed to open log file\n";
        StopLoggingIntaking(true);
        return;
    }
        
    // Set the precision for numbers
    m_shooting_csv_log_file->SetPrecision(3, true);
    *m_shooting_csv_log_file << "Time";

    if (m_mech_sort) {
        *m_shooting_csv_log_file << "Shooter Current" << "Shooter RPM" << "Shooter Drive";
        *m_shooting_csv_log_file << "Pivot Current" << "Pivot Drive" << "Pivot Angle";
        *m_shooting_csv_log_file << "Shooter RPM";
        *m_shooting_csv_log_file << "Pivot Angle";
    } else {
        *m_shooting_csv_log_file << "Shooter Current" << "Pivot Current";
        *m_shooting_csv_log_file << "Shooter Drive" << "Pivot Drive";
        *m_shooting_csv_log_file << "Shooter RPM";
        *m_shooting_csv_log_file << "Pivot Angle";
    }
    *m_shooting_csv_log_file << "\n";
}

void Manipulator::UpdateLoggingShooting() {
    if (m_shooting_csv_log_file == nullptr) return;
    
    double shooter_current = m_shooter->GetCurrent().value();
    double shooter_rpm = m_shooter->GetSpeed().value();
    double shooter_drive = m_shooter->GetOutput();
    double pivot_current = 0;
    double pivot_drive = 0;
    double pivot_angle = 0;

    *m_shooting_csv_log_file << m_shooter_timer.Get().value();
    if (m_mech_sort) {
        *m_shooting_csv_log_file << shooter_current << shooter_rpm << shooter_drive;
        *m_shooting_csv_log_file << pivot_current << pivot_drive << pivot_angle;
    } else {
        *m_shooting_csv_log_file << shooter_current << pivot_current;
        *m_shooting_csv_log_file << shooter_drive << pivot_drive;
        *m_shooting_csv_log_file << shooter_rpm;
        *m_shooting_csv_log_file << pivot_angle;
    }
    *m_shooting_csv_log_file << "\n";
}

void Manipulator::StopLoggingShooting(bool cancelled) {
    if (m_shooting_csv_log_file != nullptr) {
    // Record if the movement was cancelled
        if (cancelled) {
            *m_shooting_csv_log_file << "CANCELLED" << "\n";
        }
    }
    delete m_shooting_csv_log_file;
    m_shooting_csv_log_file = nullptr;
}
//==============================================================================
// Intake.cpp
//==============================================================================

#include "NewIntake.h"

#include "../Manipulator.h"
#include "../../RobotConfiguration.h"
#include "../../util/KoalafiedUtilities.h"
#include "../../util/Logging.h"

#include <frc/MathUtil.h>
#include <frc/Joystick.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
#include "../../Phoenix5Header.h"



namespace RC = RobotConfiguration;

const double DEFAULT_CONE_SPEED = 1.0;
const double DEFAULT_CUBE_SPEED = -0.7;
const double DEFAULT_CONE_CURRENT = 16;
const double DEFAULT_CUBE_CURRENT = 13;
const double DEFAULT_CONE_TIMEOUT = 0.3;
const double DEFAULT_CUBE_TIMEOUT = 0.0;


//==============================================================================
// Construction

NewIntake::NewIntake()  {
    m_intake_speed_controller = NULL;
}

NewIntake::~NewIntake() {
    Shutdown();
}


//==============================================================================
// Mechanism Lifetime - Setup, Shutdown and Periodic

void NewIntake::Setup() {
    std::cout << "NewIntake::Setup()\n";

    // Create and configure the intake roller Talon
    m_intake_speed_controller = new TalonSRX(RobotConfiguration::kIntakeTalonId);
    TalonSRXConfiguration intake_configuration;

    // Current limit
    intake_configuration.continuousCurrentLimit = RC::kIntakeMotorContinuousCurrentLimit.value();
    intake_configuration.peakCurrentLimit = RC::kIntakeMotorPeakCurrentLimit.value();
    intake_configuration.peakCurrentDuration = RC::kIntakeMotorPeakCurrentDurationMs.convert<units::millisecond>().value();

    // Use a fairly long ramp time so that the current doesn't exceed the high current limit
    // and cause the intake to retract.
    intake_configuration.openloopRamp = 1;

    // No feedback sensor, so no close loop control

    // Do all configuration and log if it fails
    int error = m_intake_speed_controller->ConfigAllSettings(intake_configuration, RC::kTalonTimeoutMs);
    if (error != 0) {
        std::cout << "Configuration of the intake Talon failed with code:  " << error << "\n";
    }
    
    // Perform non-configuration setup
    m_intake_speed_controller->EnableCurrentLimit(true);

    // Brake mode to try and hold the cube better (does not do much)
	m_intake_speed_controller->SetNeutralMode(NeutralMode::Brake);
}

void NewIntake::Shutdown() {
    std::cout << "NewIntake::Shutdown()\n";
}

void NewIntake::Periodic() {      

}


//==============================================================================
// Operations

void NewIntake::IntakeCone(double current_limit, double speed, double timeout) {
    // double speed, current_limit;
    units::second_t current_time = units::second_t(timeout);
    // if (m_mod_widgets != nullptr) {
    //     speed = m_mod_widgets->m_cone_speed->GetEntry()->GetDouble(DEFAULT_CONE_SPEED);
    //     current_limit = m_mod_widgets->m_cone_current->GetEntry()->GetDouble(DEFAULT_CONE_CURRENT);
    //     current_time = units::second_t(m_mod_widgets->m_cone_timeout->GetEntry()->GetDouble(DEFAULT_CONE_TIMEOUT));
    // } else {
    //     speed = DEFAULT_CONE_SPEED;
    //     current_limit = DEFAULT_CONE_CURRENT;
    //     current_time = units::second_t(DEFAULT_CONE_TIMEOUT);
    // }

    
    // Move from idle to intaking
    if (m_intake_state == IntakingState::Idle) m_intake_state = IntakingState::Intaking; 

    // While current is below the limit, run at speed in intaking state
    if (m_intake_state == IntakingState::Intaking){
        if (fabs(m_intake_speed_controller->GetStatorCurrent()) > current_limit) {
            m_intake_state = IntakingState::HighCurrent;
            m_intake_current_timer.Start();
        }
        m_intake_speed_controller->Set(ControlMode::PercentOutput, speed);
    }

    // While timer less than the timeout, run at speed in high current state
    if (m_intake_state == IntakingState::HighCurrent){
        if (m_intake_current_timer.Get() > current_time) {
          m_intake_state = IntakingState::HasGamePiece;  
        }
        m_intake_speed_controller->Set(ControlMode::PercentOutput, speed);
    }

    // Once timer finishes, hold  in has game piece state (prevents looping back to idle while holding A)
    if (m_intake_state == IntakingState::HasGamePiece) {
        m_intake_speed_controller->Set(ControlMode::PercentOutput, 0);
    } 
}

void NewIntake::IntakeCube(double current_limit, double speed, double timeout) {
    // double speed, current_limit;
    units::second_t current_time = units::second_t(timeout);
    // if (m_mod_widgets != nullptr) {
    //     speed = m_mod_widgets->m_cube_speed->GetEntry()->GetDouble(DEFAULT_CUBE_SPEED);
    //     current_limit = m_mod_widgets->m_cube_current->GetEntry()->GetDouble(DEFAULT_CUBE_CURRENT);
    //     current_time = units::second_t(m_mod_widgets->m_cube_timeout->GetEntry()->GetDouble(DEFAULT_CUBE_TIMEOUT));
    // } else {
    //     speed = DEFAULT_CUBE_SPEED;
    //     current_limit = DEFAULT_CUBE_CURRENT;
    //     current_time = units::second_t(DEFAULT_CUBE_TIMEOUT);
    // }
 
    // Move from idle to intaking
    if (m_intake_state == IntakingState::Idle) m_intake_state = IntakingState::Intaking; 

    // While current is below the limit, run at speed in intaking state
    if (m_intake_state == IntakingState::Intaking){
        if (fabs(m_intake_speed_controller->GetStatorCurrent()) > current_limit) {
            m_intake_state = IntakingState::HighCurrent;
            m_intake_current_timer.Start();
        }
        m_intake_speed_controller->Set(ControlMode::PercentOutput, speed);
    }

    // While timer less than the timeout, run at speed in high current state
    if (m_intake_state == IntakingState::HighCurrent){
        if (m_intake_current_timer.Get() > current_time) {
          m_intake_state = IntakingState::HasGamePiece;  
        }
        m_intake_speed_controller->Set(ControlMode::PercentOutput, speed);
    }

    // Once timer finishes, hold  in has game piece state (prevents looping back to idle while holding A)
    if (m_intake_state == IntakingState::HasGamePiece) {
        m_intake_speed_controller->Set(ControlMode::PercentOutput, 0);
    } 
}

bool NewIntake::HasGamePiece() {
    return m_intake_state == IntakingState::HasGamePiece;
}


void NewIntake::DropCone() {
    m_intake_speed_controller->Set(ControlMode::PercentOutput, -1.0);
    m_intake_state = IntakingState::Idle;
}

void NewIntake::DropCube() {
    m_intake_speed_controller->Set(ControlMode::PercentOutput, 1.0);
    m_intake_state = IntakingState::Idle;
}

void NewIntake::Stop() {
    m_intake_speed_controller->Set(ControlMode::PercentOutput, 0);
    
    // Re-enable intaking
    m_intake_state = IntakingState::Idle;
}


//==============================================================================
// Shuffleboard & Logging

void NewIntake::StartLoggingIntake(const char* game_piece) {
    // Disable intake logging
    return;

    StopLoggingIntake();

    // Open the CSV log file
    const char* const RESULT_FILENAME = "/home/lvuser/Intake.csv";
    m_intake_csv_log_file = new Logging::CsvFile();
    m_intake_csv_log_file->Open(RESULT_FILENAME, std::ios::out | std::ios::app);
    if (m_intake_csv_log_file->Fail()) {
        // If opening the file fails stop logging
        std::cout << "Manipulator::StartLoggingIntake() - Failed to open log file\n";
        StopLoggingIntake();
        return;
    }

    // Write an initial line saying which position we are going to
    *m_intake_csv_log_file << std::string("Intaking ") + game_piece << "\n";

    // Set the precision for numbers
    m_intake_csv_log_file->SetPrecision(3, true);

    // Column heading for output data
    *m_intake_csv_log_file << "Time";
    *m_intake_csv_log_file << "Roller Output" << "Roller Current" << "\n";

    m_intake_timer.Reset();
    m_intake_timer.Start();
}

void NewIntake::UpdateLoggingIntake() {
    if (m_intake_csv_log_file == nullptr) return;

    double roller_output = m_intake_speed_controller->GetMotorOutputPercent();
    double roller_current = m_intake_speed_controller->GetStatorCurrent();
    
    *m_intake_csv_log_file << m_intake_timer.Get().value();
    *m_intake_csv_log_file << roller_output << roller_current << "\n";
}

void NewIntake::StopLoggingIntake() {
    delete m_intake_csv_log_file;
    m_intake_csv_log_file = nullptr;
}

double NewIntake::GetCurrent() const {
    return m_intake_speed_controller->GetStatorCurrent();
}
//==============================================================================
// Testing

void NewIntake::ManualDriveIntake(double percentage_output) {
    m_intake_speed_controller->Set(ControlMode::PercentOutput, percentage_output);
}

void NewIntake::TestDriveIntake(frc::XboxController* joystick) {
    double roller_drive = frc::ApplyDeadband(joystick->GetLeftY(), RC::kJoystickDeadzone);
    m_intake_speed_controller->Set(ControlMode::PercentOutput, roller_drive);
}

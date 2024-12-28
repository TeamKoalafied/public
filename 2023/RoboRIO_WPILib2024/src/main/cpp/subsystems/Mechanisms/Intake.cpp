//==============================================================================
// Intake.cpp
//==============================================================================

#include "Intake.h"

#include "../Manipulator.h"
#include "../../RobotConfiguration.h"
#include "../../util/KoalafiedUtilities.h"

#include <frc/MathUtil.h>
#include <frc/Joystick.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
#include "../../Phoenix5Header.h"



namespace RC = RobotConfiguration;




//==============================================================================
// Construction

Intake::Intake()  {
    m_intake_speed_controller = NULL;
}

Intake::~Intake() {
    Shutdown();
}


//==============================================================================
// Mechanism Lifetime - Setup, Shutdown and Periodic

void Intake::Setup() {
    std::cout << "Intake::Setup()\n";

    // Create and configure the intake roller Talon
    m_intake_speed_controller = new TalonSRX(RobotConfiguration::kIntakeTalonId);
    TalonSRXConfiguration intake_configuration;

    // Current limit
    intake_configuration.continuousCurrentLimit = RobotConfiguration::kIntakeMotorContinuousCurrentLimit;
    intake_configuration.peakCurrentLimit = RobotConfiguration::kIntakeMotorPeakCurrentLimit;
    intake_configuration.peakCurrentDuration = RobotConfiguration::kIntakeMotorPeakCurrentDurationMs;

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

    m_winch_controller = new TalonSRX(RobotConfiguration::kWinchTalonId);
    TalonSRXConfiguration winch_configuration;
    
    // Current limit
    winch_configuration.continuousCurrentLimit = RobotConfiguration::kWinchMotorContinuousCurrentLimit;
    winch_configuration.peakCurrentLimit = RobotConfiguration::kWinchMotorPeakCurrentLimit;
    winch_configuration.peakCurrentDuration = RobotConfiguration::kWinchMotorPeakCurrentDurationMs;

	// PID parameters determined by velocity tuning
    winch_configuration.slot0.kF = 20.0;
    winch_configuration.slot0.kP = 3.0;
    winch_configuration.slot0.kI = 0;
    winch_configuration.slot0.kD = 0;

	// JE motors use a quadrature encoder
    winch_configuration.primaryPID.selectedFeedbackSensor = FeedbackDevice::QuadEncoder;

    winch_configuration.peakOutputForward = 1.0;
    winch_configuration.peakOutputReverse = -1.0;

    // Motion magic set from testing
    winch_configuration.motionCruiseVelocity = 30;
    winch_configuration.motionAcceleration = winch_configuration.motionCruiseVelocity * 4.0;

	// Do configuration
    int winch_error = m_winch_controller->ConfigAllSettings(winch_configuration, RC::kTalonTimeoutMs);
    if (winch_error != 0) {
        std::cout << "Configuration of the intake Talon failed with code:  " << error << "\n";
    }

    m_winch_controller->EnableCurrentLimit(true);

	// Brake mode to make it holds position
	m_winch_controller->SetNeutralMode(NeutralMode::Brake);

    // The JE motor encoder is wired so that the sensor is reversed (A to 1, B to 2)
    m_winch_controller->SetSensorPhase(true);

  	// Initialise the encoder to zero, for testing assume the wrist is pointing up.
    m_winch_controller->SetSelectedSensorPosition(0, RC::kTalonPidIdx, RC::kTalonTimeoutMs);

    // Initialise the state machine
    m_winch_state = WinchState::Unknown;
    m_open_winch_encoder = 0;
    m_closed_winch_encoder = -250;
}

void Intake::Shutdown() {
    std::cout << "Intake::Shutdown()\n";
}

void Intake::Periodic() {      
    // frc::SmartDashboard::PutNumber("Intake Current", m_intake_speed_controller->GetStatorCurrent());        
    // frc::SmartDashboard::PutNumber("Intake Output", m_intake_speed_controller->GetMotorOutputPercent());         

    const double HIGH_CURRENT = 5.0;
    //const double LOW_CURRENT = 0.5;

    double winch_current = fabs(m_winch_controller->GetSupplyCurrent());
    double winch_encoder = m_winch_controller->GetSelectedSensorPosition();


    switch (m_winch_state) {
        case WinchState::Openning:
            if (winch_current > HIGH_CURRENT) {
                m_winch_state = WinchState::OpenCurrent;
                m_winch_current_timer.Reset();
                m_winch_current_timer.Start();
            }
            break;
        case WinchState::OpenCurrent:
            if (winch_current < HIGH_CURRENT) {
                m_winch_state = WinchState::Openning;
            }
            else {
                if (m_winch_current_timer.Get() > 0.2_s) {
                    m_winch_state = WinchState::Open;
                    m_open_winch_encoder = winch_encoder;
                    m_winch_controller->Set(ControlMode::PercentOutput, 0);
                    std::cout << "WinchState::Open at " << winch_encoder << "\n";
                }
            }
            break;
        default:
            break;
    }

    // frc::SmartDashboard::PutNumber("Intake Winch Pos", m_winch_controller->GetSelectedSensorPosition());     
    if (m_intake_widgets!= nullptr) {
        IntakeWidgets* iw = m_intake_widgets;

        iw->m_winch_position_widget->GetEntry()->SetDouble(m_winch_controller->GetSelectedSensorPosition());
        iw->m_winch_current_widget->GetEntry()->SetDouble(m_winch_controller->GetStatorCurrent());
        switch (m_winch_state) {
            case WinchState::Openning:     iw->m_winch_state_widget->GetEntry()->SetString("Opening"); break;
            case WinchState::OpenCurrent:  iw->m_winch_state_widget->GetEntry()->SetString("OpenCurrent"); break;
            case WinchState::Seeking:      iw->m_winch_state_widget->GetEntry()->SetString("Seeking"); break;
            case WinchState::Open:         iw->m_winch_state_widget->GetEntry()->SetString("Open"); break;
            case WinchState::Unknown:      iw->m_winch_state_widget->GetEntry()->SetString("Unknown"); break;
        }
        iw->m_winch_encoder_widget->GetEntry()->SetDouble(m_winch_controller->GetSelectedSensorPosition());
        iw->m_roller_current_widget->GetEntry()->SetDouble(m_intake_speed_controller->GetStatorCurrent());
    }

	frc::SmartDashboard::PutNumber("WinchEncoder", m_winch_controller->GetSelectedSensorPosition());
	frc::SmartDashboard::PutNumber("WinchVelocity", m_winch_controller->GetSelectedSensorVelocity());
}

void Intake::CreateShuffleboardSummary(frc::ShuffleboardTab& shuffleboard_tab, int x, int y) {
	frc::ShuffleboardLayout &layout =
        shuffleboard_tab.GetLayout("Intake",frc::BuiltInLayouts::kGrid)
            .WithPosition(x, y)
            .WithSize(12, 2);

	m_intake_widgets = new IntakeWidgets;
	IntakeWidgets* iw = m_intake_widgets;

    iw->m_winch_position_widget = &layout.Add("Winch Position", 0.0).WithPosition(0,0).WithSize(3,2);
    iw->m_winch_current_widget = &layout.Add("Winch Current", 0.0).WithPosition(1,0).WithSize(3,2);
    iw->m_winch_state_widget = &layout.Add("Winch State", "Not Set").WithPosition(2, 0).WithSize(3, 2);
    iw->m_winch_encoder_widget = &layout.Add("Winch Encoder", 0.0).WithPosition(3, 0).WithSize(3, 2);
    iw->m_roller_current_widget = &layout.Add("Roller Current", 0.0).WithPosition(4, 0).WithSize(3, 2);

}

double Intake::GetRollerOutput() {
    return m_intake_speed_controller->GetMotorOutputPercent();
}

double Intake::GetRollerCurrent() {
    return m_intake_speed_controller->GetStatorCurrent();
}

double Intake::GetWinchPosition() {
    return m_winch_controller->GetSelectedSensorPosition();
}

double Intake::GetWinchCurrent() {
    return m_winch_controller->GetStatorCurrent();
}

//==============================================================================
// Operations

void Intake::GoToPosition(Position position) {
    if (position == Position::OpenFully) {
        if (!(m_winch_state == WinchState::Open ||
              m_winch_state == WinchState::Openning ||
              m_winch_state == WinchState::OpenCurrent)) {
            // Drive the intake open fully. It will stop when the current limit is hit
            m_winch_state = WinchState::Openning;
            m_winch_controller->Set(ControlMode::PercentOutput, -1.0);
        }
    }
    else {
        double winch_seek_encoder = SeekEncoderForPosition(position);

        if (m_winch_state != WinchState::Seeking || m_winch_seek_encoder != winch_seek_encoder) {
            m_winch_state = WinchState::Seeking;
            m_winch_seek_encoder = winch_seek_encoder;
            std::cout << "Seeking to " << winch_seek_encoder << "\n";
            m_winch_controller->Set(ControlMode::MotionMagic, winch_seek_encoder);
        }
    }
}

bool Intake::IsAtPosition(Position position) {
    if (position == Position::OpenFully) {
        return m_winch_state == WinchState::Open;
    }
    else {
        double seek_encoder = SeekEncoderForPosition(position);
        double delta = fabs(seek_encoder - m_winch_controller->GetSelectedSensorPosition());
        return delta < 5;
    }
}

double Intake::SeekEncoderForPosition(Position position) {
        double offset_from_open = 0;
        switch (position) {
            case Position::OpenCone: offset_from_open = 150; break;
            case Position::OpenCube: offset_from_open = 50; break;
            case Position::ClosedFully: offset_from_open = 350; break;
            case Position::ClosedCone: offset_from_open = 350; break;
            case Position::ClosedCube: offset_from_open = 250; break;
            default: break;
        }
        return m_open_winch_encoder + offset_from_open;
}


void Intake::RunCone() {
    double speed = 0.8;
    double current_limit = 12.5;
    units::second_t current_time = 0.5_s;

    if (m_intake_high_current && m_intake_current_timer.Get() > current_time) {
        m_intake_speed_controller->Set(ControlMode::PercentOutput, 0);
        GoToPosition(Position::ClosedCone);
    } else {
        if (m_intake_speed_controller->GetStatorCurrent() > current_limit && !m_intake_high_current) {
            m_intake_high_current = true;
            m_intake_current_timer.Start();
        }
        m_intake_speed_controller->Set(ControlMode::PercentOutput, speed);
        GoToPosition(Position::OpenCone);
    }
}
void Intake::RunCube() {
    double speed = 0.4;
    double current_limit = 10;
    units::second_t current_time = 0.3_s;
 
    if (m_intake_high_current && m_intake_current_timer.Get() > current_time) {
        m_intake_speed_controller->Set(ControlMode::PercentOutput, 0);
        GoToPosition(Position::ClosedCube);
    } else {
        if (m_intake_speed_controller->GetStatorCurrent() > current_limit && !m_intake_high_current) {
            m_intake_high_current = true;
            m_intake_current_timer.Start();
        }
        m_intake_speed_controller->Set(ControlMode::PercentOutput, speed);
        GoToPosition(Position::OpenCube);
    }
}

void Intake::DropGamePiece() {
    GoToPosition(Position::OpenFully);
}


// Stop running the intake for cones
void Intake::StopCone() {
    m_intake_high_current = false;
    m_intake_speed_controller->Set(ControlMode::PercentOutput, 0);
    GoToPosition(Position::ClosedFully);
}

// Stop running the intake for cubes
void Intake::StopCube() {
    m_intake_high_current = false;
    m_intake_speed_controller->Set(ControlMode::PercentOutput, 0.1);
    GoToPosition(Position::ClosedCube);
}

void Intake::ManualDriveIntake(double percentage_output) {
    m_intake_speed_controller->Set(ControlMode::PercentOutput, percentage_output);
}

void Intake::ManualDriveWinch(double percentage_output) {
    m_winch_controller->Set(ControlMode::PercentOutput, percentage_output);
    m_winch_state = WinchState::Unknown;
}


bool Intake::HasHighCurrent() {
    return m_intake_speed_controller->GetStatorCurrent() > 12.5;
}

void Intake::TestDriveIntake(frc::XboxController* joystick) {
    //double MAX_RPM = 420.0;



    // if (joystick->GetYButton()) OpenIntake();
    // else if (joystick->GetAButton()) CloseIntake();
    // else if (joystick->GetBButton()) StopIntake();
    // else {
    //     // Close loop test operation
    //     // - Up/Down - motion magic
    //     // - Left/Right - position
    //     switch (joystick->GetPOV(0)) {
    //         case RC::kJoystickPovUp:    m_winch_controller->Set(ControlMode::MotionMagic, 0); break;
    //         case RC::kJoystickPovDown:  m_winch_controller->Set(ControlMode::MotionMagic, - 250); break;
    //         default: {
    //             // Use the right joystick X axis to control the speed of the hood. Do closed loop if the
    //             // left trigger button is held down.    
    //             double winch_drive = frc::ApplyDeadband(joystick->GetRightY(), RC::kJoystickDeadzone);
    //             bool closed_loop = joystick->GetLeftBumper();
    //             KoalafiedUtilities::TuneDriveTalon(m_winch_controller, "Winch", winch_drive, MAX_RPM, closed_loop, 44.4 * 4.0);
    //             break;
    //         }
    //     }
    // }

    double roller_drive = frc::ApplyDeadband(joystick->GetLeftY(), RC::kJoystickDeadzone);
    m_intake_speed_controller->Set(ControlMode::PercentOutput, roller_drive);
}

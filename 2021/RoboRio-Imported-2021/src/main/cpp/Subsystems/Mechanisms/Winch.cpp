//==============================================================================
// Winch.cpp
//==============================================================================

#include "Winch.h"

#include "../../RobotConfiguration.h"
#include "../../KoalafiedUtilities.h"
#include "../../Subsystems/DriveBase.h"

#include <ctre/Phoenix.h>
#include <frc/Joystick.h>
#include <frc/Solenoid.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

namespace RC = RobotConfiguration;


//==============================================================================
// Construction

Winch::Winch()  {
    m_winch_speed_controller = NULL;
    m_brake_solenoid = NULL;
}

Winch::~Winch() {
    Shutdown();
}


//==============================================================================
// Mechanism Lifetime - Setup, Shutdown and Periodic

void Winch::Setup() {
    std::cout << "Winch::Setup()\n";
    
    // Create and configure the winch Talon
    m_winch_speed_controller = new TalonSRX(RC::kWinchTalonId);
    TalonSRXConfiguration winch_configuration;

    // Current limit
    winch_configuration.continuousCurrentLimit = RC::kWinchMotorContinuousCurrentLimit;
    winch_configuration.peakCurrentLimit = RC::kWinchMotorPeakCurrentLimit;
    winch_configuration.peakCurrentDuration = RC::kWinchMotorPeakCurrentDurationMs;

    // Nominal and peak outputs
    // TODO These are defaults. Winch drive is very asynmetric so these should probably be
    // set to something non-standard.
    winch_configuration.nominalOutputForward = 0.0;
    winch_configuration.nominalOutputReverse = 0.0;
    winch_configuration.peakOutputForward = 1.0;
    winch_configuration.peakOutputReverse = -1.0;

    // Ramp rates. This winch is very powerful so give it a ramp rate to reduce jerky movement
    winch_configuration.openloopRamp = 0.1;
    winch_configuration.closedloopRamp = 0.1;

    // Voltage compensation

    // Feedback sensor
    winch_configuration.primaryPID.selectedFeedbackSensor = FeedbackDevice::CTRE_MagEncoder_Absolute;

    // Set up a soft forward limit as we do not have a limit switch to protect against extending
    // up to fars
	double winch_circumference_inch = RC::kWinchDiameterInch * M_PI;
	double max_extension_revolutions = RC::kWinchMaximumExtensionInch/winch_circumference_inch;
    winch_configuration.forwardSoftLimitThreshold = max_extension_revolutions * RC::kCtreEnocderCounts;
    std::cout << "forwardSoftLimitThreshold = " << winch_configuration.forwardSoftLimitThreshold << "\n";
    winch_configuration.forwardSoftLimitEnable = true;

    // Clear the encode position on the reverse limit switch, that is when the climber is full
    // retracted. 
    winch_configuration.clearPositionOnLimitR = true;

    // Do all configuration and log if it fails
    int error = m_winch_speed_controller->ConfigAllSettings(winch_configuration, RC::kTalonTimeoutMs);
    if (error != 0) {
        std::cout << "Configuration of the winch Talon failed with code:  " << error << "\n";
    }

    // Perform non-configuration setup
    m_winch_speed_controller->SetSensorPhase(false); // Not reversed
    m_winch_speed_controller->EnableCurrentLimit(true);
	m_winch_speed_controller->SetNeutralMode(NeutralMode::Brake);

    // Zeroing the encoder should not be necessary and only creates confusion. Before a match the
    // climber should be fully retracted, which will zero the encode on the limit switch.
    // Zero the encoder so that zero is the full retracted start position
//    m_winch_speed_controller->SetSelectedSensorPosition(0, RC::kTalonPidIdx);

    // Initialise the brake solenoid
    m_brake_solenoid = new frc::Solenoid(RC::kPneumaticsWinchBrakeSolenoidId);
}

void Winch::Shutdown() {
    std::cout << "Winch::Shutdown()\n";
}

void Winch::Periodic()
{
    frc::SmartDashboard::PutNumber("Winch Current", m_winch_speed_controller->GetOutputCurrent());        
    frc::SmartDashboard::PutNumber("Winch Output", m_winch_speed_controller->GetMotorOutputPercent());         
    double winch_speed_native = m_winch_speed_controller->GetSelectedSensorVelocity(RC::kTalonPidIdx);
    double winch_speed_rpm =   KoalafiedUtilities::TalonSRXCtreVelocityNativeToRpm(winch_speed_native);
    frc::SmartDashboard::PutNumber("Winch Speed RPM", winch_speed_rpm);
    frc::SmartDashboard::PutNumber("Winch Position Inch", GetWinchPositionInch());
    frc::SmartDashboard::PutNumber("Winch Encoder Position",m_winch_speed_controller->GetSelectedSensorPosition());

    Faults talon_faults;
    m_winch_speed_controller->GetFaults(talon_faults);
    frc::SmartDashboard::PutBoolean("Winch FLimit", talon_faults.ForwardSoftLimit);
    frc::SmartDashboard::PutBoolean("Winch RLimit", talon_faults.ReverseSoftLimit);

    bool forward_limit = m_winch_speed_controller->GetSensorCollection().IsFwdLimitSwitchClosed();
    frc::SmartDashboard::PutBoolean("Winch FHardLimit", forward_limit);
    bool reverse_limit = m_winch_speed_controller->GetSensorCollection().IsRevLimitSwitchClosed();
    frc::SmartDashboard::PutBoolean("Winch RHardLimit", reverse_limit);
   
    frc::SmartDashboard::PutBoolean("Winch Brake", !m_brake_solenoid->Get());

    // Code for possible winch adjustment for oscillation of the generator switch
    // DriveBase& drive_base = DriveBase::GetInstance();
    // int16_t acceleration_xyz[3];
    // drive_base.GetPigeonIMU()->GetBiasedAccelerometer(acceleration_xyz);
    // frc::SmartDashboard::PutNumber("Accelerometer X", acceleration_xyz[0]);
    // frc::SmartDashboard::PutNumber("Accelerometer Y", acceleration_xyz[1]);
    // frc::SmartDashboard::PutNumber("Accelerometer Z", acceleration_xyz[2]);
}


//==============================================================================
// Operations

void Winch::ManualDriveWinch(double percentage_speed) {
    m_winch_speed_controller->Set(ControlMode::PercentOutput, percentage_speed);
}

double Winch::GetWinchPositionInch() {
    int encoder_count = m_winch_speed_controller->GetSelectedSensorPosition(RC::kTalonPidIdx);
	double revolutions = (double)encoder_count / (double)RC::kCtreEnocderCounts;
	double winch_circumference_inch = RC::kWinchDiameterInch * M_PI;
	return winch_circumference_inch * revolutions;
}

void Winch::BrakeOn() {
    m_brake_solenoid->Set(false);

    // Make sure the winch is not driving if the brake is on 
    m_winch_speed_controller->Set(ControlMode::PercentOutput, 0.0);
}

void Winch::BrakeOff() {
    m_brake_solenoid->Set(true);
}

void Winch::TestDriveWinch(frc::Joystick* joystick) {
    // Drive the winch with the right Y axis, applying a normal dead zone and a 50% speed limit
    double joystick_value = joystick->GetRawAxis(RC::kJoystickRightYAxis);
    if (fabs(joystick_value) < RC::kJoystickDeadzone) joystick_value = 0.0;
    joystick_value *= 0.5;
    m_winch_speed_controller->Set(ControlMode::PercentOutput, joystick_value);

    // Turn the brake off with POV up and on with POV down
    switch (joystick->GetPOV(0)) {
        case RC::kJoystickPovUp:   BrakeOff(); break;
        case RC::kJoystickPovDown: BrakeOn(); break;
    }
}
